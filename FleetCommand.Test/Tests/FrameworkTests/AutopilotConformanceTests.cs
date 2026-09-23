using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>End-to-end conformance of the remote-control autopilot and SpeedLimit, with
    /// analytic expectations.</summary>
    [TestFixture]
    public class AutopilotConformanceTests : FleetTestFixture
    {
        [Test]
        public void Autopilot_Off_Coasts_NoAutoCommand()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("off"),
                SpawnState.At(Vector3D.Zero));
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(0, 0, 500), "target");
            // autopilot stays off.
            Fc.RunFor(1.0);
            Assert.That(ship.Grid.Autopilot.FlightActive, Is.False, "Autopilot off must not fly.");
            Assert.That(ship.Body.LinearVelocity.Length(), Is.LessThan(1e-3),
                "Autopilot off must coast (no auto thrust) from rest.");
        }

        [Test]
        public void Autopilot_On_TranslatesTowardWaypoint()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("fly"),
                SpawnState.At(Vector3D.Zero)); // forward (+Z) already points at the waypoint
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(0, 0, 4000), "target");
            rc.SpeedLimit = 40f;
            rc.SetAutoPilotEnabled(true);

            double d0 = (ship.Grid.Body.PositionWorld - new Vector3D(0, 0, 4000)).Length();
            Fc.RunFor(1.0);

            double d1 = (ship.Grid.Body.PositionWorld - new Vector3D(0, 0, 4000)).Length();
            Assert.That(ship.Grid.Autopilot.FlightActive, Is.True,
                "The autopilot must be actively flying toward the waypoint.");
            Assert.That(ship.Body.LinearVelocity.Z, Is.GreaterThan(0),
                "The autopilot must translate toward the +Z waypoint.");
            Assert.That(d1, Is.LessThan(d0 - 1.0),
                "The ship must move closer to the waypoint (end-to-end translation).");
        }

        [Test]
        public void Autopilot_SpeedLimit_CapsTopSpeed()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("cap"),
                SpawnState.At(Vector3D.Zero));
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(0, 0, 20000), "far");
            rc.SpeedLimit = 25f;
            rc.SetAutoPilotEnabled(true);

            double peak = 0.0;
            for (int i = 0; i < 300; i++) // 5 s, long enough to reach and hold the cap
            {
                Fc.Step();
                double speed = ship.Body.LinearVelocity.Length();
                if (speed > peak)
                {
                    peak = speed;
                }
            }

            Assert.That(peak, Is.GreaterThan(20f),
                "The autopilot must accelerate the ship up near the speed limit.");
            Assert.That(peak, Is.LessThanOrEqualTo(25f * 1.02f + 0.01),
                "SpeedLimit must cap the top speed (the |v|² < maxSpeed² gate + MovementBlockMaxSpeed velocity clamp).");
        }

        [Test]
        public void Autopilot_TurnsTowardWaypoint()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("turn"),
                SpawnState.At(Vector3D.Zero)); // forward +Z; waypoint at +X (90 deg off)
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(5000, 0, 0), "side");
            rc.SpeedLimit = 40f;
            rc.SetAutoPilotEnabled(true);

            // The waypoint is far enough that its direction from the ship
            // is effectively fixed, so the turn can fully converge on it.
            Vector3D waypoint = new Vector3D(5000, 0, 0);
            Vector3D targetDir = Vector3D.Normalize(waypoint - ship.Grid.Body.PositionWorld);
            double dot0 = Vector3D.Dot(ship.Grid.WorldMatrix.Forward, targetDir);
            Fc.RunFor(5.0);
            Vector3D targetDir1 = Vector3D.Normalize(waypoint - ship.Grid.Body.PositionWorld);
            double dot1 = Vector3D.Dot(ship.Grid.WorldMatrix.Forward, targetDir1);
            Assert.That(dot1, Is.GreaterThan(dot0 + 0.05),
                "The autopilot must rotate the grid forward toward the off-axis waypoint (end-to-end turn).");
            Assert.That(dot1, Is.GreaterThan(0.98),
                "The grid must converge to point at the waypoint.");
        }

        [Test]
        public void Autopilot_WaypointMembers_ReadBackWhatWasSet()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("wp"),
                SpawnState.At(Vector3D.Zero));
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(10, 20, 30), "first");
            rc.AddWaypoint(new Vector3D(40, 50, 60), "second");

            var list = new List<MyWaypointInfo>();
            rc.GetWaypointInfo(list);
            Assert.That(list.Count, Is.EqualTo(2), "GetWaypointInfo must return the added waypoints.");
            Assert.That(list[0].Name, Is.EqualTo("first"));
            AssertVec3D(list[0].Coords, new Vector3D(10, 20, 30), "waypoint 0 coords");
            AssertVec3D(list[1].Coords, new Vector3D(40, 50, 60), "waypoint 1 coords");

            Assert.That(rc.CurrentWaypoint.Equals(list[0]), Is.True,
                "CurrentWaypoint must be the first (not yet reached) target.");
            Assert.That(rc.CurrentWaypoint.IsEmpty(), Is.False);

            rc.ClearWaypoints();
            rc.GetWaypointInfo(list);
            Assert.That(list.Count, Is.EqualTo(0), "ClearWaypoints must clear the list.");
            Assert.That(rc.CurrentWaypoint.IsEmpty(), Is.True, "With no waypoints CurrentWaypoint is empty.");
        }

        // — helpers —

        private static SimRemoteControl Remote(SimShip ship)
        {
            // The template's first controller-kind block is the remote.
            return (SimRemoteControl)ship.Controller;
        }

        private static void AssertVec3D(Vector3D actual, Vector3D expected, string message)
        {
            Assert.That((actual - expected).Length(), Is.LessThan(1e-6), message);
        }

        /// <summary>A craft with a remote control, a strong gyro, and six-axis thrusters so the
        /// autopilot can translate and turn in any direction.</summary>
        private static ShipTemplate AutopilotCraft(string name)
        {
            var blocks = new List<TemplateBlock>
            {
                new TemplateBlock("Hull", Vector3I.Zero, 2000f, 2.5f),
                new TemplateBlock("RC", new Vector3I(0, 0, 1), 400f, 2.5f, kind: BlockKind.RemoteControl),
                new TemplateBlock("G", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.Gyro, gyroForce: 5e6f),
            };
            blocks.AddRange(AllAxisThrusters(1.2e6f));
            return new ShipTemplate(name, 2.5f, blocks, maxLinearVelocity: 120f, maxAngularVelocity: 12f);
        }

        private static IEnumerable<TemplateBlock> AllAxisThrusters(float max)
        {
            // forward = -axis to push +axis (force = -forward).
            Vector3I[] pushes = { new Vector3I(1, 0, 0), new Vector3I(-1, 0, 0), new Vector3I(0, 1, 0), new Vector3I(0, -1, 0), new Vector3I(0, 0, 1), new Vector3I(0, 0, -1) };
            string[] names = { "PX", "NX", "PY", "NY", "PZ", "NZ" };
            int idx = 3; // offset so hull/RC/G occupy 0-2
            for (int i = 0; i < pushes.Length; i++)
            {
                yield return new TemplateBlock(names[i], new Vector3I(idx, 0, 0), 100f, 2.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: -pushes[i], maxThrust: max);
                idx++;
            }
        }

    }
}
