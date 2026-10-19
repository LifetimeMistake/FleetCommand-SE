using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using Havok;
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
        public void Autopilot_Turn_UsesStoppingPredictionRatherThanSubtractingRate()
        {
            // At 0.35 rad the source rate is 0.78125. With a small closing spin
            // its stopping test still permits that complete rate; it does not
            // subtract a fitted multiple of the current angular velocity.
            var command = TurnOneStepWithSpin(0.35, 0.01);
            Assert.That(command.Length(), Is.EqualTo(0.78125).Within(0.00001));
        }

        [Test]
        public void Autopilot_Turn_OppositeHeadingStillCommandsRotation()
        {
            var ship = Fc.Spawn(AutopilotCraft("opposite"), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.AddWaypoint(rc.GetPosition() - rc.WorldMatrix.Forward * 1000, "behind");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            Assert.That(ship.Grid.Autopilot.LastControlTorque.Length(), Is.EqualTo(2).Within(0.00001),
                "Quaternion aim error must handle a 180-degree turn where the forward cross product vanishes.");
        }

        [Test]
        public void Autopilot_Turn_RespectsMountedRemoteForward()
        {
            var mount = Quaternion.CreateFromAxisAngle(Vector3.Up, (float)Math.PI / 2);
            var ship = Fc.Spawn(new ShipTemplate("mounted-remote", 2.5f, new[] {
                new TemplateBlock("RC", Vector3I.Zero, 1000, 2.5f, kind: BlockKind.RemoteControl, orientation: mount),
                new TemplateBlock("Gyro", Vector3I.One, 100, 2.5f, kind: BlockKind.Gyro, gyroForce: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.AddWaypoint(rc.GetPosition() + rc.WorldMatrix.Forward * 1000, "ahead");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            Assert.That(ship.Grid.Autopilot.LastControlTorque, Is.EqualTo(Vector3.Zero),
                "Default Forward follows the mounted remote's orientation, not the grid's forward axis.");
        }

        [TestCase(0.5, false)]
        [TestCase(1.0, true)]
        public void OneWayFinalWaypoint_UsesDestinationRadiusAndStopsAutopilot(double distance, bool flying)
        {
            var ship = Fc.Spawn(AutopilotCraft("arrival"), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.AddWaypoint(rc.GetPosition() + Vector3D.UnitZ * distance, "destination");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            Assert.That(ship.Grid.Autopilot.FlightActive, Is.EqualTo(flying),
                "A large-grid final OneWay waypoint uses gridSize*0.25 (0.625 m), not gridSize*3.");
            Assert.That(rc.IsAutoPilotEnabled, Is.EqualTo(flying));
        }

        [Test]
        public void Autopilot_SpeedLimitDoesNotInstantlyClampExistingVelocity()
        {
            var ship = Fc.Spawn(AutopilotCraft("overspeed"),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(0, 0, 10)));
            var rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(0, 0, 5000), "target");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            // ComputeAiThrust uses -velocity/0.5; available thrust exceeds this
            // demand, so one 1/60-second physics interval removes exactly 10/30 m/s.
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(10.0 * (1 - 2.0 / 60)).Within(0.00001),
                "Remote speed limits gate throttle, not the body's physical velocity.");
        }

        [Test]
        public void Autopilot_ZeroDriveCommandStillAppliesBraking()
        {
            var ship = Fc.Spawn(AutopilotCraft("braking"),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(0, 0, 25)));
            var rc = Remote(ship);
            rc.AddWaypoint(new Vector3D(0, 0, 5000), "target");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 25;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            Assert.That(ship.Grid.Autopilot.LastAutoPilotThrust.Z, Is.LessThan(0),
                "At the speed gate, a zero drive command still runs ComputeAiThrust's velocity dampening.");
            Assert.That(ship.Body.LinearVelocity.Z, Is.LessThan(25));
        }

        [Test]
        public void Autopilot_UsesRemotePositionForTargetDelta()
        {
            var blocks = new List<TemplateBlock> {
                new TemplateBlock("RC", new Vector3I(10, 0, 0), 1000, 2.5f, kind: BlockKind.RemoteControl),
                new TemplateBlock("Balance", new Vector3I(-10, 0, 0), 1000, 2.5f),
            };
            blocks.AddRange(AllAxisThrusters(10000, 3));
            var ship = Fc.Spawn(new ShipTemplate("offset-reference", 2.5f, blocks), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.AddWaypoint(rc.GetPosition() + new Vector3D(0, 0, 100), "straight");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.Step();
            Assert.That(ship.Grid.Autopilot.LastAutoPilotThrust.X, Is.Zero,
                "A waypoint directly ahead of an offset remote must not introduce lateral thrust toward the COM.");
            Assert.That(ship.Grid.Autopilot.LastAutoPilotThrust.Z, Is.GreaterThan(0));
        }

        [Test]
        public void Autopilot_EnablingRestoresSharedDampeners()
        {
            var ship = Fc.Spawn(AutopilotCraft("dampeners"), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.DampenersOverride = false;
            rc.AddWaypoint(new Vector3D(0, 0, 500), "target");
            rc.FlightMode = FlightMode.OneWay;
            rc.SetAutoPilotEnabled(true);
            Assert.That(rc.DampenersOverride, Is.True,
                "OnSetAutoPilotEnabled calls ResetShipControls, enabling grid dampeners.");
            rc.SetAutoPilotEnabled(false);
            Assert.That(rc.DampenersOverride, Is.True, "Disabling autopilot does not restore the previous dampener setting.");
        }

        [Test]
        public void Autopilot_CommandWakesSleepingSmallGrid()
        {
            var ship = Fc.Spawn(new ShipTemplate("sleeping-autopilot", 0.5f, new[] {
                new TemplateBlock("RC", Vector3I.Zero, 1000, 0.5f, kind: BlockKind.RemoteControl),
                new TemplateBlock("Engine", new Vector3I(2, 0, 0), 1000, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.DampenersOverride = false;
            Fc.Step(600);
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.IsActive, Is.False, "The regression must start from a sleeping body.");
            rc.AddWaypoint(new Vector3D(100, 0, 0), "target");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = 3;
            rc.SetAutoPilotEnabled(true);
            Fc.RunFor(1);
            Assert.That(ship.Body.PositionWorld.X, Is.GreaterThan(1), "Autopilot must move a previously sleeping small grid.");
            Assert.That(ship.Body.LinearVelocity.X, Is.GreaterThan(1));
        }

        [Test]
        public void SingleWaypoint_RequiresOneWayFlightMode()
        {
            var ship = Fc.Spawn(AutopilotCraft("mode"), SpawnState.At(Vector3D.Zero));
            var rc = Remote(ship);
            rc.SetAutoPilotEnabled(true);
            Assert.That(rc.IsAutoPilotEnabled, Is.False, "No waypoints cannot enable autopilot.");
            rc.AddWaypoint(new Vector3D(0, 0, 500), "target");
            rc.SetAutoPilotEnabled(true);
            Assert.That(rc.IsAutoPilotEnabled, Is.False, "Default Patrol requires more than one waypoint.");
            rc.FlightMode = FlightMode.OneWay;
            rc.SetAutoPilotEnabled(true);
            Assert.That(rc.IsAutoPilotEnabled, Is.True);
            rc.SetAutoPilotEnabled(false);
            Assert.That(rc.IsAutoPilotEnabled, Is.False);
        }

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
            rc.FlightMode = FlightMode.OneWay;
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
        public void Autopilot_SpeedLimit_GatesAccelerationWithinOneFrameOvershoot()
        {
            SimShip ship = Fc.Spawn(
                AutopilotCraft("cap"),
                SpawnState.At(Vector3D.Zero));
            SimRemoteControl rc = Remote(ship);
            rc.AddWaypoint(rc.GetPosition() + rc.WorldMatrix.Forward * 20000, "far");
            rc.SpeedLimit = 25f;
            rc.FlightMode = FlightMode.OneWay;
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
            double frameAcceleration = 1.2e6 / ship.Grid.Body.Mass / 60;
            Assert.That(peak, Is.LessThanOrEqualTo(25 + frameAcceleration + 0.001),
                "Throttle is gated at SpeedLimit; one powered interval can overshoot by force*dt/mass.");
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
            rc.FlightMode = FlightMode.OneWay;
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

        // — waypoint-advance radius: the game's default gridSize · 3 (MyAutopilotComponent:1915) —

        // The default arrival radius is the game's GetWaypointAdvanceDistance fallback,
        // gridSize · 3 (MyAutopilotComponent.cs:1915): 7.5 m for a 2.5 m large grid. The
        // profile's AutopilotArrivalDistance defaults 0 (unconfigured) so this grid-size default
        // applies. An enabled autopilot with a target within that radius advances past it.

        [Test]
        public void Autopilot_AdvanceRadius_DefaultIsGridSizeTimesThree()
        {
            SimShip ship = Fc.Spawn(AutopilotCraft("ar"), SpawnState.At(Vector3D.Zero)); // large grid, 2.5 m
            SimRemoteControl rc = Remote(ship);
            // First waypoint just inside the 7.5 m large-grid default radius; second far.
            Vector3D near = rc.GetPosition() + new Vector3D(0, 0, 7.0);
            Vector3D far = new Vector3D(0, 0, 5000);
            rc.AddWaypoint(near, "near");
            rc.AddWaypoint(far, "far");
            rc.SpeedLimit = 40f;
            rc.FlightMode = FlightMode.OneWay;
            rc.SetAutoPilotEnabled(true);

            Fc.Step(); // the near waypoint is within gridSize·3 → reached on the first compute

            Assert.That((rc.CurrentWaypoint.Coords - far).Length(), Is.LessThan(1e-6),
                "A waypoint within the default gridSize·3 radius must be considered reached and advance to the next.");
        }

        [Test]
        public void Autopilot_AdvanceRadius_JustBeyondDefault_NotReached()
        {
            SimShip ship = Fc.Spawn(AutopilotCraft("ar2"), SpawnState.At(Vector3D.Zero)); // large grid
            SimRemoteControl rc = Remote(ship);
            // An intermediate waypoint just beyond the 7.5 m default radius is not reached;
            // the autopilot keeps flying toward it.
            Vector3D wp = rc.GetPosition() + new Vector3D(0, 0, 8.0);
            rc.AddWaypoint(wp, "target");
            rc.AddWaypoint(new Vector3D(0, 0, 5000), "destination");
            rc.SpeedLimit = 40f;
            rc.FlightMode = FlightMode.OneWay;
            rc.SetAutoPilotEnabled(true);

            Fc.Step();

            Assert.That(ship.Grid.Autopilot.FlightActive, Is.True,
                "A waypoint beyond the default radius must still be flown toward.");
            Assert.That((rc.CurrentWaypoint.Coords - wp).Length(), Is.LessThan(1e-6),
                "A waypoint beyond the default radius must not advance past.");
        }

        // The turn's response now follows the game's adaptive rotation-speed constants, deadzone,
        // and angular-authority feedback from MyAutopilotComponent.UpdateGyro (:2439-2500) and the
        // flight-movement definition (EntityComponents.sbc), NOT the old hand-tuned 0.8/0.5
        // proportional law. Each test reads the first-tick commanded turn (Autopilot.LastControlTorque),
        // which is computed from the spawn geometry before any thrust torque disturbs it.

        [Test]
        public void Autopilot_Turn_BelowMinThreshold_CommandsMinRotationSpeed()
        {
            // At/below MinRotationSpeedThresholdRadians (0.2) the game commands MinRotationSpeed
            // (0.5) — half the gyro's unit control sphere (MyAutopilotComponent.cs:2455-2458, and
            // EntityComponents.sbc MinRotationSpeed). The old law's proportional gain is gone.
            Vector3 cmd = TurnOneStep(0.10);
            Assert.That(Math.Abs(cmd.Y) + Math.Abs(cmd.Z) + Math.Abs(cmd.X), Is.GreaterThan(0f),
                "A remaining turn must command torque along the error axis.");
            Assert.That(cmd.Length(), Is.EqualTo(0.5).Within(0.02),
                "At a small residual the turn rate must be the game's MinRotationSpeed (0.5 rad).");
        }

        [Test]
        public void Autopilot_Turn_BetweenThresholds_InterpolatesRotationSpeed()
        {
            // Between the 0.2 and 1.0 rad thresholds the rotation speed ramps linearly: at 0.35
            // rad it is 0.5 + (0.35-0.2)/(1.0-0.2) * 1.5 = 0.78125 (MyAutopilotComponent.cs:2463-2469).
            Vector3 cmd = TurnOneStep(0.35);
            Assert.That(cmd.Length(), Is.EqualTo(0.78125).Within(0.02),
                "Between thresholds the turn rate must be the game's linear rotation-speed ramp.");
        }

        [Test]
        public void Autopilot_Turn_AtOrAboveMaxThreshold_PreservesSourceRateBeforeTorqueClamp()
        {
            // At/above MaxRotationSpeedTresholdRadians (1.0) the game commands MaxRotationSpeed
            // (2.0). UpdateGyro preserves this command; UpdateGyros later clamps each
            // torque component to ±1, rather than clamping this rate to a unit sphere
            // (MyAutopilotComponent.cs:2459-2462, EntityComponents.sbc MaxRotationSpeed).
            Vector3 cmd = TurnOneStep(1.5);
            Assert.That(cmd.Length(), Is.EqualTo(2.0).Within(1e-3),
                "At a large residual UpdateGyro must preserve the source's 2 rad/s command.");
        }

        [Test]
        public void Autopilot_Turn_BelowDeadzone_CommandsNoTorque()
        {
            // Below RotationDeadzone (0.03 rad) the game returns no torque — the turn is
            // considered aligned and stops (MyAutopilotComponent.cs:2474-2478,
            // EntityComponents.sbc RotationDeadzone).
            Vector3 cmd = TurnOneStep(0.01);
            Assert.That(cmd.Length(), Is.LessThan(1e-6),
                "Below the 0.03 rad deadzone the turn must command no torque.");
        }

        [Test]
        public void Autopilot_Turn_PureAxisStoppingRatioRetainsSourceNaNBranch()
        {
            // The source divides velocity by the predicted velocity change per axis.
            // For this pure-axis case, unused components contain 0/0 and Max() is NaN;
            // UpdateGyro explicitly permits the full command on that branch.
            Vector3 cmd = TurnOneStepWithSpin(0.10, 2.0);
            Assert.That(cmd.Length(), Is.EqualTo(0.5).Within(0.00001),
                "Preserve the game's NaN stopping-ratio branch rather than inventing a scalar damping rule.");
        }

        // — helpers —

        private static SimRemoteControl Remote(SimShip ship)
        {
            // The template's first controller-kind block is the remote.
            return (SimRemoteControl)ship.Controller;
        }

        /// <summary>Spawns the turn craft at the origin, heads it at a far waypoint placed
        /// <paramref name="offsetRad"/> from its forward, enables the autopilot, runs one step,
        /// and returns the first-tick commanded turn (<see cref="AutopilotSystem.LastControlTorque"/>).
        /// First-tick the grid is at rest, so angular authority leaves the adaptive rotation-speed
        /// command intact and the turn response reflects the game's constants and deadzone.</summary>
        private Vector3 TurnOneStep(double offsetRad)
        {
            SimShip ship = Fc.Spawn(AutopilotCraft("turn"), SpawnState.At(Vector3D.Zero));
            SetupTurnWaypoint(ship, offsetRad);
            Fc.Step();
            return ship.Grid.Autopilot.LastControlTorque;
        }

        /// <summary>Like <see cref="TurnOneStep"/> but the grid also spawns already rotating at
        /// <paramref name="spinRadPerSec"/> about the world error axis, so the first tick
        /// exercises the angular-authority feedback (the turn must coast while the grid's own
        /// rotation closes the angle).</summary>
        private Vector3 TurnOneStepWithSpin(double offsetRad, double spinRadPerSec)
        {
            // Probe the identity forward so the world error axis is known, then re-spawn the
            // turn craft with angular velocity along it.
            SimShip probe = Fc.Spawn(AutopilotCraft("probe"), SpawnState.At(Vector3D.Zero));
            Vector3D fwd = probe.Grid.WorldMatrix.Forward;
            Vector3D perp = Vector3D.CalculatePerpendicularVector(fwd);
            Vector3D targetDir = Vector3D.Normalize(fwd * Math.Cos(offsetRad) + perp * Math.Sin(offsetRad));
            Vector3 axisWorld = (Vector3)Vector3D.Normalize(Vector3D.Cross(fwd, targetDir));

            SimShip ship = Fc.Spawn(AutopilotCraft("turn"), SpawnState.At(Vector3D.Zero)
                .WithAngularVelocity(axisWorld * (float)spinRadPerSec));
            SetupTurnWaypoint(ship, offsetRad);
            Fc.Step();
            return ship.Grid.Autopilot.LastControlTorque;
        }

        /// <summary>Adds a waypoint <paramref name="offsetRad"/> from the ship's forward (far
        /// enough that its direction is effectively fixed) and enables the autopilot.</summary>
        private static void SetupTurnWaypoint(SimShip ship, double offsetRad)
        {
            SimRemoteControl rc = (SimRemoteControl)ship.Controller;
            Vector3D fwd = ship.Grid.WorldMatrix.Forward;
            Vector3D perp = Vector3D.CalculatePerpendicularVector(fwd);
            Vector3D targetDir = Vector3D.Normalize(fwd * Math.Cos(offsetRad) + perp * Math.Sin(offsetRad));
            rc.AddWaypoint(rc.GetPosition() + targetDir * 1000.0, "target");
            rc.SpeedLimit = 40f;
            rc.FlightMode = FlightMode.OneWay;
            rc.SetAutoPilotEnabled(true);
        }

        private static void AssertVec3D(Vector3D actual, Vector3D expected, string message)
        {
            Assert.That((actual - expected).Length(), Is.LessThan(1e-6), message);
        }

        /// <summary>A craft with a remote control, a strong gyro, and six-axis thrusters so the
        /// autopilot can translate and turn in any direction. The whole craft is mirror-symmetric
        /// about the origin (RC mirrored by Bal, gyro G mirrored by Bal2, and each thruster
        /// positioned on ITS own axis through the origin), so the grid CoM sits at the origin and
        /// every fired thruster's force line passes through the CoM. With per-thruster
        /// block-position forces (each thruster acts at its own block, MyThrusterBlockThrustComponent),
        /// this gives clean line-thrust with no block-offset torque, so the gyro alone steers.</summary>
        private static ShipTemplate AutopilotCraft(string name)
        {
            var blocks = new List<TemplateBlock>
            {
                new TemplateBlock("Hull", Vector3I.Zero, 2000f, 2.5f),
                new TemplateBlock("RC", new Vector3I(0, 0, 1), 400f, 2.5f, kind: BlockKind.RemoteControl),
                new TemplateBlock("Bal", new Vector3I(0, 0, -1), 400f, 2.5f), // mirrors the RC
                new TemplateBlock("G", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.Gyro, gyroForce: 5e6f),
                new TemplateBlock("Bal2", new Vector3I(-2, 0, 0), 400f, 2.5f), // mirrors the gyro
            };
            blocks.AddRange(AllAxisThrusters(1.2e6f, 3));

            // The autopilot steers with its gyro; thrusters translate along their own axis
            // through the CoM (no block-offset torque).
            return new ShipTemplate(name, 2.5f, blocks, maxLinearVelocity: 120f, maxAngularVelocity: 12f);
        }

        /// <summary>Six thrusters, one per body axis, each mounted on ITS OWN axis through the CoM
        /// (origin) so its force is collinear with the lever arm and produces no block-offset
        /// torque. forward = -axis to push +axis (force = -forward).</summary>
        private static IEnumerable<TemplateBlock> AllAxisThrusters(float max, int arm)
        {
            yield return new TemplateBlock("PX", new Vector3I(arm, 0, 0), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: max);
            yield return new TemplateBlock("NX", new Vector3I(-arm, 0, 0), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Right, maxThrust: max);
            yield return new TemplateBlock("PY", new Vector3I(0, arm, 0), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Down, maxThrust: max);
            yield return new TemplateBlock("NY", new Vector3I(0, -arm, 0), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Up, maxThrust: max);
            yield return new TemplateBlock("PZ", new Vector3I(0, 0, arm), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Forward, maxThrust: max);
            yield return new TemplateBlock("NZ", new Vector3I(0, 0, -arm), 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Backward, maxThrust: max);
        }

    }
}
