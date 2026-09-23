using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;
using static FleetCommand.Test.Shared.TestHelpers;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the ship controller surface, with the load-bearing frame conventions
    /// (world-frame velocities, gravity, body-to-world pose) verified against ground
    /// truth.</summary>
    [TestFixture]
    public class ShipControllerConformanceTests : FleetTestFixture
    {
        // — wiring: the template-designated controller —

        [Test]
        public void Spawn_DesignatesControllerFromTemplate()
        {
            SimShip withController = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(Vector3D.Zero));
            Assert.That(withController.Controller, Is.Not.Null, "A controller-kind block must become SimShip.Controller.");
            Assert.That(withController.Controller, Is.InstanceOf<IMyShipController>());
            Assert.That(withController.Controller.CustomName, Is.EqualTo("Pilot"));

            SimShip withoutController = Fc.Spawn(
                new ShipTemplate("hull", 2.5f, new[]
                {
                    new TemplateBlock("Payload", new Vector3I(0, 0, 0), 400f, 2.5f),
                }),
                SpawnState.At(Vector3D.Zero));
            Assert.That(withoutController.Controller, Is.Null, "A template with no controller-kind block must yield no controller.");
        }

        [Test]
        public void Spawn_RemoteControlAndCockpit_AreFoundAsControllers()
        {
            SimShip remote = Fc.Spawn(RemoteTemplate("R"), SpawnState.At(Vector3D.Zero));
            Assert.That(remote.Controller, Is.Not.Null);
            Assert.That(remote.Controller, Is.InstanceOf<IMyRemoteControl>());

            SimShip cockpit = Fc.Spawn(CockpitTemplate("K"), SpawnState.At(Vector3D.Zero));
            Assert.That(cockpit.Controller, Is.Not.Null);
            Assert.That(cockpit.Controller, Is.InstanceOf<IMyCockpit>());
        }

        // — pose: WorldMatrix is body-to-world —

        [Test]
        public void Controller_WorldMatrix_IsBodyToWorldPose()
        {
            // Spawn the grid at a non-identity rotation so body axes and world
            // axes differ; the controller's pose must still be built from the
            // grid's (body's) world matrix — i.e. its rotation columns are
            // the body axes expressed in world coordinates.
            QuaternionD rot = QuaternionD.CreateFromAxisAngle(new Vector3D(0, 0, 1), Math.PI / 2);
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(new Vector3D(10, 20, 30), rot));

            IMyShipController controller = ship.Controller;
            MatrixD gridWorld = ship.Grid.WorldMatrix;

            AssertVector3D(controller.WorldMatrix.Translation, ship.Grid.WorldMatrix.Translation, 1e-3,
                "Controller pose must match the grid's (body's) world pose.");
            AssertVector3D(controller.WorldMatrix.Right, gridWorld.Right, 1e-3, "WorldMatrix rotation column X must be the body X axis in world coords.");
            AssertVector3D(controller.WorldMatrix.Up, gridWorld.Up, 1e-3, "WorldMatrix rotation column Y must be the body Y axis in world coords.");
            AssertVector3D(controller.WorldMatrix.Forward, gridWorld.Forward, 1e-3, "WorldMatrix rotation column Z must be the body Z axis in world coords.");

            // Body-to-world, not world-to-body: with a 90°-about-Z body
            // rotation the body +X axis points along world +Y.
            MatrixD m = controller.WorldMatrix;
            Assert.That(Math.Abs(m.Right.X), Is.LessThan(1e-3));
            Assert.That(Math.Abs(m.Right.Y - 1.0), Is.LessThan(1e-3));
        }

        // — CenterOfMass in world coordinates —

        [Test]
        public void Controller_CenterOfMass_WorldCoords_MatchesAnalytic()
        {
            var template = new ShipTemplate("asymmetric", 2.5f, new[]
            {
                new TemplateBlock("Pilot", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.ShipController),
                new TemplateBlock("Payload", new Vector3I(3, 0, 0), 900f, 2.5f, inventoryMass: 100f),
            });

            QuaternionD rot = QuaternionD.CreateFromAxisAngle(new Vector3D(0, 1, 0), 0.7);
            SimShip ship = Fc.Spawn(template, SpawnState.At(new Vector3D(50, 0, 25), rot));
            IMyShipController controller = ship.Controller;

            // Analytic ground truth (never from Havok): weighted center of the
            // template's dry + inventory mass boxes, transformed to world.
            var boxes = new List<MassProperties.MassElement>();
            foreach (TemplateBlock b in template.Blocks)
            {
                boxes.Add(new MassProperties.MassElement { Mass = b.DryMass, HalfExtents = b.HalfExtents, CenterLocal = b.CenterLocal });
                if (b.InventoryMass > 0f)
                {
                    boxes.Add(new MassProperties.MassElement { Mass = b.InventoryMass, HalfExtents = b.HalfExtents, CenterLocal = b.CenterLocal });
                }
            }
            MassProperties analytic = MassProperties.AnalyticFromBoxes(boxes);
            Vector3D analyticWorld = Vector3D.Transform(new Vector3D(analytic.CenterOfMass), ship.Grid.WorldMatrix);

            AssertVector3D(controller.CenterOfMass, analyticWorld, 1e-2,
                "CenterOfMass must be the template's analytic COM in world coordinates.");

            // And it must agree with the body's own world-visible COM.
            Vector3D bodyWorld = Vector3D.Transform(new Vector3D(ship.Grid.Body.CenterOfMassLocal), ship.Grid.WorldMatrix);
            AssertVector3D(controller.CenterOfMass, bodyWorld, 1e-2,
                "CenterOfMass must read from the grid body's COM (world coords).");
        }

        // — mass —

        [Test]
        public void Controller_CalculateShipMass_ReportsBodyMass()
        {
            var template = new ShipTemplate("massive", 2.5f, new[]
            {
                new TemplateBlock("Pilot", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.ShipController),
                new TemplateBlock("Payload", new Vector3I(3, 0, 0), 900f, 2.5f, inventoryMass: 100f),
            });

            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            MyShipMass mass = ship.Controller.CalculateShipMass();

            Assert.That(mass.TotalMass, Is.EqualTo(1400f).Within(1e-2),
                "TotalMass must be the full physical (dry + inventory) mass.");
            Assert.That(mass.PhysicalMass, Is.EqualTo(1400f).Within(1e-2),
                "No inventory multiplier in the simulator: physical mass equals total mass.");
            Assert.That(mass.BaseMass, Is.EqualTo(1300f).Within(1e-2),
                "BaseMass must be the physical mass minus inventory cargo.");
            Assert.That(ship.Grid.Body.Mass, Is.EqualTo(mass.TotalMass).Within(1e-2),
                "The controller must read mass from the grid body.");
        }

        // — natural gravity: world frame, same vector as applied —

        [Test]
        public void Controller_GetNaturalGravity_WorldFrame_MatchesApplied()
        {
            Vector3 g = new Vector3(0, -9.81f, 0);
            Fc.World.SetGravity(SimGravityField.Uniform(g));
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(new Vector3D(5, 6, 7)));

            Vector3D natural = ship.Controller.GetNaturalGravity();
            AssertVector3D(natural, new Vector3D(g), 1e-4,
                "GetNaturalGravity must return the world-frame field vector — the same one the tick applies to the body.");
            AssertVector3D(ship.Controller.GetTotalGravity(), new Vector3D(g), 1e-4,
                "Total gravity equals natural gravity when there is no artificial gravity.");
            Assert.That(ship.Controller.GetArtificialGravity(), Is.EqualTo(Vector3D.Zero),
                "No artificial-gravity generators: artificial gravity is zero.");
        }

        [Test]
        public void Controller_GetNaturalGravity_PointField_MatchesBodyApplicationPoint()
        {
            // In a non-uniform field, GetNaturalGravity must be the vector
            // applied to the body, i.e. sampled at the body's position —
            // not a hypothetical elsewhere.
            Fc.World.SetGravity(SimGravityField.Point(
                new Vector3D(0, 0, 0), intensity: 1.0, minRadius: 5.0, maxRadius: 500.0));
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(new Vector3D(20, 0, 0)));

            Vector3D reported = ship.Controller.GetNaturalGravity();
            Vector3D applied = Fc.World.Gravity.GetNaturalGravity(ship.Grid.Body.PositionWorld);

            AssertVector3D(reported, new Vector3D(applied), 1e-5,
                "GetNaturalGravity must be the exact vector the tick applies to the body (same point, same field).");
        }

        // — velocities: WORLD frame, no body transform —

        [Test]
        public void Controller_GetShipVelocities_WorldFrame_NoBodyTransform()
        {
            // The critical frame convention: angular velocity is reported in
            // the world frame straight from the body (MyShipController.cs:3233),
            // with no body-frame transform. Spawn at a rotated orientation so
            // body-frame and world-frame genuinely differ.
            Vector3D w = new Vector3D(2, 3, 4);
            Vector3 angW = new Vector3(0.5f, 1f, -0.25f);
            QuaternionD rot = QuaternionD.CreateFromAxisAngle(new Vector3D(0, 0, 1), Math.PI / 2);
            SimShip ship = Fc.Spawn(
                ControllerTemplate("C"),
                SpawnState.At(new Vector3D(1, 2, 3), rot).WithVelocity(w).WithAngularVelocity(angW));

            MyShipVelocities v = ship.Controller.GetShipVelocities();

            AssertVector3D(v.LinearVelocity, w, 1e-3, "Linear velocity must be the world-frame body velocity.");
            AssertVector3D(v.AngularVelocity, new Vector3D(angW), 1e-3,
                "Angular velocity must be the WORLD-frame body angular velocity, not body-frame.");

            // Ground truth: exactly what the body reports.
            AssertVector3D(v.LinearVelocity, ship.Body.LinearVelocity, 1e-3, "Must match the rigid body's world linear velocity.");
            AssertVector3D(v.AngularVelocity, new Vector3D(ship.Body.AngularVelocity), 1e-3, "Must match the rigid body's world angular velocity.");

            // And it must differ from the body-frame value — proving no
            // R^T transform was applied. For a 90°-about-Z rotation,
            // R^T · (0.5, 1, -0.25) = (1, -0.5, -0.25), which is not the
            // reported value.
            Vector3D bodyFrame = Vector3D.TransformNormal(new Vector3D(angW), MatrixD.Transpose(ship.Grid.WorldMatrix));
            Assert.That(Vector3D.Distance(v.AngularVelocity, bodyFrame), Is.GreaterThan(0.5),
                "Reported angular velocity must stay in the world frame (differ from body-frame).");
        }

        [Test]
        public void Controller_GetShipSpeed_IsLinearMagnitude()
        {
            SimShip ship = Fc.Spawn(
                ControllerTemplate("C"),
                SpawnState.At(Vector3D.Zero, MatrixD.CreateRotationZ(0.5)).WithVelocity(new Vector3D(3, 4, 0)));

            Assert.That(ship.Controller.GetShipSpeed(), Is.EqualTo(5.0).Within(1e-3),
                "GetShipSpeed must be the magnitude of the world linear velocity.");
        }

        // — control flags and scenario control inputs —

        [Test]
        public void Controller_ControlFlags_AndIndicators_ScenarioSettable()
        {
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(Vector3D.Zero));
            IMyShipController c = ship.Controller;
            SimShipController block = (SimShipController)c;

            Assert.That(c.DampenersOverride, Is.True, "Dampeners default on, matching the game.");
            c.DampenersOverride = false;
            Assert.That(c.DampenersOverride, Is.False);

            Assert.That(c.ControlThrusters, Is.True);
            c.ControlThrusters = false;
            Assert.That(c.ControlThrusters, Is.False);

            // MoveIndicator / RotationIndicator / RollIndicator / IsUnderControl
            // are read-only on the interface (user input); the simulator's
            // scenario setters live on the concrete SimShipController.
            block.MoveIndicator = new Vector3(1, 0, 0.5f);
            Assert.That(c.MoveIndicator, Is.EqualTo(new Vector3(1, 0, 0.5f)));

            block.RotationIndicator = new Vector2(0.2f, -0.4f);
            Assert.That(c.RotationIndicator, Is.EqualTo(new Vector2(0.2f, -0.4f)));

            block.RollIndicator = 0.9f;
            Assert.That(c.RollIndicator, Is.EqualTo(0.9f).Within(1e-6));

            Assert.That(c.IsUnderControl, Is.False);
            block.IsUnderControl = true;
            Assert.That(c.IsUnderControl, Is.True);
        }

        [Test]
        public void Controller_CanControlShip_FollowsWorkingState()
        {
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(Vector3D.Zero));
            SimShipController block = (SimShipController)ship.TerminalSystem.GetBlockWithName("Pilot");

            Assert.That(ship.Controller.CanControlShip, Is.True, "A working controller can control the ship.");

            block.Enabled = false;
            Assert.That(ship.Controller.CanControlShip, Is.False,
                "Disabling the controller must remove its ability to control the ship.");

            block.Enabled = true;
            Assert.That(ship.Controller.CanControlShip, Is.True);
        }

        [Test]
        public void Controller_IsMainCockpit_DemotesSiblings()
        {
            SimShip ship = Fc.Spawn(TwoControllerTemplate("D"), SpawnState.At(Vector3D.Zero));
            SimShipController primary = (SimShipController)ship.TerminalSystem.GetBlockWithName("Primary");
            SimShipController secondary = (SimShipController)ship.TerminalSystem.GetBlockWithName("Secondary");

            Assert.That(primary.IsMainCockpit, Is.False);
            Assert.That(secondary.IsMainCockpit, Is.False);

            primary.IsMainCockpit = true;
            Assert.That(primary.IsMainCockpit, Is.True);
            Assert.That(secondary.IsMainCockpit, Is.False);

            secondary.IsMainCockpit = true;
            Assert.That(secondary.IsMainCockpit, Is.True, "Newly designated main must be the one set.");
            Assert.That(primary.IsMainCockpit, Is.False, "Setting a new main must demote the previous one.");
        }

        // — unimplemented controller members throw naming the member —

        [Test]
        public void Controller_UnimplementedMembers_ThrowNamed()
        {
            SimShip ship = Fc.Spawn(ControllerTemplate("C"), SpawnState.At(Vector3D.Zero));
            IMyShipController c = ship.Controller;

            AssertThrowsNamed(() => c.HasWheels, "IMyShipController.HasWheels");
            AssertThrowsNamed(() => c.HandBrake, "IMyShipController.HandBrake");
            AssertThrowsNamed(() => c.ControlWheels, "IMyShipController.ControlWheels");
            AssertThrowsNamed(() => { c.ControlWheels = true; return null; }, "IMyShipController.ControlWheels");
            AssertThrowsNamed(() => c.ShowHorizonIndicator, "IMyShipController.ShowHorizonIndicator");
            AssertThrowsNamed(() => { c.ShowHorizonIndicator = true; return null; }, "IMyShipController.ShowHorizonIndicator");
            AssertThrowsNamed(() => c.TryGetPlanetPosition(out var _), "IMyShipController.TryGetPlanetPosition()");
            AssertThrowsNamed(() => c.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out var _), "IMyShipController.TryGetPlanetElevation()");
        }

        // — remote control —

        [Test]
        public void RemoteControl_ProvidesIdentity_AndShipControllerBehavior()
        {
            SimShip ship = Fc.Spawn(RemoteTemplate("RC"), SpawnState.At(Vector3D.Zero));
            IMyRemoteControl remote = ship.Controller as IMyRemoteControl;

            Assert.That(remote, Is.Not.Null);
            Assert.That(remote.CustomName, Is.EqualTo("RC"));
            Assert.That(remote.CubeGrid, Is.SameAs(ship.Grid));
            Assert.That(remote, Is.InstanceOf<IMyFunctionalBlock>());

            // Full ship-controller surface works through the remote.
            ship.SetState(ship.Body.WithLinearVelocity(new Vector3D(0, 10, 0)).WithAngularVelocity(new Vector3(0.5f, 0, 0)));
            Assert.That(remote.GetShipSpeed(), Is.EqualTo(10.0).Within(1e-3));
            AssertVector3D(remote.GetShipVelocities().LinearVelocity, new Vector3D(0, 10, 0), 1e-3,
                "Linear velocity must read through the remote as the world-frame body velocity.");

            // Configuration surface: speed limit and autopilot flag.
            remote.SpeedLimit = 42f;
            Assert.That(remote.SpeedLimit, Is.EqualTo(42f));
            Assert.That(remote.IsAutoPilotEnabled, Is.False);
            remote.SetAutoPilotEnabled(true);
            Assert.That(remote.IsAutoPilotEnabled, Is.True);
        }

        [Test]
        public void RemoteControl_WaypointMembers_ThrowNamed()
        {
            SimShip ship = Fc.Spawn(RemoteTemplate("RC"), SpawnState.At(Vector3D.Zero));
            IMyRemoteControl remote = (IMyRemoteControl)ship.Controller;

            AssertThrowsNamed(() => remote.FlightMode, "IMyRemoteControl.FlightMode");
            AssertThrowsNamed(() => remote.Direction, "IMyRemoteControl.Direction");
            AssertThrowsNamed(() => remote.WaitForFreeWay, "IMyRemoteControl.WaitForFreeWay");
            AssertThrowsNamed(() => remote.GetNearestPlayer(out var _), "IMyRemoteControl.GetNearestPlayer()");
            AssertThrowsNamed(() => { remote.SetCollisionAvoidance(true); return null; }, "IMyRemoteControl.SetCollisionAvoidance()");
            AssertThrowsNamed(() => { remote.SetDockingMode(true); return null; }, "IMyRemoteControl.SetDockingMode()");
        }

        // — cockpit —

        [Test]
        public void Cockpit_ProvidesIdentity_AndSurfaceProvider()
        {
            SimShip ship = Fc.Spawn(CockpitTemplate("K"), SpawnState.At(Vector3D.Zero));
            IMyCockpit cockpit = ship.Controller as IMyCockpit;

            Assert.That(cockpit, Is.Not.Null);
            Assert.That(cockpit.CustomName, Is.EqualTo("K"));
            Assert.That(cockpit, Is.InstanceOf<IMyTextSurfaceProvider>());

            IMyTextSurfaceProvider provider = cockpit;
            Assert.That(provider.SurfaceCount, Is.EqualTo(0), "No modeled text surfaces on a cockpit this stage.");
            Assert.That(provider.GetSurface(0), Is.Null);

            // Ship-controller surface still works.
            Assert.That(cockpit.GetShipSpeed(), Is.EqualTo(0.0).Within(1e-6));
        }

        [Test]
        public void Cockpit_OxygenMembers_ThrowNamed()
        {
            SimShip ship = Fc.Spawn(CockpitTemplate("K"), SpawnState.At(Vector3D.Zero));
            IMyCockpit cockpit = (IMyCockpit)ship.Controller;

            AssertThrowsNamed(() => cockpit.OxygenCapacity, "IMyCockpit.OxygenCapacity");
            AssertThrowsNamed(() => cockpit.OxygenFilledRatio, "IMyCockpit.OxygenFilledRatio");
        }

        // — helpers —

        private static void AssertVector3D(Vector3D actual, Vector3D expected, double tolerance, string message)
        {
            Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance), message + " (X)");
            Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance), message + " (Y)");
            Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance), message + " (Z)");
        }

        private static ShipTemplate ControllerTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("Pilot", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.ShipController),
                });
        }

        private static ShipTemplate RemoteTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("RC", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.RemoteControl),
                });
        }

        private static ShipTemplate CockpitTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("K", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.Cockpit),
                });
        }

        private static ShipTemplate TwoControllerTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("Primary", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.ShipController),
                    new TemplateBlock("Secondary", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.ShipController),
                });
        }
    }
}
