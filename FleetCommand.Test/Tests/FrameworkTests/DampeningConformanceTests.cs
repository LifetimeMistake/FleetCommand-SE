using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of inertia dampening in the thrust base path — reproduces
    /// MyEntityThrustComponent.cs:1543-1656 ComputeBaseThrust via the controller route
    /// (MyShipController.cs:1780-1820).</summary>
    [TestFixture]
    public class DampeningConformanceTests : FleetTestFixture
    {
        // — 1. dampening stops a moving ship —

        [Test]
        public void Dampening_StopsMovingShip_MatchesVelocityDeviationLaw()
        {
            SimShip ship = Fc.Spawn(
                DampeningCraft("stop", thrusters: AxialX()),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(5, 0, 0)));
            Control(ship).IsUnderControl = true; // dampeners default on

            float v0 = 5f;
            Fc.Step(); // one fixed 1/60 s interval

            // The game's law: dv = vector4 / 0.5 * m / m * dt =
            // -v * 2 * dt (no gravity, no manual command, full harness).
            float dt = (float)TickClock.FixedStep;
            float expectedDv = -v0 * 2f * dt;
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(v0 + expectedDv).Within(0.002),
                "The first-interval deceleration must match the game's vector4/0.5·mass velocity law (dv = -2·v·dt).");

            // And it comes (near) to rest: time constant ~0.5 s, so after
            // 6 seconds the residual speed is negligible.
            Fc.RunFor(6.0);
            Assert.That(Math.Abs(ship.Body.LinearVelocity.X), Is.LessThan(0.05),
                "A ship with dampeners on and no input must come to rest.");
            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(0).Within(1e-6));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-6));
        }

        // — 2. dampening holds against gravity —

        [Test]
        public void Dampening_HoldsAgainst_Gravity()
        {
            Fc.World.SetGravity(SimGravityField.Uniform(new Vector3(0, -9.81f, 0)));
            SimShip ship = Fc.Spawn(
                DampeningCraft("hover", thrusters: new[]
                {
                    Thrust("U", new Vector3I(0, 1, 0), Vector3I.Down, max: 200000f),   // pushes +Y
                    Thrust("D", new Vector3I(0, -1, 0), Vector3I.Up, max: 200000f),   // pushes -Y
                }),
                SpawnState.At(new Vector3D(0, 50, 0)));
            Control(ship).IsUnderControl = true; // dampeners default on

            Fc.RunFor(2.0);

            // The +g/2 anti-gravity part of vector2 holds altitude: the
            // dampener cancels the full weight, so the ship neither falls
            // nor rises.
            Assert.That(Math.Abs(ship.Body.LinearVelocity.Y), Is.LessThan(0.05),
                "Dampeners on must hold the ship against gravity (no net vertical drift).");
            Assert.That(Math.Abs(ship.Grid.Body.PositionWorld.Y - 50.0), Is.LessThan(0.5),
                "The hovering ship must hold its altitude (the +g/2 anti-gravity term).");
        }

        // — 3. manual control and the dampener yield to each other —

        [Test]
        public void DampeningShare_ScalesWith_ManualControl()
        {
            // No gravity: isolate the dampening term. With velocity +X,
            // the dampener would command -X thrust scaled by (1 - |control|)
            // on that axis.
            SimShip full = Fc.Spawn(
                DampeningCraft("full", thrusters: AxialX()),
                SpawnState.At(new Vector3D(0, 0, 0)).WithVelocity(new Vector3D(6, 0, 0)));
            SimShip half = Fc.Spawn(
                DampeningCraft("half", thrusters: AxialX()),
                SpawnState.At(new Vector3D(0, 0, 30)).WithVelocity(new Vector3D(6, 0, 0)));
            SimShip zero = Fc.Spawn(
                DampeningCraft("zero", thrusters: AxialX()),
                SpawnState.At(new Vector3D(0, 0, 60)).WithVelocity(new Vector3D(6, 0, 0)));

            Control(full).IsUnderControl = true;   // control 0 -> sum0 = 1
            Control(half).IsUnderControl = true;   // -> sum0 = 0.5
            Control(zero).IsUnderControl = true;   // -> sum0 = 0

            Control(half).MoveIndicator = new Vector3(0.5f, 0, 0);   // half command on X
            Control(zero).MoveIndicator = new Vector3(1f, 0, 0);      // full command on X

            Fc.Step(); // one interval to compute the base demand

            Vector3 sf = full.Grid.BaseControl.LastSlowdownThrust;
            Vector3 sh = half.Grid.BaseControl.LastSlowdownThrust;
            Vector3 sz = zero.Grid.BaseControl.LastSlowdownThrust;

            Assert.That(sf.X, Is.LessThan(0), "Full share: the dampener must command -X against +X velocity.");
            Assert.That(sh.X, Is.EqualTo(sf.X * 0.5f).Within(Math.Abs(sf.X) * 0.05),
                "The dampening share must scale as zero = 1 - |controlThrust| (half command -> half dampening).");
            Assert.That(sz.X, Is.EqualTo(0).Within(1e-3),
                "At full manual command on an axis the dampener must contribute nothing there (zero = 0).");
        }

        // — 4. control + slowdown clamped to ±Max·SlowdownFactor —

        [Test]
        public void BaseForce_ClampedTo_MaxTimesSlowdownFactor()
        {
            // A very large velocity makes the dampening demand far exceed the
            // grid's capacity; the delivered base force must be clamped to
            // ±Max·SlowdownFactor on every axis.
            SimShip ship = Fc.Spawn(
                DampeningCraft("clamp", hullMass: 5000f, thrusters: new[]
                {
                    Thrust("F", new Vector3I(1, 0, 0), Vector3I.Left, max: 10000f, slowdownFactor: 0.5f),
                    Thrust("B", new Vector3I(-1, 0, 0), Vector3I.Right, max: 10000f, slowdownFactor: 0.5f),
                }),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(500, 0, 0)));
            Control(ship).IsUnderControl = true;

            Fc.Step();

            float max = 10000f * 0.5f; // MaxEffectiveThrust=Max (eff 1) * SlowdownFactor 0.5
            Assert.That(ship.Grid.BaseControl.LastBaseForce.X, Is.EqualTo(-max).Within(1.0),
                "The dampening demand beyond capacity must be clamped to -Max·SlowdownFactor on the +X-velocity axis.");
        }

        // — 5. MoveIndicator -> ControlThrust in the grid frame; delivered follows aggregation —

        [Test]
        public void MoveIndicator_ProducesControlThrust_AndDelivered_FollowsAggregation()
        {
            // Full +Y command on a grid whose controller is under control.
            // The controller is identity-oriented, so MoveIndicator maps
            // directly to the grid-frame ControlThrust.
            SimShip ship = Fc.Spawn(
                DampeningCraft("ctrl", thrusters: new[]
                {
                    Thrust("T", new Vector3I(0, 0, 2), Vector3I.Down, max: 90000f, eff: 1f), // pushes +Y
                    Thrust("U2", new Vector3I(0, 0, -2), Vector3I.Down, max: 40000f, eff: 0.5f), // pushes +Y, half eff
                }),
                SpawnState.At(Vector3D.Zero));
            Control(ship).IsUnderControl = true;
            Control(ship).MoveIndicator = new Vector3(0, 1, 0); // full up

            Fc.Step();

            // ControlThrust in the grid frame equals the transformed
            // MoveIndicator (identity block orientation).
            Assert.That(ship.Grid.BaseControl.LastControlThrust.Y, Is.EqualTo(1f).Within(1e-6),
                "MoveIndicator must become the grid-frame ControlThrust (the Vector3.Transform route).");

            // Delivered thrust follows the existing group-demand
            // aggregation: full command, effectiveness 1 and 0.5, and the
            // grid is pushed +Y (force -forward).
            Assert.That(ship.Body.LinearVelocity.Y, Is.GreaterThan(0),
                "A full up command must push the grid along +Y (direction applies).");
            SimThrust full = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            SimThrust half = (SimThrust)ship.TerminalSystem.GetBlockWithName("U2");
            Assert.That(full.CurrentThrust, Is.GreaterThan(0),
                "The delivered force must be non-zero for the full-effective thruster.");
            Assert.That(half.CurrentThrust, Is.GreaterThan(0),
                "The half-effective thruster must also fire (effectiveness scales it).");
        }

        // — 6. dampeners off -> coast —

        [Test]
        public void DampenersOff_Coasts_NoSlowdown()
        {
            SimShip ship = Fc.Spawn(
                DampeningCraft("coast", thrusters: AxialX()),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(4, 0, 0)));
            Control(ship).IsUnderControl = true;
            Control(ship).DampenersOverride = false; // no dampening

            Fc.Step();

            Assert.That(ship.Grid.BaseControl.LastSlowdownThrust.X, Is.EqualTo(0).Within(1e-4),
                "With dampeners off there is no slowdown term.");
            // And the ship coasts: velocity (essentially) preserved.
            Fc.RunFor(1.0);
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(4.0).Within(0.05),
                "A ship with dampeners off must coast (only manual control, and none here).");
        }

        // — 7. deterministic across independent worlds —

        [Test]
        public void Dampening_Deterministic_AcrossTwoWorlds()
        {
            (double a, double b) = DeterminismHelper.RunTwice(RunDampening);
            Assert.That(a, Is.EqualTo(b).Within(1e-9),
                "The dampened final speed must be identical across two independent worlds (no wall-clock).");
        }

        private static double RunDampening(FleetTestContext ctx)
        {
            SimShip ship = ctx.Spawn(
                DampeningCraft("det", thrusters: AxialX()),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(3, 0, 0)));
            ((SimShipController)ship.Controller).IsUnderControl = true;
            ctx.RunFor(2.0);
            return ship.Body.LinearVelocity.X;
        }

        // — helpers —

        private static SimShipController Control(SimShip ship)
        {
            return (SimShipController)ship.Controller;
        }

        private static TemplateBlock[] AxialX()
        {
            // Push along +X with forward -X (Left); along -X with
            // forward +X (Right) — force = -forward, so Left pushes +X.
            return new[]
            {
                Thrust("F", new Vector3I(1, 0, 0), Vector3I.Left, max: 120000f),
                Thrust("B", new Vector3I(-1, 0, 0), Vector3I.Right, max: 120000f),
            };
        }

        /// <summary>A craft with a controller at the origin (heavy hull) plus the given thrusters;
        /// dampeners and ControlThrusters on.</summary>
        private static ShipTemplate DampeningCraft(string name, TemplateBlock[] thrusters, float hullMass = 2000f)
        {
            var blocks = new System.Collections.Generic.List<TemplateBlock>
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 2.5f),
                new TemplateBlock("Pilot", new Vector3I(0, 0, 1), 400f, 2.5f, kind: BlockKind.ShipController),
            };
            blocks.AddRange(thrusters);
            return new ShipTemplate(name, 2.5f, blocks, maxLinearVelocity: 120f, maxAngularVelocity: 12f);
        }

        private static TemplateBlock Thrust(string name, Vector3I min, Vector3I forward, float max,
            float eff = 1f, float slowdownFactor = 1f)
        {
            return new TemplateBlock(name, min, 100f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: forward,
                maxThrust: max, effectiveness: eff, powerDemand: 0f, slowdownFactor: slowdownFactor);
        }
    }
}
