using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;
using static FleetCommand.Test.Shared.TestHelpers;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (, /): the fixed thruster
    /// block (<see cref="SimThrust"/>, real <see cref="IMyThrust"/>) and
    /// the grid thrust aggregation (<see cref="ForceAggregator"/>) — the 
    /// boundary. The tests prove the linear impulse response in vacuum and
    /// in uniform gravity, asymmetric and missing-axis aggregation (by data,
    /// no craft-type branches), power scaling through the bus's
    /// <see cref="SimFunctionalBlock.DeliveredRatio"/> (single source of
    /// truth), and that the ONE resultant force is applied at the grid COM
    /// — so an off-center fixed thruster produces zero unintended torque
    /// (the verified application point, MyThrusterBlockThrustComponent.cs:201).
    /// Directions are game 1:1 (MyThrust.cs:188, 222, 276): a template's
    /// thrust direction is the block's grid-body FORWARD (exhaust) axis and
    /// the delivered force is applied along its negation
    /// (ThrustForce = -ThrustForwardVector), so e.g. a forward of
    /// Vector3I.Forward (-Z) pushes the grid along +Z.
    /// Expectations are analytic (F t / m, exact shares), never derived
    /// from the same Havok result (no circular validation).
    /// </summary>
    [TestFixture]
    public class ThrusterConformanceTests : FleetTestFixture
    {
        // — linear impulse response: vacuum —

        [Test]
        public void Override_ProducesAnalyticLinearImpulse_InVacuum()
        {
            // The thruster's forward (exhaust) is -Z (Vector3I.Forward in
            // this build); the game applies force = -forward, pushing +Z.
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("vac", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Forward, maxThrust: 60000f)),
                SpawnState.At(Vector3D.Zero));
            IMyThrust thruster = (IMyThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = 45000f;

            Fc.RunFor(1.0); // 60 fixed steps

            float m = ship.Grid.Body.Mass;
            float expectedV = 45000f / m; // F·t / m, t = 1 s

            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(expectedV).Within(expectedV * 0.01),
                "A fired thruster must produce the analytic impulse F·t/m along its push axis.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(0).Within(1e-3));

            // Delivered force tracks the override at full effectiveness and
            // full power, and the force applied through the COM must not
            // rotate the grid.
            Assert.That(((SimThrust)thruster).CurrentThrust, Is.EqualTo(45000f).Within(1e-2),
                "CurrentThrust must be the delivered force from the last applied interval.");
            Assert.That(ship.Body.AngularVelocity.Length(), Is.LessThan(1e-3),
                "A thruster firing through the grid COM must not induce rotation.");
        }

        [Test]
        public void GridThrustDirection_IsBlockForward_AndForceIsOpposite()
        {
            // Game 1:1 contract: GridThrustDirection returns the block's
            // grid-body forward (exhaust) axis, and the applied force is
            // its negation (MyThrust.cs:276; ThrustForce =
            // -ThrustForwardVector, MyThrust.cs:188). A thruster with
            // forward -Z (Forward) must push +Z; one with forward +Z
            // (Backward) must push -Z.
            SimShip fwd = Fc.Spawn(
                HullPlusThrusters("dir-fwd", hullMass: 2000f,
                    Thruster("F", new Vector3I(2, 0, 0), Vector3I.Forward, maxThrust: 50000f)),
                SpawnState.At(Vector3D.Zero));
            SimShip bwd = Fc.Spawn(
                HullPlusThrusters("dir-bwd", hullMass: 2000f,
                    Thruster("B", new Vector3I(2, 0, 0), Vector3I.Backward, maxThrust: 50000f)),
                SpawnState.At(new Vector3D(0, 0, 60)));

            IMyThrust f = (IMyThrust)fwd.TerminalSystem.GetBlockWithName("F");
            IMyThrust b = (IMyThrust)bwd.TerminalSystem.GetBlockWithName("B");
            Assert.That(f.GridThrustDirection, Is.EqualTo(Vector3I.Forward),
                "GridThrustDirection must be the thruster's grid-body forward (exhaust) axis.");
            Assert.That(b.GridThrustDirection, Is.EqualTo(Vector3I.Backward));

            f.ThrustOverride = 50000f;
            b.ThrustOverride = 50000f;

            Fc.RunFor(1.0);

            Assert.That(fwd.Body.LinearVelocity.Z, Is.GreaterThan(0),
                "forward -Z must push the grid along +Z (force = -forward).");
            Assert.That(bwd.Body.LinearVelocity.Z, Is.LessThan(0),
                "forward +Z must push the grid along -Z (force = -forward).");
        }

        // — linear impulse response: uniform gravity —

        [Test]
        public void Override_ExactlyBalancingWeight_HoversIn_UniformGravity()
        {
            Fc.World.SetGravity(SimGravityField.Uniform(new Vector3(0, -9.81f, 0)));
            // forward = -Y (Vector3I.Down), so the grid is pushed +Y.
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("hover", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Down, maxThrust: 90000f)),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(0, -2, 0)));
            IMyThrust thruster = (IMyThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = ship.Grid.Body.Mass * 9.81f; // exactly the weight

            Fc.RunFor(1.0);

            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(-2.0).Within(0.02),
                "Thrust equal to the weight must exactly cancel gravity: the initial velocity is preserved.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-3));
        }

        [Test]
        public void Override_TwiceWeight_AcceleratesUpward_In_UniformGravity()
        {
            Fc.World.SetGravity(SimGravityField.Uniform(new Vector3(0, -9.81f, 0)));
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("climb", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Down, maxThrust: 120000f)),
                SpawnState.At(Vector3D.Zero));
            IMyThrust thruster = (IMyThrust)ship.TerminalSystem.GetBlockWithName("T");
            float m = ship.Grid.Body.Mass;
            thruster.ThrustOverride = 2f * m * 9.81f; // net acceleration +9.81 upward

            Fc.RunFor(1.0);

            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(9.81).Within(0.3),
                "Over-pushing thrust must leave a net upward acceleration of g after 1 s.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-3));
        }

        // — asymmetric / missing thruster aggregation (by data, no craft-type branches) —

        [Test]
        public void GroupDemand_AsymmetricThrusters_SumsByGridBodyDirection()
        {
            SimShip ship = Fc.Spawn(AsymmetricCraft(), SpawnState.At(Vector3D.Zero));
            float m = ship.Grid.Body.Mass;

            ship.Grid.ForceAggregator.SetGroupDemand(new Vector3(1, 0, 0)); // full forward along +X

            Fc.RunFor(1.0);

            // Main (60 kN) + Aux (40 kN) both push +X: the net must be their sum.
            float expectedV = 100000f / m;
            AssertVector3(ship.Grid.ForceAggregator.LastNetForce, new Vector3(100000f, 0, 0), 1e-2,
                "The aggregator must sum delivered thrust by grid-body direction.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(expectedV).Within(expectedV * 0.02),
                "The summed resultant must produce the analytic impulse.");
            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-3));

            Assert.That(((SimThrust)ship.TerminalSystem.GetBlockWithName("Main")).CurrentThrust, Is.EqualTo(60000f).Within(1e-2));
            Assert.That(((SimThrust)ship.TerminalSystem.GetBlockWithName("Aux")).CurrentThrust, Is.EqualTo(40000f).Within(1e-2));
            Assert.That(((SimThrust)ship.TerminalSystem.GetBlockWithName("Down")).CurrentThrust, Is.EqualTo(0f),
                "A thruster on an undemanded axis must deliver nothing.");
        }

        [Test]
        public void GroupDemand_MissingAxis_ProducesNoForce_OrMotion()
        {
            // The craft has no -X thrusters; demanding brake along -X must
            // produce no force and no motion (missing axes are supported by
            // data, not branches).
            SimShip ship = Fc.Spawn(AsymmetricCraft(), SpawnState.At(Vector3D.Zero));

            ship.Grid.ForceAggregator.SetGroupDemand(new Vector3(-1, 0, 0));
            Fc.RunFor(1.0);

            AssertVector3(ship.Grid.ForceAggregator.LastNetForce, Vector3.Zero, 1e-4,
                "A demanded axis with no thrusters must yield a zero resultant.");
            Assert.That(ship.Body.LinearVelocity.LengthSquared(), Is.LessThan(1e-6),
                "With no thruster on the demanded axis the grid must not move.");
        }

        [Test]
        public void GroupDemand_OverrideTakesPrecedence_OverGroupDemand()
        {
            // Overridden thrusters answer their override, not the group
            // demand (game: IsOverridden takes precedence over IsUsed) —
            // the autopilot's per-thruster command path.
            SimShip ship = Fc.Spawn(AsymmetricCraft(), SpawnState.At(Vector3D.Zero));
            float m = ship.Grid.Body.Mass;
            SimThrust main = (SimThrust)ship.TerminalSystem.GetBlockWithName("Main");
            main.ThrustOverride = 30000f;

            ship.Grid.ForceAggregator.SetGroupDemand(new Vector3(1, 0, 0));
            Fc.RunFor(1.0);

            float expectedV = (30000f + 40000f) / m; // Main's override + Aux's group share
            Assert.That(main.CurrentThrust, Is.EqualTo(30000f).Within(1e-2),
                "An overridden thruster must deliver its override, not the group demand.");
            Assert.That(((SimThrust)ship.TerminalSystem.GetBlockWithName("Aux")).CurrentThrust, Is.EqualTo(40000f).Within(1e-2));
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(expectedV).Within(expectedV * 0.02),
                "The net must be override-precedent thrust plus the remaining group thrust.");
        }

        [Test]
        public void GroupDemand_TwoAxisDemand_AggregatesIntoResultant()
        {
            SimShip ship = Fc.Spawn(AsymmetricCraft(), SpawnState.At(Vector3D.Zero));
            float m = ship.Grid.Body.Mass;

            ship.Grid.ForceAggregator.SetGroupDemand(new Vector3(1, -1, 0)); // +X full, down (-Y) full

            Fc.RunFor(1.0);

            Vector3 net = ship.Grid.ForceAggregator.LastNetForce;
            AssertVector3(net, new Vector3(100000f, -80000f, 0), 1e-2,
                "The resultant must be the vector sum of the demanded axes.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(100000f / m).Within((100000f / m) * 0.02));
            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(-80000f / m).Within((80000f / m) * 0.02));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-3));
        }

        // — power scaling (reads the bus's DeliveredRatio — single source of truth) —

        [Test]
        public void PowerShortfall_ScalesDeliveredThrust_AndResponse()
        {
            ShipTemplate template = PoweredCraft(maxOutput: 5f, powerDemand: 10f, batteryStored: 100f);
            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            SimShip full = Fc.Spawn(PoweredCraft(maxOutput: 20f, powerDemand: 10f, batteryStored: 100f),
                SpawnState.At(new Vector3D(0, 0, 60)));
            SimThrust engine = (SimThrust)ship.TerminalSystem.GetBlockWithName("Engine");
            SimThrust fullEngine = (SimThrust)full.TerminalSystem.GetBlockWithName("Engine");
            engine.ThrustOverride = 40000f;
            fullEngine.ThrustOverride = 40000f;
            float m = ship.Grid.Body.Mass;

            Fc.RunFor(1.0);

            Assert.That(engine.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "The phase-4 bus allocation must give the 10 MW engine a 0.5 ratio from the 5 MW battery.");
            Assert.That(fullEngine.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));
            Assert.That(engine.CurrentThrust, Is.EqualTo(20000f).Within(1e-2),
                "Delivered thrust must scale by the bus's delivered ratio (40000 x 0.5).");
            Assert.That(fullEngine.CurrentThrust, Is.EqualTo(40000f).Within(1e-2));

            float expectedV = 20000f / m;
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(expectedV).Within(expectedV * 0.02),
                "The response must reflect the power-scaled delivered thrust, not the override.");
        }

        [Test]
        public void SharedPowerShortfall_ScalesEachThruster_ByTierRatio()
        {
            // Two equal-priority 4 MW engines share a 6 MW battery: the
            // tier receives ratio 0.75, and each delivered force is scaled
            // by it — shared shortfall through the power profile.
            var template = new ShipTemplate("shared", 2.5f, new[]
            {
                new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 2.5f,
                    kind: BlockKind.Battery, maxOutput: 6f, maxStoredPower: 100f),
                new TemplateBlock("Hull", new Vector3I(2, 0, 0), 2000f, 2.5f),
                new TemplateBlock("E1", new Vector3I(4, 0, 0), 400f, 2.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left,
                    maxThrust: 60000f, powerDemand: 4f),
                new TemplateBlock("E2", new Vector3I(5, 0, 0), 400f, 2.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left,
                    maxThrust: 60000f, powerDemand: 4f),
            });
            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            SimThrust e1 = (SimThrust)ship.TerminalSystem.GetBlockWithName("E1");
            SimThrust e2 = (SimThrust)ship.TerminalSystem.GetBlockWithName("E2");
            e1.ThrustOverride = 50000f;
            e2.ThrustOverride = 50000f;
            float m = ship.Grid.Body.Mass;

            Fc.RunFor(1.0);

            Assert.That(e1.DeliveredRatio, Is.EqualTo(0.75f).Within(1e-6),
                "Equal-priority engines must share the shortfall fairly (6/8).");
            Assert.That(e2.DeliveredRatio, Is.EqualTo(0.75f).Within(1e-6));
            Assert.That(e1.CurrentThrust, Is.EqualTo(37500f).Within(1e-2));
            Assert.That(e2.CurrentThrust, Is.EqualTo(37500f).Within(1e-2));

            float expectedV = (2f * 37500f) / m;
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(expectedV).Within(expectedV * 0.02),
                "Both scaled delivered forces must feed the single resultant.");
        }

        // — zero unintended torque from an off-center fixed thruster —
        // The net force is applied at the grid COM (source-verified),
        // so block position must not torque the grid.

        [Test]
        public void OffCenterThruster_ProducesZeroUnintendedTorque()
        {
            // A single thruster mounted far from the hull's center: if the
            // force were applied at the block there would be a large arm;
            // applied at the grid COM it must translate without rotating.
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("lopsided", hullMass: 4000f,
                    Thruster("T", new Vector3I(8, 6, 4), Vector3I.Left, maxThrust: 80000f)),
                SpawnState.At(Vector3D.Zero));
            IMyThrust thruster = (IMyThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = 80000f;

            Fc.RunFor(1.0);

            float m = ship.Grid.Body.Mass;
            float expectedV = 80000f / m;
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(expectedV).Within(expectedV * 0.02),
                "The off-center thruster must still produce its full linear impulse.");
            Assert.That(ship.Body.LinearVelocity.Y, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.LinearVelocity.Z, Is.EqualTo(0).Within(1e-3));
            Assert.That(ship.Body.AngularVelocity.Length(), Is.LessThan(1e-3),
                "Fixed grid thrusters must generate no torque from block position (force applied at the grid COM).");
            Assert.That(ship.Body.PositionWorld.Z, Is.EqualTo(0).Within(1e-3));
        }

        // — SimThrust interface surface —

        [Test]
        public void Thruster_InterfaceSurface_AndOverrideClamping()
        {
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("surf", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Right, maxThrust: 50000f, effectiveness: 0.8f)),
                SpawnState.At(Vector3D.Zero));
            SimThrust block = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            IMyThrust t = block;

            Assert.That(t.MaxThrust, Is.EqualTo(50000f));
            Assert.That(t.MaxEffectiveThrust, Is.EqualTo(40000f).Within(1e-3),
                "MaxEffectiveThrust must be template maximum x effectiveness profile.");
            Assert.That(t.GridThrustDirection, Is.EqualTo(Vector3I.Right),
                "GridThrustDirection must be the block's grid-body forward (exhaust) axis from the template.");
            Assert.That(t.CurrentThrust, Is.EqualTo(0f));
            Assert.That(t.CurrentThrustPercentage, Is.EqualTo(0f));

            // Override clamps to [0, MaxThrust]; percentage clamps to [0, 1];
            // both share one storage (game: single m_thrustOverride).
            t.ThrustOverride = 90000f;
            Assert.That(t.ThrustOverride, Is.EqualTo(50000f), "Override must clamp to MaxThrust.");
            Assert.That(t.ThrustOverridePercentage, Is.EqualTo(1f));

            t.ThrustOverridePercentage = 0.5f;
            Assert.That(t.ThrustOverridePercentage, Is.EqualTo(0.5f));
            Assert.That(t.ThrustOverride, Is.EqualTo(25000f).Within(1e-3),
                "Override and override percentage must share one value.");
            Assert.That(t.ThrustOverride, Is.GreaterThan(0f));

            t.ThrustOverridePercentage = 2f;
            Assert.That(t.ThrustOverridePercentage, Is.EqualTo(1f), "Percentage must clamp to 1.");
            t.ThrustOverride = -10f;
            Assert.That(t.ThrustOverride, Is.EqualTo(0f), "Negative override must clamp to 0.");
        }

        [Test]
        public void Thruster_CurrentThrust_OnlyAfterPhysicsInterval()
        {
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("ph", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Right, maxThrust: 50000f)),
                SpawnState.At(Vector3D.Zero));
            SimThrust thruster = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = 30000f;

            Assert.That(thruster.CurrentThrust, Is.EqualTo(0f),
                "CurrentThrust is the applied interval's delivered force — nothing before the first step.");

            Fc.Step();
            Assert.That(thruster.CurrentThrust, Is.EqualTo(30000f).Within(1e-2),
                "After the physics interval (tick phase 8), CurrentThrust must be the delivered force.");
        }

        [Test]
        public void Thruster_WorkingState_Gates_DeliveredForce()
        {
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("gate", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Right, maxThrust: 50000f)),
                SpawnState.At(Vector3D.Zero));
            SimThrust thruster = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = 40000f;

            // Disabled: the enable gate cuts the delivered force.
            thruster.Enabled = false;
            Fc.Step();
            Assert.That(thruster.CurrentThrust, Is.EqualTo(0f), "A disabled thruster must not deliver thrust.");
            Assert.That(ship.Body.LinearVelocity.LengthSquared(), Is.LessThan(1e-8));

            // Non-functional: same gate.
            thruster.Enabled = true;
            thruster.IsFunctional = false;
            Fc.Step();
            Assert.That(thruster.CurrentThrust, Is.EqualTo(0f), "A non-functional thruster must not deliver thrust.");

            // Restored: delivers again.
            thruster.IsFunctional = true;
            Fc.Step();
            Assert.That(thruster.CurrentThrust, Is.EqualTo(40000f).Within(1e-2));
        }

        [Test]
        public void Thruster_ZeroEffectiveness_ProducesNoEffectiveThrust()
        {
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("eff", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Right, maxThrust: 50000f, effectiveness: 0f)),
                SpawnState.At(Vector3D.Zero));
            SimThrust thruster = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            thruster.ThrustOverride = 50000f;

            Assert.That(((IMyThrust)thruster).MaxEffectiveThrust, Is.EqualTo(0f),
                "An environmentally ineffective thruster has no effective thrust.");
            Fc.Step();
            Assert.That(thruster.CurrentThrust, Is.EqualTo(0f),
                "Effectiveness zero must gate the delivered force (no environmental effect).");
            Assert.That(ship.Body.LinearVelocity.LengthSquared(), Is.LessThan(1e-8));
        }

        [Test]
        public void Thruster_InheritedUnimplementedMembers_ThrowNamed()
        {
            SimShip ship = Fc.Spawn(
                HullPlusThrusters("ns", hullMass: 2000f,
                    Thruster("T", new Vector3I(2, 0, 0), Vector3I.Right, maxThrust: 50000f)),
                SpawnState.At(Vector3D.Zero));
            IMyThrust t = (IMyThrust)ship.TerminalSystem.GetBlockWithName("T");
            IMyTerminalBlock block = (IMyTerminalBlock)t;

            AssertThrowsNamed(() => t.DetailedInfo, "IMyTerminalBlock.DetailedInfo");
            AssertThrowsNamed(() => t.CustomNameWithFaction, "IMyTerminalBlock.CustomNameWithFaction");
            AssertThrowsNamed(() => block.GetActionWithName("OnOff"), "IMyTerminalBlock.GetActionWithName()");
        }

        // — helpers —

        private static void AssertVector3(Vector3 actual, Vector3 expected, double tolerance, string message)
        {
            Assert.Multiple((Action)(() =>
            {
                Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance), message + " (X)");
                Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance), message + " (Y)");
                Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance), message + " (Z)");
            }));
        }

        /// <summary>A heavy hull plus any thruster blocks.</summary>
        private static ShipTemplate HullPlusThrusters(string name, float hullMass, params TemplateBlock[] thrusters)
        {
            var blocks = new List<TemplateBlock>
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 2.5f),
            };
            blocks.AddRange(thrusters);
            return new ShipTemplate(name, 2.5f, blocks);
        }

        /// <summary>
        /// A fixed thruster template block with thruster params. The
        /// direction is the block's grid-body FORWARD (exhaust) axis;
        /// the grid is pushed opposite it (force = -forward).
        /// </summary>
        private static TemplateBlock Thruster(string name, Vector3I cell, Vector3I forward,
            float maxThrust, float effectiveness = 1f, float powerDemand = 0f)
        {
            return new TemplateBlock(name, cell, 400f, 2.5f, kind: BlockKind.FixedThruster,
                thrustDirection: forward, maxThrust: maxThrust,
                effectiveness: effectiveness, powerDemand: powerDemand);
        }

        /// <summary>
        /// The asymmetric fixture: thrusters pushing +X (60 kN Main, 40 kN
        /// Aux) and -Y (80 kN Down), and NO push on -X or Z — so
        /// asymmetric and missing axes are expressed as data, with no
        /// craft-type branch. Forwards are -X (Left, force +X) and +Y
        /// (Up, force -Y) respectively.
        /// </summary>
        private static ShipTemplate AsymmetricCraft()
        {
            return HullPlusThrusters("asym", hullMass: 2000f,
                Thruster("Main", new Vector3I(2, 0, 0), Vector3I.Left, maxThrust: 60000f),
                Thruster("Aux", new Vector3I(3, 0, 0), Vector3I.Left, maxThrust: 40000f),
                Thruster("Down", new Vector3I(0, 2, 0), Vector3I.Up, maxThrust: 80000f));
        }

        /// <summary>A battery-powered +X engine with the given bus shortfall.</summary>
        private static ShipTemplate PoweredCraft(float maxOutput, float powerDemand, float batteryStored)
        {
            return CraftTemplates.PoweredDriverCraft("powered", maxOutput, batteryStored, new TemplateBlock(
                "Engine", new Vector3I(4, 0, 0), 400f, 2.5f,
                kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left,
                maxThrust: 50000f, powerDemand: powerDemand));
        }
    }
}
