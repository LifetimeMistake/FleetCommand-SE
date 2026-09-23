using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

// Obsolete battery aliases are not exercised here, but the obsolete-call
// warnings gate off nothing; keep pragma matched to the power tests.
#pragma warning disable CS0618

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    /// The scheduled mutation surface. Each mutation type applies its
    /// effect, where practical against an analytic expectation (delivered
    /// thrust, the delivered-ratio law, momentum change from an external
    /// impulse, mass change from inventory / add / remove). Block loss and
    /// a missed Update1 happen at the exact scheduled tick, deterministically
    /// across two worlds.
    /// </summary>
    [TestFixture]
    public class ScheduledMutationConformanceTests : FleetTestFixture
    {
        // — scheduled block loss and missed Update1, deterministic () —

        [Test]
        public void ScheduledBlockRemoval_HappensExactlyAtTick_DeterministicAcrossTwoWorlds()
        {
            var (resultA, resultB) = DeterminismHelper.RunTwice(RunBlockLossScenario);

            Assert.That(resultA.PresentThroughTick, Is.EqualTo(resultB.PresentThroughTick));
            Assert.That(resultA.FinalMass, Is.EqualTo(resultB.FinalMass).Within(1e-3));

            // The removal happened exactly at tick 3 and not before.
            Assert.That(resultA.PresentThroughTick, Is.EqualTo(2),
                "The block must be present through tick 2 and removed at tick 3.");
            // Final mass equals the template minus the removed thruster's mass
            // (only the Hull block remains).
            Assert.That(resultA.FinalMass, Is.EqualTo(HullMass).Within(1e-3));
        }

        private const float HullMass = 1000f;
        private const float BlockMass = 500f;

        private sealed class ScenarioA
        {
            public int PresentThroughTick;
            public float FinalMass;
        }

        private static ScenarioA RunBlockLossScenario(FleetTestContext context)
        {
            var result = new ScenarioA();
            SimShip ship = context.Spawn(Rig(), SpawnState.At(Vector3D.Zero));
            context.AtTick(3, c => c.RemoveBlock(ship, "Thruster"));

            for (long tick = 0; tick < 6; tick++)
            {
                // Removal happens during tick 3's phase 2, so the block
                // is present after completing tick 2 but gone after tick 3.
                context.Step();
                bool present = ship.TerminalSystem.GetBlockWithName("Thruster") != null;
                if (present)
                {
                    result.PresentThroughTick = (int)tick;
                }
            }
            result.FinalMass = ship.Grid.Body.Mass;
            return result;
        }

        [Test]
        public void ScheduledMissedUpdate1_HappensAtExactTick_PhysicsStillAdvances_DeterministicAcrossTwoWorlds()
        {
            var (a, b) = DeterminismHelper.RunTwice(RunMissedUpdateScenario);

            Assert.That(a.SeenTicks, Is.EqualTo(b.SeenTicks));
            Assert.That(a.EndTick, Is.EqualTo(b.EndTick));

            // The only skipped Update1 was at tick 2, exactly.
            Assert.That(a.SeenTicks, Is.EqualTo(new List<long> { 0, 1, 3, 4 }),
                "Only tick 2's Update1 is skipped; physics still advanced all five ticks.");
            Assert.That(a.EndTick, Is.EqualTo(5), "Skipping an Update1 must not skip physics.");
        }

        private sealed class ScenarioB
        {
            public List<long> SeenTicks = new List<long>();
            public long EndTick;
        }

        private static ScenarioB RunMissedUpdateScenario(FleetTestContext context)
        {
            var result = new ScenarioB();
            // Schedule the skip as a scenario mutation at tick 2 (the
            // public SkipNextUpdate1 surface); physics still advances.
            context.AtTick(2, c => c.SkipNextUpdate1());
            context.EveryUpdate1(frame => result.SeenTicks.Add(context.Tick));
            context.Step(5);
            result.EndTick = context.Tick;
            return result;
        }

        // — delivered thrust: enable / disable / damage / repair (analytic) —

        [Test]
        public void EnableDisableDamagedRepair_DriveDeliveredThrust()
        {
            SimShip ship = Fc.Spawn(Rig(), SpawnState.At(Vector3D.Zero));
            SimThrust thruster = (SimThrust)ship.TerminalSystem.GetBlockWithName("Thruster");
            Fc.SetGroupDemand(ship, new Vector3(0, 0, 1)); // drive the +Z force axis
            Fc.Step();

            float full = ship.Grid.ForceAggregator.LastNetForce.Length();
            Assert.That(full, Is.GreaterThan(0f), "An enabled, functional thruster must deliver force.");

            // Disable: delivered force drops to zero.
            Fc.SetBlockEnabled(thruster, false);
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(0f), "A disabled thruster delivers no force.");
            Assert.That(thruster.IsWorking, Is.False);

            // Re-enable, then damage: not working, no force.
            Fc.SetBlockEnabled(thruster, true);
            Fc.SetBlockFunctional(thruster, false);
            Fc.Step();
            Assert.That(thruster.IsWorking, Is.False, "A damaged functional block is not working.");
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(0f), "A damaged thruster delivers no force.");

            // Repair: working again, force returns (deterministic envelope).
            Fc.SetBlockFunctional(thruster, true);
            Fc.Step();
            Assert.That(thruster.IsWorking, Is.True);
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(full).Within(1e-3),
                "Repairing the thruster restores the full delivered force.");
        }

        // — delivered thrust: applied external impulse matches momentum change —

        [Test]
        public void ExternalLinearImpulse_MatchesAnalyticMomentumChange()
        {
            SimShip ship = Fc.Spawn(HullRig(1000f), SpawnState.At(Vector3D.Zero));

            Fc.ApplyLinearImpulse(ship, new Vector3(0f, 100f, 0f)); // 100 N·s impulse, 1000 kg -> dv = 0.1 m/s

            Fc.Step();

            Vector3D v = ship.Grid.Body.LinearVelocity;
            Assert.That(v.X, Is.EqualTo(0f).Within(1e-6));
            Assert.That(v.Y, Is.EqualTo(0.1).Within(1e-3), "dv = J / m = 100 / 1000 = 0.1 m/s.");
            Assert.That(v.Z, Is.EqualTo(0f).Within(1e-6));
        }

        // — add / remove block: registration, disposal, and mass (analytic) —

        [Test]
        public void AddBlock_RegistersAndAddsMass()
        {
            SimShip ship = Fc.Spawn(HullRig(HullMass), SpawnState.At(Vector3D.Zero));
            float m0 = ship.Grid.Body.Mass;
            Assert.That(ship.TerminalSystem.GetBlockWithName("NewThruster"), Is.Null);

            var added = new TemplateBlock("NewThruster", new Vector3I(3, 0, 0), BlockMass, 0.5f,
                kind: BlockKind.FixedThruster, thrustDirection: new Vector3I(0, 0, -1), maxThrust: 30000f);

            Fc.AddBlock(ship, added);

            Assert.That(ship.Grid.Body.Mass, Is.EqualTo(m0 + BlockMass).Within(1e-3),
                "Adding a block must add its dry mass to the body.");
            SimThrust t = (SimThrust)ship.TerminalSystem.GetBlockWithName("NewThruster");
            Assert.That(t, Is.Not.Null, "The added block is discoverable through the terminal system.");
            Assert.That(ship.Grid.ForceAggregator.LastNetForce, Is.EqualTo(Vector3.Zero));

            // The added thruster now contributes to delivery (analytic).
            t.ThrustOverride = 30000f;
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(30000f).Within(1e-3),
                "The added thruster delivers its override thrust exactly.");
        }

        [Test]
        public void RemoveBlock_DeregistersDisposesAndRemovesMass()
        {
            SimShip ship = Fc.Spawn(Rig(), SpawnState.At(Vector3D.Zero));
            float m0 = ship.Grid.Body.Mass;
            SimTerminalBlock block = (SimTerminalBlock)ship.TerminalSystem.GetBlockWithName("Thruster");

            Fc.RemoveBlock(ship, "Thruster");

            Assert.That(ship.TerminalSystem.GetBlockWithName("Thruster"), Is.Null,
                "A removed block is no longer discoverable through the terminal system.");
            Assert.That(block.Closed, Is.True, "A removed block is disposed.");
            Assert.That(ship.Grid.Body.Mass, Is.EqualTo(m0 - BlockMass).Within(1e-3),
                "Removing a block must remove its dry mass from the body.");
        }

        // — power ratio change reproduces the  delivered-ratio law —

        [Test]
        public void ElectricalRatioChange_ReproducesDeliveredRatioLaw()
        {
            // Battery supplies 8 MW; a single load demands 8 MW.
            SimShip ship = Fc.Spawn(BatteryRig(8f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            load.PowerDemand = 8f;
            Fc.Step();
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6), "Full supply satisfies the load.");

            // Available electrical ratio 0.5 -> supply 4 MW -> ratio 4/8 = 0.5.
            Fc.SetPowerAvailableRatio(ship, 0.5f);
            Fc.Step();
            Assert.That(load.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "The  ratio law min(1, available/demand) with available = ratio·supply.");
        }

        // — fuel ratio change reproduces the supply reduction —

        [Test]
        public void FuelRatioChange_ReproducesSupplyReduction()
        {
            SimShip ship = Fc.Spawn(ReactorRig(8f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            load.PowerDemand = 8f;
            Fc.Step();
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));

            Fc.SetFuelRatio(ship, "Reactor", 0f);
            Fc.Step();
            Assert.That(ship.Grid.PowerBus.TotalSupply, Is.EqualTo(0f), "No fuel -> no supply.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(0f).Within(1e-6), "Out of fuel starves the load.");
            Assert.That(load.IsPowered, Is.False);

            Fc.SetFuelRatio(ship, "Reactor", 0.5f);
            Fc.Step();
            Assert.That(load.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "Half fuel halves the delivered ratio for a full-demand load.");
        }

        // — atmosphere effectiveness change scales delivered thrust —

        [Test]
        public void AtmosphereChange_ScalesDeliveredThrust()
        {
            SimShip ship = Fc.Spawn(Rig(), SpawnState.At(Vector3D.Zero));
            SimThrust thruster = (SimThrust)ship.TerminalSystem.GetBlockWithName("Thruster");
            thruster.ThrustOverride = 20000f;
            Fc.Step();
            float full = ship.Grid.ForceAggregator.LastNetForce.Length();
            Assert.That(full, Is.GreaterThan(0f));

            Fc.SetAtmosphereEffectiveness(0f); // vacuum
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(0f),
                "Atmosphere effectiveness 0 (vacuum) produces no thrust.");

            Fc.SetAtmosphereEffectiveness(0.5f); // thin atmosphere
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(0.5f * full).Within(1e-3),
                "Delivered thrust scales linearly with atmosphere effectiveness.");
        }

        // — gravity change accelerates the body analytically —

        [Test]
        public void GravityChange_AcceleratesBodyByG()
        {
            SimShip ship = Fc.Spawn(HullRig(1000f), SpawnState.At(Vector3D.Zero));
            var field = SimGravityField.Uniform(new Vector3(0f, -9.81f, 0f));
            Fc.SetGravity(field);

            Fc.Step();
            Vector3D v1 = ship.Grid.Body.LinearVelocity;
            Assert.That(v1.Y, Is.EqualTo(-9.81f * TickClock.FixedStep).Within(1e-4),
                "One step of uniform gravity gives dv = g·dt.");

            // Change gravity to a different direction and magnitude.
            Fc.SetGravity(SimGravityField.Uniform(new Vector3(0f, 0f, -5f)));
            Fc.Step();
            Vector3D v2 = ship.Grid.Body.LinearVelocity;
            Assert.That(v2.Z, Is.EqualTo(-5f * TickClock.FixedStep).Within(1e-4),
                "Changing gravity changes the acceleration exactly.");
        }

        // — inventory mass change reproduces the analytic mass increase —

        [Test]
        public void InventoryMassChange_ReproducesMassIncrease()
        {
            SimShip ship = Fc.Spawn(HullRig(HullMass), SpawnState.At(Vector3D.Zero));
            float m0 = ship.Grid.Body.Mass;

            SimTerminalBlock hull = (SimTerminalBlock)ship.TerminalSystem.GetBlockWithName("Hull");
            Fc.SetInventoryMass(hull, 300f);

            Assert.That(ship.Grid.Body.Mass, Is.EqualTo(m0 + 300f).Within(1e-3),
                "Changing inventory mass adds it to the body mass.");
        }

        // — guidance target snapshot: replace and stop refreshing —

        [Test]
        public void TargetSnapshot_ReplaceAndStopRefreshing()
        {
            SimTargetSnapshot first = SimTargetSnapshot.At(new Vector3D(10, 0, 0));
            Fc.ReplaceTargetSnapshot("t1", first);

            SimTargetSnapshot read;
            Assert.That(Fc.TryGetTargetSnapshot("t1", out read), Is.True);
            Assert.That(read.Position, Is.EqualTo(new Vector3D(10, 0, 0)));
            Assert.That(read.IsStale, Is.False, "A freshly replaced snapshot is not stale.");

            // Replace with a new value.
            SimTargetSnapshot second = SimTargetSnapshot.Moving(new Vector3D(50, 0, 0), new Vector3D(0, 5, 0));
            Fc.ReplaceTargetSnapshot("t1", second);
            Assert.That(Fc.TryGetTargetSnapshot("t1", out read), Is.True);
            Assert.That(read.Position, Is.EqualTo(new Vector3D(50, 0, 0)), "The snapshot was replaced.");

            // Stop refreshing: retained but stale.
            Fc.StopRefreshingTarget("t1");
            Assert.That(Fc.TryGetTargetSnapshot("t1", out read), Is.True);
            Assert.That(read.IsStale, Is.True, "Stopping refresh marks the snapshot stale.");
            Assert.That(read.Position, Is.EqualTo(new Vector3D(50, 0, 0)), "A stale snapshot keeps its last value.");
        }

        // — change a demanded vector / attitude while the craft is flying/turning —

        [Test]
        public void ChangeDemandedVector_WhileFlying_ChangesDeliveredForce()
        {
            SimShip ship = Fc.Spawn(Rig(), SpawnState.At(Vector3D.Zero));
            Fc.SetGroupDemand(ship, new Vector3(0, 0, 1));
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Z, Is.GreaterThan(0f), "Demand along +Z fires the +Z force axis.");

            // Change the demanded vector while flying: the delivered force follows.
            Fc.SetGroupDemand(ship, Vector3.Zero);
            Fc.Step();
            Assert.That(ship.Grid.ForceAggregator.LastNetForce.Length(), Is.EqualTo(0f),
                "Changing the demanded vector to zero cancels delivered force.");
        }

        [Test]
        public void ChangeDemandedAttitude_WhileTurning_ChangesRequestedRate()
        {
            SimShip ship = Fc.Spawn(GyroRig(), SpawnState.At(Vector3D.Zero));
            SimGyro gyro = (SimGyro)ship.TerminalSystem.GetBlockWithName("Gyro");
            gyro.GyroOverride = true;
            gyro.Yaw = 0.5f;
            Fc.Step();
            float first = ship.Grid.GyroRateSystem.OverrideTargetVelocity.Length();
            Assert.That(first, Is.GreaterThan(0f), "A commanded yaw produces a requested target rate.");

            // Change the demanded attitude while turning: the requested rate changes.
            gyro.Yaw = 2f;
            Fc.Step();
            Assert.That(ship.Grid.GyroRateSystem.OverrideTargetVelocity.Length(), Is.Not.EqualTo(first).Within(1e-9),
                "Changing the demanded attitude changes the requested gyro rate.");
        }

        // — determinism: a mutation-heavy scenario repeats identically —

        [Test]
        public void MutationHeavyScenario_IsDeterministicAcrossTwoWorlds()
        {
            var (a, b) = DeterminismHelper.RunTwice(RunMutationScenario);

            Assert.That((b.FinalPosition - a.FinalPosition).Length(), Is.LessThan(1e-6),
                "Final position is identical across two identical worlds.");
            Assert.That((b.FinalVelocity - a.FinalVelocity).Length(), Is.LessThan(1e-6),
                "Final velocity is identical across two identical worlds.");
            Assert.That(b.FinalMass, Is.EqualTo(a.FinalMass).Within(1e-3));
            Assert.That(b.LoadRatio, Is.EqualTo(a.LoadRatio).Within(1e-9));
            Assert.That(b.WorkingEvents, Is.EqualTo(a.WorkingEvents));
        }

        private sealed class ScenarioC
        {
            public Vector3D FinalPosition;
            public Vector3D FinalVelocity;
            public float FinalMass;
            public float LoadRatio;
            public int WorkingEvents;
        }

        private static ScenarioC RunMutationScenario(FleetTestContext context)
        {
            var result = new ScenarioC();
            SimShip ship = context.Spawn(BatteryRig(8f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            load.PowerDemand = 8f;
            int workingEvents = 0;
            load.WorkingChanged += _ => workingEvents++;

            // A rich, deterministic schedule of mutations.
            context.AtTick(1, c => c.SetPowerAvailableRatio(ship, 0.5f));   // load now half-powered
            context.AtTick(4, c => c.SetBlockEnabled(load, false));          // disabled
            context.AtTick(6, c => c.SetBlockEnabled(load, true));           // re-enabled
            context.AtTick(8, c => c.ApplyLinearImpulse(ship, new Vector3(0, 0, 50f))); // external impulse
            context.AtTick(10, c => c.SetInventoryMass((SimTerminalBlock)load, 200f)); // inventory change

            context.Step(12);

            result.FinalPosition = ship.Grid.Body.PositionWorld;
            result.FinalVelocity = ship.Grid.Body.LinearVelocity;
            result.FinalMass = ship.Grid.Body.Mass;
            result.LoadRatio = load.DeliveredRatio;
            result.WorkingEvents = workingEvents;
            return result;
        }

        // — templates —

        private static ShipTemplate Rig()
        {
            return new ShipTemplate("Rig", 0.5f, new[]
            {
                new TemplateBlock("Hull", new Vector3I(0, 0, 0), HullMass, 0.5f),
                new TemplateBlock("Thruster", new Vector3I(1, 0, 0), BlockMass, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: new Vector3I(0, 0, -1), maxThrust: 50000f),
            });
        }

        private static ShipTemplate HullRig(float hullMass)
        {
            return new ShipTemplate("Hull", 0.5f, new[]
            {
                new TemplateBlock("Hull", new Vector3I(0, 0, 0), hullMass, 0.5f),
            });
        }

        private static ShipTemplate BatteryRig(float maxOutput)
        {
            return new ShipTemplate("Pwr", 0.5f, new[]
            {
                new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 0.5f,
                    kind: BlockKind.Battery, maxOutput: maxOutput, maxStoredPower: 100f),
                new TemplateBlock("Load", new Vector3I(2, 0, 0), 400f, 0.5f, kind: BlockKind.FixedThruster),
            });
        }

        private static ShipTemplate ReactorRig(float maxOutput)
        {
            return new ShipTemplate("Reactor", 0.5f, new[]
            {
                new TemplateBlock("Reactor", new Vector3I(0, 0, 0), 400f, 0.5f,
                    kind: BlockKind.Reactor, maxOutput: maxOutput),
                new TemplateBlock("Load", new Vector3I(2, 0, 0), 400f, 0.5f, kind: BlockKind.FixedThruster),
            });
        }

        private static ShipTemplate GyroRig()
        {
            return new ShipTemplate("Gyr", 0.5f, new[]
            {
                new TemplateBlock("Hull", new Vector3I(0, 0, 0), 1000f, 0.5f),
                new TemplateBlock("Gyro", new Vector3I(1, 0, 0), 300f, 0.5f,
                    kind: BlockKind.Gyro, gyroForce: 200000f),
            });
        }
    }
}
