using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;
using static FleetCommand.Test.Shared.TestHelpers;

// The obsolete battery aliases (OnlyRecharge/OnlyDischarge/SemiautoEnabled)
// are exercised deliberately; the obsolete-call usage warnings would
// otherwise be noise here.
#pragma warning disable CS0618

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (, /): the deterministic
    /// per-ship <see cref="SimPowerBus"/> and its
    /// <see cref="SimBatteryBlock"/> (finite or ideal) supply. The tests
    /// prove the delivered power ratio is computed exactly — priority
    /// ordered, tier-fair, repeatable — and is the same value that
    /// updates <c>IsWorking</c> and that  thrust /  gyro
    /// will read; that block working state transitions under power
    /// change; and that a finite battery's stored power depletes
    /// deterministically until it is exhausted (and then supplies
    /// nothing and is itself not working).
    /// </summary>
    [TestFixture]
    public class PowerBusConformanceTests : FleetTestFixture
    {
        // — wiring: SimShip.Power and the default ideal source —

        [Test]
        public void Spawn_ExposesPowerBus_WithIdealSourceByDefault()
        {
            SimShip ship = Fc.Spawn(TwoLoadTemplate("T"), SpawnState.At(Vector3D.Zero));

            Assert.That(ship.Power, Is.Not.Null);
            Assert.That(ship.Power, Is.SameAs(ship.Grid.PowerBus),
                "SimShip.Power must be the grid's (ship's) power bus.");
            Assert.That(ship.Power.IsBatterySupply, Is.False,
                "A template without a battery provides the ideal (unlimited) source.");

            // Ideal source: however much a block demands, it is satisfied.
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            load.PowerDemand = 10000f;
            Fc.Step();

            Assert.That(ship.Power.TotalSupply, Is.EqualTo(float.PositiveInfinity));
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6),
                "An ideal source must fully satisfy any demand by default.");
            Assert.That(load.IsPowered, Is.True);
            Assert.That(load.IsWorking, Is.True);
        }

        // — the delivered ratio: priority ordering and exposure —

        [Test]
        public void DeliveredRatio_HonorsPriority_AndIsExposedForThrustAndGyro()
        {
            // Battery supplies 8 MW. High priority demands 4, low priority
            // demands 8. High must be fully served; low gets the
            // remaining 4, i.e. ratio 0.5 — exact, per tier, and the
            // same value IsWorking and (later) thrust/gyro will read.
            SimShip ship = Fc.Spawn(FiniteTwoLoadTemplate(maxOutput: 8f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock high = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("High");
            SimFunctionalBlock low = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Low");
            SetLoad(high, demand: 4f, priority: 2);
            SetLoad(low, demand: 8f, priority: 1);

            Fc.Step();

            Assert.That(high.DeliveredRatio, Is.EqualTo(1f).Within(1e-6),
                "The higher-priority block must be fully satisfied first.");
            Assert.That(low.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "The lower-priority block must receive the exact remaining share (4/8).");

            // The bus exposes the same delivered ratios Stages 7–8 will
            // read to scale thrust/gyro output — one source of truth.
            Assert.That(ship.Power.GetDeliveredRatio(high), Is.EqualTo(1f).Within(1e-6));
            Assert.That(ship.Power.GetDeliveredRatio(low), Is.EqualTo(0.5f).Within(1e-6));

            // The values that feed IsWorking.
            Assert.That(high.IsPowered, Is.True);
            Assert.That(low.IsPowered, Is.True, "A block receiving any power is powered.");
            Assert.That(high.IsWorking, Is.True);

            // The battery delivered exactly the full 8 MW.
            SimBatteryBlock battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
            Assert.That(battery.CurrentOutput, Is.EqualTo(8f).Within(1e-4));
            Assert.That(battery.CurrentOutputRatio, Is.EqualTo(1f).Within(1e-6));
        }

        [Test]
        public void DeliveredRatio_EqualPrioritySharesShortfallFairly()
        {
            // Battery supplies 6 MW; two equal-priority blocks each demand
            // 6. The tier shares the shortfall: both get 6/12 = 0.5.
            SimShip ship = Fc.Spawn(FiniteTwoLoadTemplate(maxOutput: 6f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock a = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("High");
            SimFunctionalBlock b = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Low");
            SetLoad(a, demand: 6f, priority: 1);
            SetLoad(b, demand: 6f, priority: 1);

            Fc.Step();

            Assert.That(a.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "Equal-priority blocks must share a shortfall evenly.");
            Assert.That(b.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6));
        }

        // — block working transitions under power change —

        [Test]
        public void WorkingState_Transitions_UnderPriorityStarve_AndRecovers()
        {
            // Battery supplies 4 MW; a top-priority load takes all of it,
            // starving a bottom-priority load (ratio 0) — which must stop
            // working via phase 4, firing WorkingChanged. When the
            // top-priority load stops demanding, the starved one recovers.
            SimShip ship = Fc.Spawn(FiniteTwoLoadTemplate(maxOutput: 4f, maxStored: 10f), SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock high = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("High");
            SimFunctionalBlock low = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Low");
            SetLoad(high, demand: 4f, priority: 2);
            SetLoad(low, demand: 4f, priority: 1);

            int workingEvents = 0;
            low.WorkingChanged += _ => workingEvents++;

            Fc.Step();

            Assert.That(high.IsWorking, Is.True, "The top-priority load keeps full power.");
            Assert.That(low.DeliveredRatio, Is.EqualTo(0f), "The starved load receives no power.");
            Assert.That(low.IsPowered, Is.False);
            Assert.That(low.IsWorking, Is.False, "An unpowered functional block must not work.");
            Assert.That(workingEvents, Is.EqualTo(1), "Phase 4 must fire WorkingChanged on the power transition.");

            // The top-priority load stops demanding; the starved load
            // recovers to full power at the next phase 4.
            high.PowerDemand = 0f;
            Fc.Step();

            Assert.That(high.DeliveredRatio, Is.EqualTo(1f));
            Assert.That(low.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));
            Assert.That(low.IsWorking, Is.True, "Restoring supply must restore working state.");
        }

        // — finite battery depletion —

        [Test]
        public void FiniteBattery_Depletes_Exactly_ThenExhausts_AndStopsSupplying()
        {
            // A 4 MW battery with 0.002 MWh stored and a load that draws
            // the full output. Each second depletes 4/3600 MWh; the
            // battery lasts exactly 1.8 s, then is empty, stops
            // supplying, and the load loses power — all deterministic.
            SimShip ship = Fc.Spawn(SingleLoadTemplate(maxOutput: 4f, maxStored: 0.002f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            SetLoad(load, demand: 4f, priority: 1);

            Fc.RunFor(1.0);

            float expected = 0.002f - 4f * 1.0f / 3600f;
            Assert.That(battery.HasCapacityRemaining, Is.True);
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(expected).Within(1e-6),
                "Stored power must deplete by the drawn energy (MW·h), exact per fixed step.");
            Assert.That(load.IsWorking, Is.True);
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));

            // Run past exhaustion: the battery supplies nothing and is
            // itself not working (game: an empty battery's
            // CheckIsWorking is false); the load loses power.
            Fc.RunFor(2.0);

            Assert.That(battery.CurrentStoredPower, Is.EqualTo(0f));
            Assert.That(battery.HasCapacityRemaining, Is.False);
            Assert.That(battery.CurrentOutput, Is.EqualTo(0f));
            Assert.That(battery.IsWorking, Is.False, "An exhausted battery is not working.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(0f));
            Assert.That(load.IsPowered, Is.False);
            Assert.That(load.IsWorking, Is.False, "The load must stop working once the battery is empty.");
        }

        // — determinism: identical scenarios produce identical ratios —

        [Test]
        public void Bus_IsDeterministic_AcrossShips()
        {
            // Two identical ships in one world, run the same number of
            // ticks: the delivered ratios and stored power must be
            // identical (no wall-clock, no noise).
            ShipTemplate template = FiniteTwoLoadTemplate(maxOutput: 8f, maxStored: 0.01f);
            SimShip shipA = Fc.Spawn(template, SpawnState.At(new Vector3D(-50, 0, 0)));
            SimShip shipB = Fc.Spawn(template, SpawnState.At(new Vector3D(50, 0, 0)));
            SetLoad((SimFunctionalBlock)shipA.TerminalSystem.GetBlockWithName("High"), demand: 4f, priority: 2);
            SetLoad((SimFunctionalBlock)shipA.TerminalSystem.GetBlockWithName("Low"), demand: 8f, priority: 1);
            SetLoad((SimFunctionalBlock)shipB.TerminalSystem.GetBlockWithName("High"), demand: 4f, priority: 2);
            SetLoad((SimFunctionalBlock)shipB.TerminalSystem.GetBlockWithName("Low"), demand: 8f, priority: 1);

            Fc.Step(100);

            SimFunctionalBlock aHigh = (SimFunctionalBlock)shipA.TerminalSystem.GetBlockWithName("High");
            SimFunctionalBlock aLow = (SimFunctionalBlock)shipA.TerminalSystem.GetBlockWithName("Low");
            SimFunctionalBlock bHigh = (SimFunctionalBlock)shipB.TerminalSystem.GetBlockWithName("High");
            SimFunctionalBlock bLow = (SimFunctionalBlock)shipB.TerminalSystem.GetBlockWithName("Low");
            SimBatteryBlock aBattery = (SimBatteryBlock)shipA.TerminalSystem.GetBlockWithName("Battery");
            SimBatteryBlock bBattery = (SimBatteryBlock)shipB.TerminalSystem.GetBlockWithName("Battery");

            Assert.That(aHigh.DeliveredRatio, Is.EqualTo(bHigh.DeliveredRatio).Within(1e-9),
                "Delivered ratios must be deterministic across identical ships.");
            Assert.That(aLow.DeliveredRatio, Is.EqualTo(bLow.DeliveredRatio).Within(1e-9));
            Assert.That(aBattery.CurrentStoredPower, Is.EqualTo(bBattery.CurrentStoredPower).Within(1e-9),
                "Battery depletion must be deterministic across identical ships.");
        }

        // — battery interface surface —

        [Test]
        public void Battery_ImplementsThePowerProducerSurface()
        {
            SimShip ship = Fc.Spawn(SingleLoadTemplate(maxOutput: 4f, maxStored: 5f, maxInput: 1f), SpawnState.At(Vector3D.Zero));
            IMyBatteryBlock battery = (IMyBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
            SimBatteryBlock block = (SimBatteryBlock)battery;
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            SetLoad(load, demand: 2f, priority: 1);

            Assert.That(battery.MaxStoredPower, Is.EqualTo(5f));
            Assert.That(battery.MaxOutput, Is.EqualTo(4f));
            Assert.That(battery.MaxInput, Is.EqualTo(1f));
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(5f).Within(1e-6));
            Assert.That(battery.HasCapacityRemaining, Is.True);
            Assert.That(battery.CurrentInput, Is.EqualTo(0f), "No charging source is modeled.");
            Assert.That(battery.IsCharging, Is.False);
            Assert.That(battery.CurrentOutput, Is.EqualTo(0f), "Nothing has supplied yet.");
            Assert.That(battery.CurrentOutputRatio, Is.EqualTo(0f));

            // CurrentStoredPower setter (sim surface) clamps to
            // [0, MaxStoredPower], matching the game setter's contract.
            block.CurrentStoredPower = 999f;
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(5f));
            block.CurrentStoredPower = -5f;
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(0f));
            Assert.That(battery.HasCapacityRemaining, Is.False);

            // Charge mode and the obsolete aliases map onto each other as
            // the game's do.
            Assert.That(battery.ChargeMode, Is.EqualTo(ChargeMode.Auto));
            Assert.That(battery.OnlyRecharge, Is.False);
            battery.OnlyRecharge = true;
            Assert.That(battery.ChargeMode, Is.EqualTo(ChargeMode.Recharge));
            battery.ChargeMode = ChargeMode.Discharge;
            Assert.That(battery.OnlyDischarge, Is.True);
            Assert.That(battery.SemiautoEnabled, Is.False);
            battery.SemiautoEnabled = true;
            Assert.That(battery.ChargeMode, Is.EqualTo(ChargeMode.Auto));

            // CurrentOutput becomes the delivered power after a step.
            block.CurrentStoredPower = 5f;
            Fc.Step();
            Assert.That(battery.CurrentOutput, Is.EqualTo(2f).Within(1e-4),
                "The battery's CurrentOutput must be the power actually delivered this tick.");
            Assert.That(battery.CurrentOutputRatio, Is.EqualTo(0.5f).Within(1e-6));

            // Inherited members outside the implemented contract still
            // throw naming the member.
            AssertThrowsNamed(() => battery.DetailedInfo, "IMyTerminalBlock.DetailedInfo");
        }

        [Test]
        public void Battery_RechargeMode_ContributesNoSupply()
        {
            // A battery in Recharge mode supplies nothing, so a demanding
            // block goes unpowered and not working.
            SimShip ship = Fc.Spawn(SingleLoadTemplate(maxOutput: 4f, maxStored: 0.001f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
            SimFunctionalBlock load = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            SetLoad(load, demand: 4f, priority: 1);
            battery.ChargeMode = ChargeMode.Recharge;

            Fc.Step();

            Assert.That(ship.Power.TotalSupply, Is.EqualTo(0f));
            Assert.That(load.DeliveredRatio, Is.EqualTo(0f), "A battery in Recharge mode supplies nothing.");
            Assert.That(load.IsWorking, Is.False);
        }

        // — helpers —

        /// <summary>Scenario-set a functional load's demand and priority on the sim surface.</summary>
        private static void SetLoad(SimFunctionalBlock block, float demand, int priority)
        {
            block.PowerDemand = demand;
            block.PowerPriority = priority;
        }

        /// <summary>An empty battery template with two functional loads at distinct cells.</summary>
        private static ShipTemplate FiniteTwoLoadTemplate(float maxOutput, float maxStored = 100f)
        {
            return new ShipTemplate(
                "withBattery", 2.5f,
                new[]
                {
                    new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 2.5f,
                        kind: BlockKind.Battery, maxOutput: maxOutput, maxStoredPower: maxStored),
                    new TemplateBlock("High", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                    new TemplateBlock("Low", new Vector3I(4, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                });
        }

        /// <summary>An empty battery template with one functional load.</summary>
        private static ShipTemplate SingleLoadTemplate(float maxOutput, float maxStored, float maxInput = 0f)
        {
            return new ShipTemplate(
                "withBattery", 2.5f,
                new[]
                {
                    new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 2.5f,
                        kind: BlockKind.Battery, maxOutput: maxOutput, maxInput: maxInput,
                        maxStoredPower: maxStored, initialStoredPower: 1f),
                    new TemplateBlock("Load", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                });
        }

        private static ShipTemplate TwoLoadTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("Load", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                    new TemplateBlock("Payload", new Vector3I(3, 0, 0), 100f, 2.5f),
                });
        }
    }
}
