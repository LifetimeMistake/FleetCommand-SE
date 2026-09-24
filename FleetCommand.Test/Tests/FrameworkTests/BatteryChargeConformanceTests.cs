using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

// The obsolete battery aliases (OnlyRecharge/OnlyDischarge/SemiautoEnabled)
// are exercised via ChargeMode in these tests; the obsolete-call usage
// warnings would otherwise be noise here.
#pragma warning disable CS0618

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the battery charging path / ChargeMode, reproduced from
    /// MyBatteryBlock.cs.</summary>
    [TestFixture]
    public class BatteryChargeConformanceTests : FleetTestFixture
    {
        // MW·h per MW per fixed 1/60 s step: power · dt / 3600.
        private const float StepCharge = 1f / (3600f * 60f);

        // — Recharge: disable output and store surplus (MyBatteryBlock :520-560, :613-626,
        // :756-775) —

        [Test]
        public void RechargeStoresSurplus_RaisesStored_Analytic()
        {
            // Reactor 10 MW; battery in Recharge (MaxOut 4, MaxIn 6,
            // 5/10 stored); load 2. Recharge disables the battery's
            // output, so the reactor serves the load and the battery
            // absorbs the surplus (6 MW = min(charge demand, surplus))
            // into stored power, exactly input·step — the game's
            // StorePower (:756-775).
            SimShip ship = Fc.Spawn(ChargeTemplate("rc", reactorMaxOut: 10f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.5f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            SimReactorBlock reactor = Reactor(ship);
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 2f, priority: 1);
            battery.ChargeMode = ChargeMode.Recharge;

            Fc.RunFor(1.0); // 60 fixed steps

            // Recharge disables output: nothing leaves the battery (conformance 2).
            Assert.That(battery.CurrentOutput, Is.EqualTo(0f), "In Recharge the battery must not supply power.");
            Assert.That(reactor.CurrentOutput, Is.EqualTo(2f).Within(1e-4),
                "The reactor must carry the load while the battery recharges.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6), "The load stays fully powered.");

            // The battery absorbed 6 MW each step: stored rose by 60 · 6 · step.
            float expected = 5f + 60f * 6f * StepCharge;
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(expected).Within(1e-5),
                "Stored power must rise analytically by input·dt (StorePower), capped at MaxStored.");
            Assert.That(battery.CurrentInput, Is.EqualTo(6f).Within(1e-4),
                "CurrentInput must be the surplus the bus actually delivered to the charge sink.");
            Assert.That(battery.IsCharging, Is.True,
                "IsCharging follows the game: CurrentInput > 0 (and > CurrentOutput).");
            Assert.That(battery.CurrentInput, Is.LessThanOrEqualTo(battery.MaxInput),
                "Charge input is bounded by the battery's MaxInput.");
            Assert.That(battery.CurrentStoredPower, Is.LessThanOrEqualTo(battery.MaxStoredPower));
        }

        [Test]
        public void RechargeCharges_TowardMax_CappedAndStopsWhenFull()
        {
            // Battery starts 0.99 full (9.9/10), MaxIn 6; reactor 10;
            // no load. It fills toward MaxStoredPower — the game's
            // Sink_ComputeRequiredPower (:551) tapers the charge demand
            // as it approaches full, so it approaches but never exceeds
            // its cap (:757-773 StorePower clamps at MaxStored). A
            // full battery then wants no more input.
            SimShip ship = Fc.Spawn(ChargeTemplate("cc", reactorMaxOut: 10f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.99f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            battery.ChargeMode = ChargeMode.Recharge;

            Fc.RunFor(70.0);

            // Approaches MaxStored and is capped there (never exceeds it).
            Assert.That(battery.CurrentStoredPower, Is.GreaterThanOrEqualTo(battery.MaxStoredPower - 1e-3f),
                "The battery charges toward MaxStoredPower.");
            Assert.That(battery.CurrentStoredPower, Is.LessThanOrEqualTo(battery.MaxStoredPower),
                "Stored power never exceeds MaxStoredPower.");
            // The taper: very near full the charge input has dropped well
            // below MaxInput (:551's (Max − Current)·0.6·3600 term narrows).
            Assert.That(battery.CurrentInput, Is.LessThan(battery.MaxInput),
                "The charge demand tapers as the battery approaches full.");

            // Snap to full: a full battery takes no more input and stays capped.
            battery.CurrentStoredPower = battery.MaxStoredPower;
            Fc.Step();
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(battery.MaxStoredPower),
                "A full battery stays at MaxStoredPower.");
            Assert.That(battery.CurrentInput, Is.EqualTo(0f), "A full battery wants no input.");
            Assert.That(battery.IsCharging, Is.False, "A full battery is not charging.");
        }

        // — Auto: TransferPower — stores surplus, consumes deficit (MyBatteryBlock :743-754) —

        [Test]
        public void AutoChargesFromSurplus_NetInputMinusOutput()
        {
            // Reactor 30 MW; battery Auto (MaxOut 4, MaxIn 6, 5/10);
            // load 10. The battery (registered first) serves 4 to the
            // load and absorbs 6 of the surplus; TransferPower nets
            // input − output = 2 into stored. IsCharging is true
            // (CurrentInput > CurrentOutput > 0) — the brief's and the
            // game's surplus-branch case.
            SimShip ship = Fc.Spawn(ChargeTemplate("ac", reactorMaxOut: 30f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.5f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 10f, priority: 1);

            Fc.Step();

            Assert.That(battery.CurrentOutput, Is.EqualTo(4f).Within(1e-4),
                "The battery still sources toward the load in Auto.");
            Assert.That(battery.CurrentInput, Is.EqualTo(6f).Within(1e-4),
                "The battery absorbs the surplus up to MaxInput.");
            Assert.That(battery.IsCharging, Is.True,
                "TransferPower surplus branch: CurrentInput > CurrentOutput > 0 ⇒ IsCharging true.");
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(5f + 2f * StepCharge).Within(1e-5),
                "Auto banks the net input − output (StorePower of 2), the game's :750-753.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6), "The load remains fully served.");
        }

        [Test]
        public void AutoDischargesIntoDeficit_ConsumesNetOutputMinusInput()
        {
            // Reactor 2 MW; battery Auto (MaxOut 4, MaxIn 6, 5/10);
            // load 7 (> 6 available). The load is served 6/7; the
            // battery outputs 4 into the deficit, absorbs no surplus
            // (none left), and TransferPower consumes output − input = 4
            // from stored (:746-749) to cover the shortfall.
            SimShip ship = Fc.Spawn(ChargeTemplate("ad", reactorMaxOut: 2f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.5f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 7f, priority: 1);

            Fc.Step();

            Assert.That(load.DeliveredRatio, Is.EqualTo(6f / 7f).Within(1e-6),
                "The deficit is covered as far as supply allows.");
            Assert.That(battery.CurrentInput, Is.EqualTo(0f), "No surplus exists, so nothing charges.");
            Assert.That(battery.IsCharging, Is.False, "Deficit: output > input ⇒ not charging.");
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(5f - 4f * StepCharge).Within(1e-5),
                "Auto consumes the net deficit (output − input = 4) from stored (:748).");
        }

        // — Discharge: consume only, never charge (MyBatteryBlock :756-775) —

        [Test]
        public void Discharge_NeverCharges_OnlyConsumesOutput()
        {
            // Reactor 30 MW; battery Discharge (MaxOut 4, MaxIn 6,
            // 5/10); load 2. The battery (registered first) serves the
            // load and is never fed back: CurrentInput stays 0,
            // IsCharging stays false, stored only ever decreases.
            SimShip ship = Fc.Spawn(ChargeTemplate("dc", reactorMaxOut: 30f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.5f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 2f, priority: 1);
            battery.ChargeMode = ChargeMode.Discharge;

            Fc.Step(10); // exactly a full second

            Assert.That(battery.CurrentInput, Is.EqualTo(0f),
                "Discharge never absorbs input (charge demand is 0, :547-560).");
            Assert.That(battery.IsCharging, Is.False, "Discharge never reports charging.");
            Assert.That(battery.CurrentStoredPower, Is.EqualTo(5f - 10f * 2f * StepCharge).Within(1e-5),
                "Discharge only consumes its own delivered output — stored never increases.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));
        }

        // — depletion still modeled: a discharged battery fails over —

        [Test]
        public void DepletedBattery_FailsOverToReactor_NoPowerLoss()
        {
            // Battery (MaxOut 4, 0.002 MWh, full, MaxIn 0 so it cannot
            // charge) serves the load first, runs dry after 1.8 s, and
            // the reactor (4 MW) carries the load the whole time — the
            // load never loses power (depletion preserved, now with a
            // peer producer to fail over to).
            SimShip ship = Fc.Spawn(ChargeTemplate("fo", reactorMaxOut: 4f, batteryMaxOut: 4f,
                batteryMaxIn: 0f, batteryMaxStored: 0.002f, batteryInitial: 1f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);
            SimReactorBlock reactor = Reactor(ship);
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 4f, priority: 1);

            Fc.RunFor(3.0);

            Assert.That(battery.CurrentStoredPower, Is.EqualTo(0f), "The battery has run dry.");
            Assert.That(battery.HasCapacityRemaining, Is.False);
            Assert.That(battery.IsWorking, Is.False, "An exhausted battery is not working.");
            Assert.That(battery.CurrentOutput, Is.EqualTo(0f), "An exhausted battery supplies nothing.");
            Assert.That(reactor.CurrentOutput, Is.EqualTo(4f).Within(1e-4),
                "The reactor takes over the full load after the battery is spent.");
            Assert.That(load.DeliveredRatio, Is.EqualTo(1f).Within(1e-6),
                "The load never loses power — no regression of depletion.");
            Assert.That(load.IsWorking, Is.True);
        }

        // — output-vs-charge: the game's piecewise charge factor (MaxOutputLimitedByCapacity) —

        [Test]
        public void OutputFollows_GameChargeFactor_NearDepletion()
        {
            // The battery's max available output as a function of stored energy is the game's
            // MaxOutputLimitedByCapacity (MyResourceSourceComponent.cs:274-277):
            //   min(MaxOutput, RemainingCapacity · ProductionToCapacityMultiplier · 60)
            // where ProductionToCapacityMultiplier = 3600 for a battery (MyBatteryBlock.cs:460).
            // Full while stored energy is healthy; a linear ramp over the last ≈MaxOutput/216000
            // MWh near depletion. The factor is fixed (no ÷ dt) — independent of the update
            // step, exactly as the game computes it.
            SimShip ship = Fc.Spawn(ChargeTemplate("vd", reactorMaxOut: 100f, batteryMaxOut: 4f,
                batteryMaxIn: 0f, batteryMaxStored: 10f, batteryInitial: 0.5f), SpawnState.At(Vector3D.Zero));
            SimBatteryBlock battery = Battery(ship);

            // Healthy stored: full MaxOutput regardless of step.
            Assert.That(battery.AvailableOutput(1f / 60f), Is.EqualTo(4f).Within(1e-6),
                "Full output while stored energy is healthy (the capacity term is far above MaxOutput).");

            // Near depletion the cap is min(MaxOutput, stored · 3600 · 60): a linear ramp.
            // stored = 1e-5 MWh → cap = 1e-5 · 216000 = 2.16 MW (< MaxOutput 4).
            battery.CurrentStoredPower = 1e-5f;
            Assert.That(battery.AvailableOutput(1f / 60f), Is.EqualTo(1e-5f * 3600f * 60f).Within(1e-4),
                "Near depletion the available output is stored · 3600 · 60 (MyResourceSourceComponent.cs:276).");
            Assert.That(battery.AvailableOutput(1f / 60f), Is.LessThan(battery.MaxOutput),
                "The near-depletion ramp must be below MaxOutput.");

            // The game's factor is not step-rate dependent: the same cap at any valid step.
            Assert.That(battery.AvailableOutput(1f / 10f), Is.EqualTo(1e-5f * 3600f * 60f).Within(1e-4),
                "The capacity cap is a fixed piecewise factor (no ÷ dt) — independent of the step.");

            // Full linearly: the ramp reaches min(MaxOutput, ·) exactly at stored = MaxOutput/216000.
            float fullAt = battery.MaxOutput / (3600f * 60f);
            battery.CurrentStoredPower = fullAt;
            Assert.That(battery.AvailableOutput(1f / 60f), Is.EqualTo(battery.MaxOutput).Within(1e-4),
                "The ramp meets MaxOutput exactly at stored = MaxOutput/216000.");

            battery.CurrentStoredPower = 0f;
            Assert.That(battery.HasCapacityRemaining, Is.False);
            Assert.That(battery.AvailableOutput(1f / 60f), Is.EqualTo(0f),
                "An empty battery has no available output (HasCapacityRemaining gate).");
        }

        // — determinism —

        [Test]
        public void BatteryCharge_IsDeterministic_AcrossShips()
        {
            ShipTemplate template = ChargeTemplate("det", reactorMaxOut: 30f, batteryMaxOut: 4f,
                batteryMaxIn: 6f, batteryMaxStored: 10f, batteryInitial: 0.5f);
            SimShip shipA = Fc.Spawn(template, SpawnState.At(new Vector3D(-50, 0, 0)));
            SimShip shipB = Fc.Spawn(template, SpawnState.At(new Vector3D(50, 0, 0)));
            SetLoad(Load(shipA), demand: 10f, priority: 1);
            SetLoad(Load(shipB), demand: 10f, priority: 1);

            Fc.Step(60);

            SimBatteryBlock a = Battery(shipA);
            SimBatteryBlock b = Battery(shipB);
            Assert.That(a.CurrentStoredPower, Is.EqualTo(b.CurrentStoredPower).Within(1e-9),
                "Charging is deterministic across identical ships.");
            Assert.That(a.CurrentInput, Is.EqualTo(b.CurrentInput).Within(1e-9));
            Assert.That(a.CurrentOutput, Is.EqualTo(b.CurrentOutput).Within(1e-9));
            Assert.That(a.IsCharging, Is.EqualTo(b.IsCharging));
        }

        // — reactor supply surface —

        [Test]
        public void Reactor_ImplementsThePowerProducerSurface()
        {
            SimShip ship = Fc.Spawn(ChargeTemplate("rs", reactorMaxOut: 12f, batteryMaxOut: 2f,
                batteryMaxIn: 2f, batteryMaxStored: 5f, batteryInitial: 1f), SpawnState.At(Vector3D.Zero));
            IMyReactor reactor = (IMyReactor)ship.TerminalSystem.GetBlockWithName("Reactor");
            SimReactorBlock block = Reactor(ship);
            SimBatteryBlock battery = Battery(ship);
            battery.ChargeMode = ChargeMode.Recharge; // keep the recharger out of supply
            SimFunctionalBlock load = Load(ship);
            SetLoad(load, demand: 6f, priority: 1);

            Assert.That(reactor.MaxOutput, Is.EqualTo(12f));
            Assert.That(reactor.CurrentOutput, Is.EqualTo(0f), "Nothing has drawn yet.");
            Assert.That(reactor.CurrentOutputRatio, Is.EqualTo(0f));
            Assert.That(reactor.UseConveyorSystem, Is.True, "UseConveyorSystem defaults true (the game's).");
            block.UseConveyorSystem = false;
            Assert.That(reactor.UseConveyorSystem, Is.False, "UseConveyorSystem is a stored, settable value.");

            Fc.Step();
            Assert.That(reactor.CurrentOutput, Is.EqualTo(6f).Within(1e-4),
                "The reactor supplies what the load draws (up to its MaxOutput).");
            Assert.That(reactor.CurrentOutputRatio, Is.EqualTo(0.5f).Within(1e-6));
        }

        // — helpers —

        private static SimBatteryBlock Battery(SimShip ship)
        {
            return (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
        }

        private static SimReactorBlock Reactor(SimShip ship)
        {
            return (SimReactorBlock)ship.TerminalSystem.GetBlockWithName("Reactor");
        }

        private static SimFunctionalBlock Load(SimShip ship)
        {
            return (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
        }

        private static void SetLoad(SimFunctionalBlock block, float demand, int priority)
        {
            block.PowerDemand = demand;
            block.PowerPriority = priority;
        }

        /// <summary>A reactor + battery + one fixed load; batteryFirst controls supply registration
        /// order.</summary>
        private static ShipTemplate ChargeTemplate(
            string name, float reactorMaxOut, float batteryMaxOut,
            float batteryMaxIn, float batteryMaxStored, float batteryInitial,
            bool batteryFirst = true)
        {
            TemplateBlock reactor = new TemplateBlock("Reactor", new Vector3I(0, 0, 0), 400f, 2.5f,
                kind: BlockKind.Reactor, maxOutput: reactorMaxOut);
            TemplateBlock battery = new TemplateBlock("Battery", new Vector3I(2, 0, 0), 400f, 2.5f,
                kind: BlockKind.Battery, maxOutput: batteryMaxOut, maxInput: batteryMaxIn,
                maxStoredPower: batteryMaxStored, initialStoredPower: batteryInitial);
            TemplateBlock load = new TemplateBlock("Load", new Vector3I(4, 0, 0), 400f, 2.5f,
                kind: BlockKind.FixedThruster);
            TemplateBlock[] blocks = batteryFirst
                ? new[] { battery, reactor, load }
                : new[] { reactor, battery, load };
            return new ShipTemplate(name, 2.5f, blocks);
        }
    }
}
