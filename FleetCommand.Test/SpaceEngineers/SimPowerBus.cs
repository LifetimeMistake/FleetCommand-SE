using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A finite power source the bus can draw from: a battery or a reactor.
    /// <see cref="AvailableOutput"/> is the maximum MW the source can serve this step;
    /// <see cref="Draw"/> records the actually delivered output (returns it, so the bus can track
    /// how much delivered power remains). Batteries are reversible — their stored change is applied
    /// by the net charge path, not by <see cref="Draw"/>, so what the bus delivers and what the
    /// battery banks never diverge. Sources are drawn by the game's source-group priority tier
    /// (see <see cref="SimPowerBus.SourceTier"/>), not registration order.</summary>
    internal interface SimPowerSource
    {
        /// <summary>Max power this source can serve on one fixed step (MW).</summary>
        float AvailableOutput(float dtSeconds);

        /// <summary>Records and returns the power actually supplied, bounded by
        /// <see cref="AvailableOutput"/> (MW).</summary>
        float Draw(float power, float dtSeconds);
    }

    /// <summary>The deterministic per-ship power model. Functional blocks register a power demand
    /// and a priority with their grid's bus; in tick phase 4 the bus computes the delivered ratio
    /// for every registered consumer and draws the delivered energy from the finite supply
    /// (batteries and reactors). Each functional block's <c>IsPowered</c> derives from its
    /// delivered ratio and power requirement, so <c>IsWorking</c> (recomputed in the same phase)
    /// follows before physics. Thrust aggregation precedes this distribution and
    /// reads its existing supplied ratio, matching source priorities 6 and 14.</summary>
    /// <remarks>Allocation semantics reproduce <c>MyResourceDistributorComponent</c>. A grid with
    /// no power-producing block supplies nothing — blocks with a real power demand are unpowered
    /// (only blocks with no power requirement stay powered). Where producers exist, the available
    /// supply is the sum of their outputs, consumers are served tier by tier from the highest
    /// priority to the lowest (a tier is fully satisfied when supply reaches it; an adaptable tier
    /// that cannot be fully met shares the shortfall proportionally; a non-adaptable tier that
    /// cannot be fully met is blacked out and its supply cascades to lower tiers), and the
    /// consumed power is drawn from producers by their source-group priority tier (battery before
    /// reactor). The fixed 1/60 s step and the absence of wall-clock reads and noise make the bus
    /// exact and repeatable. Charging: after the real consumers (grid loads) are served, the bus
    /// routes surplus to Recharge storage first, then Auto storage from ordinary producers, each up
    /// to its charge demand (<see cref="SimBatteryBlock.ComputeChargeDemand"/> — the game's
    /// <c>Sink_ComputeRequiredPower</c>). The charge power actually delivered becomes the battery's
    /// real <see cref="SimBatteryBlock.CurrentInput"/> and is netted against its output
    /// (<see cref="SimBatteryBlock.ApplyCharge(float, float)"/>, the game's <c>TransferPower</c> /
    /// <c>StorePower</c> / <c>ConsumePower</c>), so a battery in <c>Auto</c> charges from surplus
    /// after grid demand and discharges into a deficit. Charge is the lowest-priority sink — the
    /// game's BatteryBlock group — and never starves a load.</remarks>
    public sealed class SimPowerBus
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimFunctionalBlock> _consumers = new List<SimFunctionalBlock>();
        private readonly List<SimPowerSource> _sources = new List<SimPowerSource>();
        private readonly List<SimBatteryBlock> _batteries = new List<SimBatteryBlock>();
        private readonly List<SimFunctionalBlock> _ordered = new List<SimFunctionalBlock>();
        private bool _initialized;

        // Grid registration performs an initial distribution in the game
        // (MyCubeGridSystems.OnAddedToGroup), before scheduled thrust callbacks.
        // Establish sink ratios without consuming a second physics interval.
        internal void EnsureInitialized(float dtSeconds)
        {
            if (!_initialized) Recompute(dtSeconds, false);
        }

        /// <summary>The rigid grid (ship) this bus serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>True when the grid has at least one power-producing block (a battery and/or
        /// reactor); false for a source-less grid, which supplies no power.</summary>
        public bool IsBatterySupply => _sources.Count > 0;

        /// <summary>The supply available for the most recent <see cref="Recompute"/>: the summed
        /// producer output; 0 when the grid has no producer (a source-less grid delivers no power —
        /// the game's NoPower state, never an unlimited source).</summary>
        public float TotalSupply { get; private set; }

        /// <summary>Scenario-side electrical availability in [0, 1]: a multiplier applied to the
        /// grid's finite supply each phase 4 (a degraded bus, e.g. a shared shortfall); no effect
        /// on the ideal source (infinite). Default 1.</summary>
        public float AvailableRatio { get; set; } = 1f;

        internal SimPowerBus(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers a functional block as a power consumer (its demand feeds phase 4
        /// allocation).</summary>
        internal void RegisterConsumer(SimFunctionalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            _consumers.Add(block);
        }

        /// <summary>Registers a battery as part of the ship's finite, reversible supply.</summary>
        internal void RegisterSupply(SimBatteryBlock battery)
        {
            if (battery == null)
            {
                throw new ArgumentNullException(nameof(battery));
            }
            _sources.Add(battery);
            _batteries.Add(battery);
        }

        /// <summary>Registers a non-battery producer (e.g. a reactor) as part of the ship's finite
        /// supply.</summary>
        internal void RegisterProducer(SimPowerSource producer)
        {
            if (producer == null)
            {
                throw new ArgumentNullException(nameof(producer));
            }
            _sources.Add(producer);
        }

        /// <summary>Removes a functional block from the consumer demand list (scenario block
        /// removal).</summary>
        internal void UnregisterConsumer(SimFunctionalBlock block)
        {
            if (block != null)
            {
                _consumers.Remove(block);
            }
        }

        /// <summary>Removes a battery from the supply and the charge-sink list (scenario block
        /// removal).</summary>
        internal void UnregisterBattery(SimBatteryBlock battery)
        {
            if (battery == null)
            {
                return;
            }
            _sources.Remove(battery);
            _batteries.Remove(battery);
        }

        /// <summary>Removes a non-battery producer (e.g. a reactor) from the supply (scenario block
        /// removal).</summary>
        internal void UnregisterProducer(SimPowerSource producer)
        {
            if (producer != null)
            {
                _sources.Remove(producer);
            }
        }

        /// <summary>Recomputes the delivered power ratio for every registered consumer from this
        /// tick's finite supply and demands, then draws the delivered energy from the producers by
        /// source priority and routes the surplus to charging batteries. Called by the world in tick
        /// phase 4, before actuation.</summary>
        /// <param name="dtSeconds">The fixed step interval in seconds (one 1/60 s tick).</param>
        public void Recompute(float dtSeconds)
        {
            Recompute(dtSeconds, true);
        }

        private void Recompute(float dtSeconds, bool transferEnergy)
        {
            if (dtSeconds <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(dtSeconds), "The power bus runs on the fixed positive physics step.");
            }
            _initialized = true;

            if (_sources.Count == 0)
            {
                // A grid with no power-producing block supplies nothing. The game's
                // distributor computes MaxAvailableResource = 0 -> NoPower and gives every
                // sink zero input (MyResourceDistributorComponent.cs:1731-1734,
                // RecomputeResourceDistributionPartial :2247-2250). Blocks with a real power
                // demand are unpowered and stop working; blocks with no power requirement
                // (PowerDemand 0) stay powered (MyResourceSinkComponent.cs:353).
                TotalSupply = 0f;
                for (int i = 0; i < _consumers.Count; i++)
                {
                    _consumers[i].SetDeliveredRatio(0f);
                }
                return;
            }

            float supply = ComputeAvailableSupply(dtSeconds);
            TotalSupply = supply;

            // Consumers sorted by priority, highest first (cached buffer —
            // single-writer single-tick). Same-priority blocks share the
            // tier's ratio, so their relative order never affects the
            // result (the sort's stability is irrelevant).
            _ordered.Clear();
            _ordered.AddRange(_consumers);
            _ordered.Sort((a, b) => b.PowerPriority.CompareTo(a.PowerPriority));

            // remaining tracks the supply not yet committed to consumers:
            // after the loop it is the surplus available for charging.
            float remaining = supply;
            int index = 0;
            while (index < _ordered.Count)
            {
                int priority = _ordered[index].PowerPriority;
                int tierStart = index;
                float tierDemand = 0f;
                float thrustCommandDemand = 0f, thrustIdleDemand = 0f;
                bool tierAdaptable = true;
                while (index < _ordered.Count && _ordered[index].PowerPriority == priority)
                {
                    var consumer = _ordered[index];
                    if (consumer.Enabled && consumer.IsFunctional)
                    {
                        var thruster = consumer as SimThrust;
                        if (thruster != null && thruster.UsesAutomaticPowerDemand)
                        {
                            thrustCommandDemand += thruster.CommandPowerDemand;
                            thrustIdleDemand += thruster.MinimumPowerDemand;
                        }
                        else tierDemand += consumer.PowerDemand;
                    }
                    if (!_ordered[index].IsPowerAdaptable)
                    {
                        tierAdaptable = false; // tier adaptability is the AND across the tier
                    }
                    index++;
                }
                int tierEnd = index;
                // One electrical fuel group in the supported rigid-grid model. The game
                // floors summed command/override demand once, not once per idle thruster.
                tierDemand += Math.Max(thrustCommandDemand, thrustIdleDemand);

                // RecomputeResourceDistributionPartial: a tier is fully satisfied when the
                // supply reaches it (:2229-2236); an adaptable tier that cannot be fully met
                // shares the shortfall proportionally (:2238-2245); a non-adaptable tier
                // that cannot be fully met is blacked out and its supply is NOT consumed,
                // so it cascades to lower tiers (:2247-2250).
                float ratio;
                if (tierDemand <= 0f)
                {
                    ratio = 1f; // no demand: nothing to supply, always satisfied
                }
                else if (tierDemand <= remaining)
                {
                    ratio = 1f;
                    remaining -= tierDemand;
                }
                else if (tierAdaptable && remaining > 0f)
                {
                    ratio = remaining / tierDemand;
                    remaining = 0f;
                }
                else
                {
                    ratio = 0f;
                }

                for (int i = tierStart; i < tierEnd; i++)
                {
                    _ordered[i].SetDeliveredRatio(ratio);
                }
            }

            if (!transferEnergy) return;

            // Storage pairs are handled separately from ordinary sources in the game
            // (MyResourceDistributorComponent.RecomputeResourceDistributionPartial).
            // Recharge storage gets first claim on surplus. Auto storage may absorb only
            // surplus from ordinary producers, never unused storage output (self-charging).
            _plannedOutputs.Clear();
            _chargeInputs.Clear();
            float rechargeDemand = 0f;
            for (int i = 0; i < _batteries.Count; i++)
                if (_batteries[i].ChargeMode == ChargeMode.Recharge)
                    rechargeDemand += _batteries[i].ComputeChargeDemand();
            float rechargeInput = Math.Min(remaining, rechargeDemand);
            float rechargeRatio = rechargeDemand > 0f ? rechargeInput / rechargeDemand : 0f;
            for (int i = 0; i < _batteries.Count; i++)
            {
                var battery = _batteries[i];
                _chargeInputs[battery] = battery.ChargeMode == ChargeMode.Recharge
                    ? battery.ComputeChargeDemand() * rechargeRatio : 0f;
            }
            PlanDraw(supply - remaining + rechargeInput, dtSeconds, includeBatteries: true);

            float ordinarySurplus = 0f;
            for (int i = 0; i < _sources.Count; i++)
                if (!(_sources[i] is SimBatteryBlock))
                    ordinarySurplus += Math.Max(0f, SourceCapacity(_sources[i], dtSeconds) - PlannedOutput(_sources[i]));
            float autoDemand = 0f;
            for (int i = 0; i < _batteries.Count; i++)
                if (_batteries[i].ChargeMode == ChargeMode.Auto)
                    autoDemand += _batteries[i].ComputeChargeDemand();
            float autoInput = Math.Min(ordinarySurplus, autoDemand);
            float autoRatio = autoDemand > 0f ? autoInput / autoDemand : 0f;
            for (int i = 0; i < _batteries.Count; i++)
                if (_batteries[i].ChargeMode == ChargeMode.Auto)
                    _chargeInputs[_batteries[i]] = _batteries[i].ComputeChargeDemand() * autoRatio;
            PlanDraw(autoInput, dtSeconds, includeBatteries: false);

            // Draw once per producer so fuel accounting and telemetry include charge sinks.
            for (int i = 0; i < _sources.Count; i++)
                _sources[i].Draw(PlannedOutput(_sources[i]), dtSeconds);
            for (int i = 0; i < _batteries.Count; i++)
                _batteries[i].ApplyCharge(_chargeInputs[_batteries[i]], dtSeconds);
        }

        private readonly Dictionary<SimPowerSource, float> _plannedOutputs = new Dictionary<SimPowerSource, float>();
        private readonly Dictionary<SimBatteryBlock, float> _chargeInputs = new Dictionary<SimBatteryBlock, float>();

        private float PlannedOutput(SimPowerSource source)
        {
            float output;
            return _plannedOutputs.TryGetValue(source, out output) ? output : 0f;
        }

        private float SourceCapacity(SimPowerSource source, float dt)
        {
            return source.AvailableOutput(dt) * Math.Max(0f, Math.Min(1f, AvailableRatio));
        }

        private void PlanDraw(float needed, float dt, bool includeBatteries)
        {
            _sources.Sort((a, b) => SourceTier(a).CompareTo(SourceTier(b)));
            int start = 0;
            while (start < _sources.Count)
            {
                int end = start;
                int tier = SourceTier(_sources[start]);
                float capacity = 0f;
                while (end < _sources.Count && SourceTier(_sources[end]) == tier)
                {
                    var source = _sources[end++];
                    if (includeBatteries || !(source is SimBatteryBlock))
                        capacity += Math.Max(0f, SourceCapacity(source, dt) - PlannedOutput(source));
                }
                float ratio = capacity > 0f ? Math.Min(1f, needed / capacity) : 0f;
                for (int i = start; i < end; i++)
                {
                    var source = _sources[i];
                    if (!includeBatteries && source is SimBatteryBlock) continue;
                    float extra = Math.Max(0f, SourceCapacity(source, dt) - PlannedOutput(source)) * ratio;
                    _plannedOutputs[source] = PlannedOutput(source) + extra;
                }
                needed = Math.Max(0f, needed - capacity * ratio);
                start = end;
            }
        }
        /// <summary>The source-group priority tier of a producer — the game's resource source
        /// group, drawn in ascending <c>Priority</c> order (<c>ResourceDistributionGroups.sbc</c>:
        /// SolarPanels 1, Battery 2, Reactors 3; the distributor builds the priority arrays from
        /// <c>def.Priority</c>, MyResourceDistributorComponent.cs:702-743, and draws source tiers
        /// from index 0 up, :2371-2391). Lower tier = drawn first, so a battery discharges
        /// before a reactor.</summary>
        private static int SourceTier(SimPowerSource source)
        {
            if (source is SimBatteryBlock)
            {
                return 2; // "Battery"
            }
            if (source is SimReactorBlock)
            {
                return 3; // "Reactors"
            }
            return 1; // generic producer (solar first)
        }

        /// <summary>The delivered power ratio for <paramref name="block"/>: the fraction of its
        /// demand the supply allocated in phase 4 (1 when fully satisfied or when the block demands
        /// nothing); the value that drives the block's <c>IsPowered</c> /
        /// <c>IsWorking</c>.</summary>
        public float GetDeliveredRatio(SimFunctionalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            return block.DeliveredRatio;
        }

        private float ComputeAvailableSupply(float dtSeconds)
        {
            // Called only with at least one producer present (Recompute returns early on a
            // source-less grid). The scenario electrical availability ratio scales the supply.
            float total = 0f;
            for (int i = 0; i < _sources.Count; i++)
            {
                total += _sources[i].AvailableOutput(dtSeconds);
            }
            float ratio = AvailableRatio < 0f ? 0f : AvailableRatio > 1f ? 1f : AvailableRatio;
            return total * ratio;
        }
    }
}
