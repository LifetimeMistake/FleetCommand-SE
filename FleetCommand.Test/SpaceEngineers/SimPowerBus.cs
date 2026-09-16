using System;
using System.Collections.Generic;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// A finite power source the bus can draw from: a battery or a
    /// reactor. <see cref="AvailableOutput"/> is the maximum MW the
    /// source can serve this step; <see cref="Draw"/> records the
    /// actually delivered output (returns it, so the bus can track how
    /// much delivered power remains). Batteries are reversible — their
    /// stored change is applied by the net charge path, not by
    /// <see cref="Draw"/>, so what the bus delivers and what the battery
    /// banks never diverge.
    /// </summary>
    internal interface SimPowerSource
    {
        /// <summary>Max power this source can serve on one fixed step (MW).</summary>
        float AvailableOutput(float dtSeconds);

        /// <summary>Records and returns the power actually supplied, bounded by
        /// <see cref="AvailableOutput"/> (MW).</summary>
        float Draw(float power, float dtSeconds);
    }

    /// <summary>
    /// The deterministic per-ship power model (, 
    /// <c>SimShip.Power</c> and ). Functional blocks register a power
    /// demand and a priority with their grid's bus; in tick phase 4 the
    /// bus computes the delivered ratio for every registered consumer and
    /// draws the delivered energy from the finite supply (batteries and
    /// reactors). Each functional block's <c>IsPowered</c> derives from
    /// its delivered ratio, so <c>IsWorking</c> (recomputed in the same
    /// phase) follows before actuation — phases 5–8 read the state this
    /// phase produced.
    /// </summary>
    /// <remarks>
    /// Allocation semantics (): the available supply is the sum of
    /// the grid's producer outputs, or effectively unlimited when the
    /// template provides no producer (the "ideal source"). Consumers are
    /// processed from the highest priority tier to the lowest; every block
    /// in a tier receives the same delivered ratio
    /// <c>min(1, available / tierDemand)</c>, so blocks of equal
    /// priority share a shortfall fairly and deterministically. The
    /// constant per-tier ratio, the fixed 1/60 s step, and the absence
    /// of wall-clock reads and noise make the bus exact and repeatable.
    /// The delivered ratio is the single source of truth that 
    /// (thrust) and  (gyro) will read to scale output and that
    /// updates <c>IsWorking</c> — no divergence between what phase 4
    /// computes and what actuation consumes.
    ///
    /// .7 charging: after the real consumers (grid loads) are
    /// served, the bus routes the remaining (surplus) supply to
    /// <strong>charging batteries</strong> in registration order, each up
    /// to its charge demand (<see cref="SimBatteryBlock.ComputeChargeDemand"/>
    /// — the game's <c>Sink_ComputeRequiredPower</c>). The charge power
    /// actually delivered becomes the battery's real
    /// <see cref="SimBatteryBlock.CurrentInput"/> and is netted against
    /// its output (<see cref="SimBatteryBlock.ApplyCharge(float, float)"/>,
    /// the game's <c>TransferPower</c> / <c>StorePower</c> /
    /// <c>ConsumePower</c>), so a battery in <c>Auto</c> charges from
    /// surplus after grid demand and discharges into a deficit, exactly
    /// as the source specifies. Charge is strictly lower priority than
    /// grid loads: batteries sink the surplus, they never starve a load.
    /// </remarks>
    public sealed class SimPowerBus
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimFunctionalBlock> _consumers = new List<SimFunctionalBlock>();
        private readonly List<SimPowerSource> _sources = new List<SimPowerSource>();
        private readonly List<SimBatteryBlock> _batteries = new List<SimBatteryBlock>();
        private readonly List<SimFunctionalBlock> _ordered = new List<SimFunctionalBlock>();

        /// <summary>The rigid grid (ship) this bus serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>
        /// True when the template provided finite production (a battery
        /// and/or reactor); false when the ship has the ideal
        /// (unlimited) source.
        /// </summary>
        public bool IsBatterySupply => _sources.Count > 0;

        /// <summary>
        /// The supply available for the most recent <see cref="Recompute"/>:
        /// the summed producer output (finite) or
        /// <see cref="float.PositiveInfinity"/> for the ideal source.
        /// </summary>
        public float TotalSupply { get; private set; }

        /// <summary>
        /// Scenario-side electrical availability in [0, 1] (
        /// "change available electrical ratio"): a multiplier applied to
        /// the grid's finite supply each phase 4 (a degraded bus, e.g. a
        /// shared shortfall). No effect on the ideal source (infinite).
        /// Default 1. A ratio change reproduces the  delivered
        /// ratio law <c>min(1, available / tierDemand)</c>.
        /// </summary>
        public float AvailableRatio { get; set; } = 1f;

        internal SimPowerBus(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers a functional block as a power consumer (its demand feeds phase 4 allocation).</summary>
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

        /// <summary>Registers a non-battery producer (e.g. a reactor) as part of the ship's finite supply.</summary>
        internal void RegisterProducer(SimPowerSource producer)
        {
            if (producer == null)
            {
                throw new ArgumentNullException(nameof(producer));
            }
            _sources.Add(producer);
        }

        /// <summary>Removes a functional block from the consumer demand list (scenario block removal).</summary>
        internal void UnregisterConsumer(SimFunctionalBlock block)
        {
            if (block != null)
            {
                _consumers.Remove(block);
            }
        }

        /// <summary>Removes a battery from the supply and the charge-sink list (scenario block removal).</summary>
        internal void UnregisterBattery(SimBatteryBlock battery)
        {
            if (battery == null)
            {
                return;
            }
            _sources.Remove(battery);
            _batteries.Remove(battery);
        }

        /// <summary>Removes a non-battery producer (e.g. a reactor) from the supply (scenario block removal).</summary>
        internal void UnregisterProducer(SimPowerSource producer)
        {
            if (producer != null)
            {
                _sources.Remove(producer);
            }
        }

        /// <summary>
        /// Recomputes the delivered power ratio for every registered
        /// consumer from the ideal or finite supply and this tick's
        /// demands, then draws the delivered energy from the producers
        /// and routes the surplus to charging batteries. Called by the
        /// world in tick phase 4, before actuation.
        /// </summary>
        /// <param name="dtSeconds">The fixed step interval in seconds (one 1/60 s tick).</param>
        public void Recompute(float dtSeconds)
        {
            if (dtSeconds <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(dtSeconds), "The power bus runs on the fixed positive physics step.");
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
            float delivered = 0f;
            int index = 0;
            while (index < _ordered.Count)
            {
                int priority = _ordered[index].PowerPriority;
                int tierStart = index;
                float tierDemand = 0f;
                while (index < _ordered.Count && _ordered[index].PowerPriority == priority)
                {
                    tierDemand += _ordered[index].PowerDemand;
                    index++;
                }
                int tierEnd = index;

                float ratio;
                if (tierDemand <= 0f)
                {
                    // No demand: nothing to supply, always satisfied.
                    ratio = 1f;
                }
                else if (float.IsPositiveInfinity(remaining))
                {
                    // Ideal source: demand is never limited.
                    ratio = 1f;
                }
                else
                {
                    // Deliver min(1, available / tierDemand) to the whole tier.
                    ratio = remaining <= 0f ? 0f : Math.Min(1f, remaining / tierDemand);
                    float tierDelivered = tierDemand * ratio;
                    remaining -= tierDelivered;
                    delivered += tierDelivered;
                }

                for (int i = tierStart; i < tierEnd; i++)
                {
                    _ordered[i].SetDeliveredRatio(ratio);
                }
            }

            // Depletion: the delivered power is drawn from the finite
            // supply in deterministic registration order, and every
            // producer's CurrentOutput stays fresh (0 when nothing left
            // to draw). Ideal supply (no producers) draws nothing.
            for (int i = 0; i < _sources.Count; i++)
            {
                SimPowerSource source = _sources[i];
                delivered = Math.Max(0f, delivered - source.Draw(delivered, dtSeconds));
            }

            // .7 — charging: route the surplus (what grid loads
            // did not consume) to charging batteries in registration
            // order, each up to its charge demand. The power actually
            // delivered is CurrentInput and is netted against output.
            float surplus = remaining;
            for (int i = 0; i < _batteries.Count; i++)
            {
                SimBatteryBlock battery = _batteries[i];
                float demand = battery.ComputeChargeDemand();
                float input;
                if (demand <= 0f || surplus <= 0f)
                {
                    input = 0f;
                }
                else
                {
                    input = Math.Min(demand, surplus);
                    surplus -= input;
                }
                battery.ApplyCharge(input, dtSeconds);
            }
        }

        /// <summary>
        /// The delivered power ratio for <paramref name="block"/>: the
        /// fraction of its demand the supply allocated in phase 4 (1 when
        /// fully satisfied or when the block demands nothing). This is the
        /// value  thrust and  gyro will read to scale
        /// output, and the value that drives the block's
        /// <c>IsPowered</c> / <c>IsWorking</c>.
        /// </summary>
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
            if (_sources.Count == 0)
            {
                // Ideal source: unlimited power, demand never limited.
                return float.PositiveInfinity;
            }
            float total = 0f;
            for (int i = 0; i < _sources.Count; i++)
            {
                total += _sources[i].AvailableOutput(dtSeconds);
            }
            // The scenario electrical availability ratio scales the finite
            // supply (); ideal supply stays infinite.
            float ratio = AvailableRatio < 0f ? 0f : AvailableRatio > 1f ? 1f : AvailableRatio;
            return total * ratio;
        }
    }
}
