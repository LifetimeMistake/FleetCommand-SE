using System;
using System.Collections.Generic;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The deterministic per-ship power model (, 
    /// <c>SimShip.Power</c> and ). Functional blocks register a power
    /// demand and a priority with their grid's bus; in tick phase 4 the
    /// bus computes the delivered ratio for every registered consumer and
    /// draws the delivered energy from the finite battery supply. Each
    /// functional block's <c>IsPowered</c> derives from its delivered
    /// ratio, so <c>IsWorking</c> (recomputed in the same phase) follows
    /// before actuation — phases 5–8 read the state this phase produced.
    /// </summary>
    /// <remarks>
    /// Allocation semantics (): the available supply is the sum of
    /// the grid's battery outputs, or effectively unlimited when the
    /// template provides no battery (the "ideal source"). Consumers are
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
    /// </remarks>
    public sealed class SimPowerBus
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimFunctionalBlock> _consumers = new List<SimFunctionalBlock>();
        private readonly List<SimBatteryBlock> _supplies = new List<SimBatteryBlock>();
        private readonly List<SimFunctionalBlock> _ordered = new List<SimFunctionalBlock>();

        /// <summary>The rigid grid (ship) this bus serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>
        /// True when the template provided battery supply (finite); false
        /// when the ship has the ideal (unlimited) source.
        /// </summary>
        public bool IsBatterySupply => _supplies.Count > 0;

        /// <summary>
        /// The supply available for the most recent <see cref="Recompute"/>:
        /// the summed battery output (finite) or
        /// <see cref="float.PositiveInfinity"/> for the ideal source.
        /// </summary>
        public float TotalSupply { get; private set; }

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

        /// <summary>Registers a battery block as part of the ship's finite supply.</summary>
        internal void RegisterSupply(SimBatteryBlock battery)
        {
            if (battery == null)
            {
                throw new ArgumentNullException(nameof(battery));
            }
            _supplies.Add(battery);
        }

        /// <summary>
        /// Recomputes the delivered power ratio for every registered
        /// consumer from the ideal or finite battery supply and this tick's
        /// demands, then draws the delivered energy from the batteries.
        /// Called by the world in tick phase 4, before actuation.
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
            // battery's CurrentOutput stays fresh (0 when nothing left to
            // draw). Ideal supply (no batteries) draws nothing.
            for (int i = 0; i < _supplies.Count; i++)
            {
                SimBatteryBlock battery = _supplies[i];
                delivered = Math.Max(0f, delivered - battery.Draw(delivered, dtSeconds));
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
            if (_supplies.Count == 0)
            {
                // Ideal source: unlimited power, demand never limited.
                return float.PositiveInfinity;
            }
            float total = 0f;
            for (int i = 0; i < _supplies.Count; i++)
            {
                total += _supplies[i].AvailableOutput(dtSeconds);
            }
            return total;
        }
    }
}
