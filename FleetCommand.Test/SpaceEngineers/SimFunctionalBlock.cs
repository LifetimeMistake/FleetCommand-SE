using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// A block with an enable toggle, implementing the real
    /// <see cref="IMyFunctionalBlock"/> (from Sandbox.ModAPI.Ingame; plan
    /// , ). <see cref="IsWorking"/> is derived from
    /// enablement, functionality, and power. <see cref="Enabled"/> and
    /// <see cref="IsFunctional"/> recompute working state immediately (the
    /// game's <c>MyFunctionalBlock.Enabled</c> setter calls
    /// <c>UpdateIsWorking()</c>), while power is a system quantity: the
    /// grid's <see cref="SimPowerBus"/> computes the block's
    /// <see cref="DeliveredRatio"/> in tick phase 4 from this block's
    /// <see cref="PowerDemand"/> and <see cref="PowerPriority"/> against
    /// the available supply, and <see cref="IsPowered"/> derives from it.
    /// <see cref="IsWorking"/> therefore follows on the same phase-4
    /// recompute, before actuation. State-change events fire on
    /// transitions for observation ().
    /// </summary>
    public class SimFunctionalBlock : SimTerminalBlock, IMyFunctionalBlock
    {
        private bool _enabled = true;
        private bool _functional = true;
        private float _deliveredRatio = 1f;
        private bool _working;

        public SimFunctionalBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            // Initial working state is the derived value from the default
            // enablement, functionality, and power; phase 4 keeps it fresh.
            // DeliveredRatio defaults to 1 (full power) so a block that
            // has not yet been through a phase-4 allocation starts
            // working. WorkingRequirementSatisfied is deliberately not
            // called here — it is virtual and a derived class's fields
            // (e.g. the battery's stored power) are not initialized
            // before this base constructor body runs; the first phase-4
            // recompute applies the real requirement.
            _working = _enabled && _functional && IsPowered;
        }

        /// <inheritdoc />
        public bool Enabled
        {
            get { return _enabled; }
            set
            {
                if (_enabled == value)
                {
                    return;
                }
                _enabled = value;
                EnabledChanged?.Invoke(this);
                RecomputeIsWorking();
            }
        }

        /// <inheritdoc />
        public override bool IsFunctional
        {
            get { return _functional; }
            set
            {
                if (_functional == value)
                {
                    return;
                }
                _functional = value;
                RecomputeIsWorking();
            }
        }

        /// <summary>
        /// The fraction of <see cref="PowerDemand"/> the ship's
        /// <see cref="SimPowerBus"/> delivers this tick, set in phase 4:
        /// 1 when fully satisfied (including blocks that demand nothing),
        /// 0 when supply reaches no power to this block, in between for a
        /// shared shortfall. This is the value  thrust and 
        /// gyro will read to scale their output, and what drives
        /// <see cref="IsPowered"/>.
        /// </summary>
        public float DeliveredRatio => _deliveredRatio;

        /// <summary>
        /// Power the block requires (MW) to operate at full capability.
        /// 0 (the default) means the block needs no power and is always
        /// considered fully powered. Scenario-set on the sim surface
        /// before the tick that should apply it. Virtual so a derived
        /// block (e.g. the gyro, whose required input scales with its
        /// power setting) can supply a computed demand that the bus
        /// consumes each phase 4.
        /// </summary>
        public virtual float PowerDemand { get; set; }

        /// <summary>
        /// Allocation priority: higher values are satisfied first when
        /// supply is short. Blocks with the same priority share a
        /// shortfall fairly (all receive the tier's delivered ratio).
        /// </summary>
        public int PowerPriority { get; set; }

        /// <summary>
        /// Scenario-side power availability, derived from
        /// <see cref="DeliveredRatio"/> computed by the ship's
        /// <see cref="SimPowerBus"/> in tick phase 4. A block whose
        /// delivered ratio is zero receives no power and therefore stops
        /// working on the same phase-4 recompute.
        /// </summary>
        public bool IsPowered => _deliveredRatio > 0f;

        /// <summary>True while the block is able to do its work (recomputed in phase 4).</summary>
        public override bool IsWorking => _working;

        /// <summary>Raised when <see cref="Enabled"/> changes value.</summary>
        public event Action<SimFunctionalBlock> EnabledChanged;

        /// <summary>Raised when <see cref="IsWorking"/> changes value.</summary>
        public event Action<SimFunctionalBlock> WorkingChanged;

        /// <inheritdoc />
        [Obsolete("Use the Enabled setter.")]
        public void RequestEnable(bool enable)
        {
            Enabled = enable;
        }

        /// <summary>
        /// Additional working requirement beyond enablement, functionality,
        /// and power. Defaults to none; <see cref="SimBatteryBlock"/>
        /// overrides it with the capacity gate (an empty battery is not
        /// working — the game's <c>CheckIsWorking</c>). Kept on
        /// <see cref="SimFunctionalBlock"/> so the derived working state
        /// has a single recompute path.
        /// </summary>
        protected virtual bool WorkingRequirementSatisfied => true;

        /// <summary>
        /// Applies the delivered ratio computed by the power bus in phase
        /// 4 (clamped to <c>[0, 1]</c>). This is the single source of
        /// truth that drives <see cref="IsPowered"/> — and therefore
        /// <see cref="IsWorking"/> — and that Stages 7–8 will consume.
        /// </summary>
        internal void SetDeliveredRatio(float ratio)
        {
            _deliveredRatio = ratio < 0f ? 0f : ratio > 1f ? 1f : ratio;
        }

        /// <summary>
        /// Refreshes <see cref="IsWorking"/> now (phase 4 and state
        /// setters). A functional block works when it is enabled,
        /// functional, powered, and its additional working requirement is
        /// satisfied ().
        /// </summary>
        protected internal override void RecomputeIsWorking()
        {
            base.RecomputeIsWorking();
            bool working = _enabled && _functional && IsPowered && WorkingRequirementSatisfied;
            if (_working != working)
            {
                _working = working;
                WorkingChanged?.Invoke(this);
            }
        }
    }
}
