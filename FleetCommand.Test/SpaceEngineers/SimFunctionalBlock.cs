using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A block with an enable toggle, implementing <see cref="IMyFunctionalBlock"/>:
    /// <see cref="IsWorking"/> derives from enablement, functionality, and power.
    /// <see cref="Enabled"/> and <see cref="IsFunctional"/> recompute working state immediately
    /// (the game's <c>MyFunctionalBlock.Enabled</c> setter calls <c>UpdateIsWorking()</c>), while
    /// power is a system quantity: the grid's <see cref="SimPowerBus"/> computes the block's
    /// <see cref="DeliveredRatio"/> in tick phase 4 from this block's <see cref="PowerDemand"/> and
    /// <see cref="PowerPriority"/>, and against the available supply, and <see cref="IsPowered"/>
    /// derives from it (a block with no power requirement is always powered). State-change events
    /// fire on transitions for observation.</summary>
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

        /// <summary>The fraction of <see cref="PowerDemand"/> the ship's <see cref="SimPowerBus"/>
        /// delivers this tick (set in phase 4): 1 when fully satisfied (including blocks that
        /// demand nothing), 0 when the supply reaches no power to this block, in between for a
        /// shared shortfall; drives <see cref="IsPowered"/>.</summary>
        public float DeliveredRatio => _deliveredRatio;

        /// <summary>Power the block requires (MW) to operate at full capability; 0 (the default)
        /// means no power needed and always fully powered. Virtual so a derived block (e.g. the
        /// gyro, whose required input scales with its power setting) can supply a computed demand
        /// the bus consumes each phase 4.</summary>
        public virtual float PowerDemand { get; set; }

        /// <summary>Allocation priority: higher values are satisfied first when supply is short;
        /// blocks with the same priority share a shortfall fairly (all receive the tier's delivered
        /// ratio).</summary>
        public int PowerPriority { get; set; }

        /// <summary>Whether this block's power sink is adaptable — the game's
        /// <c>MyResourceDistributionGroup.IsAdaptible</c>. An adaptable sink shares a tier
        /// shortfall proportionally and stays powered on partial power; a non-adaptable sink is
        /// blacked out (no power) when its tier cannot be fully met and its demand is bypassed so
        /// the supply cascades to lower tiers (MyResourceDistributorComponent.cs:2238-2250). The
        /// sink's adaptability flags come from its group's definition in
        /// <c>ResourceDistributionGroups.sbc</c> (Thrust, BatteryBlock and Charging groups are
        /// adaptable; Gyro and most high-priority groups are not). Default true.</summary>
        public bool IsPowerAdaptable { get; set; } = true;

        /// <summary>Scenario-side power availability, matching the game's
        /// <c>MyResourceSinkComponent.SetInputFromDistributor</c> (MyResourceSinkComponent.cs:346-367):
        /// a block with no power requirement (<see cref="PowerDemand"/> 0) is always powered; an
        /// adaptable sink is powered on any nonzero delivery; a non-adaptable sink is powered only
        /// when fully satisfied (<see cref="DeliveredRatio"/> ≥ 1); and zero delivery to a
        /// demanding block leaves it unpowered.</summary>
        public bool IsPowered
        {
            get
            {
                float demand = PowerDemand;
                if (demand == 0f)
                {
                    return true; // no power requirement -> always powered (:353)
                }
                if (_deliveredRatio > 0f)
                {
                    return IsPowerAdaptable || _deliveredRatio >= 1f;
                }
                return false;
            }
        }

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

        /// <summary>Additional working requirement beyond enablement, functionality, and power
        /// (defaults to none); <see cref="SimBatteryBlock"/> overrides it with the capacity gate
        /// (an empty battery is not working — the game's <c>CheckIsWorking</c>). Kept here so the
        /// derived working state has a single recompute path.</summary>
        protected virtual bool WorkingRequirementSatisfied => true;

        /// <summary>Applies the delivered ratio computed by the power bus in phase 4 (clamped to
        /// [0, 1]) — the single source of truth that drives <see cref="IsPowered"/> and therefore
        /// <see cref="IsWorking"/>. A block with no power requirement is always fully supplied
        /// (ratio 1, its <c>SuppliedRatio</c>): the game's <c>SetInputFromDistributor</c> reports
        /// supplied ratio 1 and powered when required input is 0, even on a source-less grid
        /// (MyResourceSinkComponent.cs:353-361).</summary>
        internal void SetDeliveredRatio(float ratio)
        {
            if (PowerDemand == 0f)
            {
                _deliveredRatio = 1f;
                return;
            }
            _deliveredRatio = ratio < 0f ? 0f : ratio > 1f ? 1f : ratio;
        }

        /// <summary>Refreshes <see cref="IsWorking"/> now (phase 4 and state setters): a functional
        /// block works when it is enabled, functional, powered, and its additional working
        /// requirement is satisfied.</summary>
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
