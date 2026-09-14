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
    /// <c>UpdateIsWorking()</c>), while <see cref="IsPowered"/> is a
    /// scenario-side power flag that phase 4 of the tick applies on the
    /// next <see cref="Framework.SimWorld.Step"/> — power is a system
    /// quantity recomputed per tick (the power bus arrives in ).
    /// State-change events fire on transitions for observation ().
    /// </summary>
    public class SimFunctionalBlock : SimTerminalBlock, IMyFunctionalBlock
    {
        private bool _enabled = true;
        private bool _functional = true;
        private bool _powered = true;
        private bool _working;

        public SimFunctionalBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            // Initial working state is the derived value from the default
            // enablement, functionality, and power; phase 4 keeps it fresh.
            _working = _enabled && _functional && _powered;
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
        /// Scenario-side power availability. Defaults true (this stage has no
        /// power bus — : "power is assumed available"); 
        /// replaces it with the ship's <c>SimPowerBus</c>. Changes are
        /// applied by the tick's phase 4, so <see cref="IsWorking"/> follows
        /// on the next <see cref="Framework.SimWorld.Step"/>.
        /// </summary>
        public bool IsPowered
        {
            get { return _powered; }
            set { _powered = value; }
        }

        /// <inheritdoc />
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
        /// Refreshes <see cref="IsWorking"/> now (phase 4 and state setters).
        /// A functional block works when it is enabled, functional, and
        /// powered ().
        /// </summary>
        protected internal override void RecomputeIsWorking()
        {
            base.RecomputeIsWorking();
            bool working = _enabled && _functional && _powered;
            if (_working != working)
            {
                _working = working;
                WorkingChanged?.Invoke(this);
            }
        }
    }
}
