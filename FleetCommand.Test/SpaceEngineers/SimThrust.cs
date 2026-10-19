using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A fixed grid thruster implementing <see cref="IMyThrust"/> (from
    /// Decompiled/Sandbox.Common/Sandbox/ModAPI/Ingame/IMyThrust.cs): the template maximum and
    /// effectiveness profile give <see cref="MaxThrust"/> and <see cref="MaxEffectiveThrust"/>;
    /// <see cref="ThrustOverride"/> and <see cref="ThrustOverridePercentage"/> are one shared
    /// override (the game's single <c>m_thrustOverride</c>). <see cref="CurrentThrust"/> is the
    /// delivered force from the previous physics interval, written by the grid's
    /// <see cref="ForceAggregator"/>. Power scaling reads the bus-computed
    /// <see cref="SimFunctionalBlock.DeliveredRatio"/>. <see cref="GridThrustDirection"/> is the
    /// grid-body forward (exhaust) axis and the grid is pushed opposite to it (<c>ThrustForce =
    /// -ThrustForwardVector</c>), so the aggregator groups thrusters by forward axis with no
    /// craft-type branches. The resultant acts at the grid COM, without mount-offset torque.</summary>
    public class SimThrust : SimFunctionalBlock, IMyThrust
    {
        private readonly Vector3I _direction;
        private readonly float _maxThrust;
        private readonly float _effectiveness;
        private readonly float _slowdownFactor;

        // Shared override storage, mirroring the game's single
        // m_thrustOverride: ThrustOverride (N) and ThrustOverridePercentage
        // (0..1) write the same fraction of MaxThrust. 0 means "not
        // overridden" (the thruster then answers the grid's group demand).
        private float _overrideFraction;
        private float? _scenarioDemand;
        internal bool UsesAutomaticPowerDemand => !_scenarioDemand.HasValue;
        internal float CommandPowerDemand
        {
            get
            {
                float throttle = IsOverridden ? _overrideFraction : Math.Max(0f,
                    Vector3.Dot(Grid.ForceAggregator.GroupDemand, -new Vector3(_direction)));
                return Template.ThrusterPowerDemand * throttle;
            }
        }
        internal float MinimumPowerDemand => IsOverridden ? 0f : Template.ThrusterIdlePowerDemand;

        /// <summary>Thrust electrical input follows commanded strength, bounded below by
        /// the definition's idle consumption (MyEntityThrustComponent.UpdatePowerAndThrustStrength).
        /// An explicit scenario demand remains available for synthetic power-bus fixtures.</summary>
        public override float PowerDemand
        {
            get
            {
                if (_scenarioDemand.HasValue) return _scenarioDemand.Value;
                return Math.Max(MinimumPowerDemand, CommandPowerDemand);
            }
            set { _scenarioDemand = value; }
        }

        private float _lastDelivered; // computed by the aggregator in phase 5
        private float _currentThrust; // committed in phase 8, after the physics interval

        public SimThrust(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            _direction = template.ThrustDirection ?? Vector3I.Zero;
            _maxThrust = template.MaxThrust;
            _effectiveness = template.Effectiveness;
            _slowdownFactor = template.SlowdownFactor;
            _scenarioDemand = null;
            _overrideFraction = 0f;
            _lastDelivered = 0f;
            _currentThrust = 0f;
            EnabledChanged += _ => Grid.Body.Activate();
        }

        // — IMyThrust —

        public override bool IsFunctional
        {
            get { return base.IsFunctional; }
            set
            {
                if (base.IsFunctional == value) return;
                base.IsFunctional = value;
                // ComponentStack_IsFunctionalChanged wakes physics even when
                // the thruster is disabled and its working state stays false.
                Grid.Body.Activate();
            }
        }

        /// <inheritdoc />
        public float ThrustOverride
        {
            get { return _overrideFraction * _maxThrust; }
            set
            {
                // Clamp to [0, MaxThrust], as the game's setter does
                // (m_thrustOverride (0..100) * ForceMagnitude * 0.01).
                if (value <= 0f)
                {
                    SetOverrideFraction(0f);
                }
                else if (_maxThrust > 1e-6f)
                {
                    SetOverrideFraction(Math.Min(1f, value / _maxThrust));
                }
                else
                {
                    SetOverrideFraction(0f);
                }
            }
        }

        /// <inheritdoc />
        public float ThrustOverridePercentage
        {
            get { return _overrideFraction; }
            set
            {
                SetOverrideFraction(value < 0f ? 0f : value > 1f ? 1f : value);
            }
        }

        /// <inheritdoc />
        public float MaxThrust => _maxThrust;

        private void SetOverrideFraction(float value)
        {
            if (_overrideFraction == value) return;
            _overrideFraction = value;
            // MyThrust_ThrustOverrideChanged -> MarkDirty -> RecomputeThrustParameters
            // wakes the grid once; unchanged per-frame writes do not.
            Grid.Body.Activate();
        }

        /// <inheritdoc />
        public float MaxEffectiveThrust => _maxThrust * _effectiveness;

        /// <summary>The thruster definition's <c>SlowdownFactor</c> (template data, default the
        /// definition value 1) — the multiplier on the delivered-thrust envelope in the grid's base
        /// (manual + dampened) control path. Read by <see cref="BaseControlSystem"/> via the grid's
        /// <see cref="ForceAggregator"/>.</summary>
        public float SlowdownFactor => _slowdownFactor;

        /// <summary>The delivered thrust from the last applied physics interval, written by the
        /// grid's <see cref="ForceAggregator"/> (after the force was applied). In
        /// Newtons.</summary>
        public float CurrentThrust => _currentThrust;

        /// <inheritdoc />
        public float CurrentThrustPercentage => _maxThrust > 1e-6f
            ? (float)Math.Round(_currentThrust / _maxThrust, 3) * 100f
            : 0f;

        /// <summary>The thruster's grid-body forward (exhaust) axis — the value the game returns
        /// (<c>MyThrust.cs:276</c>, controller-relative; the grid-body forward axis for a single
        /// rigid grid). The grid is pushed OPPOSITE to it (game: <c>ThrustForce =
        /// -ThrustForwardVector</c>, MyThrust.cs:188). Vector3I.Zero when the template supplied no
        /// direction.</summary>
        public Vector3I GridThrustDirection => _direction;

        // — aggregation surface (the grid ForceAggregator) —

        /// <summary>True while an override command is active (shared with
        /// <see cref="ThrustOverridePercentage"/>).</summary>
        internal bool IsOverridden => _overrideFraction > 0f;

        /// <summary>The template's current effectiveness multiplier (in [0, 1]).</summary>
        internal float Effectiveness => _effectiveness;

        /// <summary>Stashes the delivered force computed by the aggregator in phase 5.</summary>
        internal void SetLastDelivered(float delivered)
        {
            _lastDelivered = delivered;
        }

        /// <summary>Commits the phase-5 delivered force to <see cref="CurrentThrust"/> after the
        /// physics interval (tick phase 8).</summary>
        internal void CommitCurrentThrust()
        {
            _currentThrust = _lastDelivered;
        }
    }
}
