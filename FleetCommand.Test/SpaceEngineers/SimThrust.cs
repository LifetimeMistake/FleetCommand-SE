using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// A fixed grid thruster, implementing the real
    /// <see cref="Sandbox.ModAPI.Ingame.IMyThrust"/> (from
    /// Decompiled/Sandbox.Common/Sandbox/ModAPI/Ingame/IMyThrust.cs; ,
    /// /). The thruster's template maximum and effectiveness
    /// profile give <see cref="MaxThrust"/> and <see cref="MaxEffectiveThrust"/>;
    /// <see cref="ThrustOverride"/> and <see cref="ThrustOverridePercentage"/>
    /// are one shared override (like the game's single
    /// <c>m_thrustOverride</c>), the per-thruster command the PB code
    /// writes. <see cref="CurrentThrust"/> is the delivered force from the
    /// previous physics interval, written by the grid's
    /// <see cref="ForceAggregator"/> in tick phase 8 ( step 6) —
    /// readable in the next tick's callbacks. Inherited working state from
    /// <see cref="SimFunctionalBlock"/>; power scaling reads the
    /// bus-computed <see cref="SimFunctionalBlock.DeliveredRatio"/>
    /// (single source of truth, ). <see cref="GridThrustDirection"/>
    /// is the thruster's grid-body forward (exhaust) axis — the value the
    /// game returns — and the grid is pushed opposite to it, matching the
    /// game's <c>ThrustForce = -ThrustForwardVector</c>. The aggregator
    /// thus groups thrusters by forward axis and applies each force along
    /// its negation, with no craft-type branches (asymmetric and missing
    /// axes are supported by template data). Fixed grid thrusters generate
    /// no torque from block position (the net force is applied at the grid
    /// COM, ).
    /// </summary>
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

        private float _lastDelivered; // computed by the aggregator in phase 5
        private float _currentThrust; // committed in phase 8, after the physics interval

        public SimThrust(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            _direction = template.ThrustDirection ?? Vector3I.Zero;
            _maxThrust = template.MaxThrust;
            _effectiveness = template.Effectiveness;
            _slowdownFactor = template.SlowdownFactor;
            PowerDemand = template.ThrusterPowerDemand;
            _overrideFraction = 0f;
            _lastDelivered = 0f;
            _currentThrust = 0f;
        }

        // — IMyThrust —

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
                    _overrideFraction = 0f;
                }
                else if (_maxThrust > 1e-6f)
                {
                    _overrideFraction = Math.Min(1f, value / _maxThrust);
                }
                else
                {
                    _overrideFraction = 0f;
                }
            }
        }

        /// <inheritdoc />
        public float ThrustOverridePercentage
        {
            get { return _overrideFraction; }
            set
            {
                _overrideFraction = value < 0f ? 0f : value > 1f ? 1f : value;
            }
        }

        /// <inheritdoc />
        public float MaxThrust => _maxThrust;

        /// <inheritdoc />
        public float MaxEffectiveThrust => _maxThrust * _effectiveness;

        /// <summary>
        /// The thruster definition's <c>SlowdownFactor</c> (template data,
        /// default the definition value 1) — the multiplier on the
        /// delivered-thrust envelope in the grid's base (manual + dampened)
        /// control path (.5). Read by
        /// <see cref="SpaceEngineers.BaseControlSystem"/> via the grid's
        /// <see cref="ForceAggregator"/>.
        /// </summary>
        public float SlowdownFactor => _slowdownFactor;

        /// <summary>
        /// The delivered thrust from the last applied physics interval,
        /// updated by the grid's <see cref="ForceAggregator"/> in tick
        /// phase 8 (after the force was applied in phase 6). In Newtons.
        /// </summary>
        public float CurrentThrust => _currentThrust;

        /// <inheritdoc />
        public float CurrentThrustPercentage => _maxThrust > 1e-6f
            ? (float)Math.Round(_currentThrust / _maxThrust, 3) * 100f
            : 0f;

        /// <summary>
        /// The thruster's grid-body forward direction (the template's
        /// thruster direction): for a single rigid grid, the body axis the
        /// thruster's exhaust points along. This is the value the game
        /// returns — <c>MyThrust.cs:276</c> (<see cref="IMyThrust.GridThrustDirection"/>)
        /// returns the block's forward vector, controller-relative; for a
        /// single rigid grid that is the grid-body forward axis. The grid
        /// is pushed OPPOSITE to this direction (game:
        /// <c>ThrustForce = -ThrustForwardVector x ForceMagnitude</c>,
        /// MyThrust.cs:188). Vector3I.Zero when the template supplied no
        /// direction.
        /// </summary>
        public Vector3I GridThrustDirection => _direction;

        // — aggregation surface (the grid ForceAggregator) —

        /// <summary>True while an override command is active (shared with <see cref="ThrustOverridePercentage"/>).</summary>
        internal bool IsOverridden => _overrideFraction > 0f;

        /// <summary>The template's current effectiveness multiplier (in [0, 1]).</summary>
        internal float Effectiveness => _effectiveness;

        /// <summary>Stashes the delivered force computed by the aggregator in phase 5.</summary>
        internal void SetLastDelivered(float delivered)
        {
            _lastDelivered = delivered;
        }

        /// <summary>
        /// Commits the phase-5 delivered force to <see cref="CurrentThrust"/>
        /// after the physics interval (tick phase 8,  step 6).
        /// </summary>
        internal void CommitCurrentThrust()
        {
            _currentThrust = _lastDelivered;
        }
    }
}
