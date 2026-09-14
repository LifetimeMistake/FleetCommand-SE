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
    /// is the body-space axis the thruster pushes the grid along — for a
    /// single rigid grid the grid's forward, per the task brief — so the
    /// aggregator groups thrusters by push axis with no craft-type
    /// branches (asymmetric and missing axes are supported by template
    /// data). Fixed grid thrusters generate no torque from block position
    /// (the net force is applied at the grid COM, ).
    /// </summary>
    public class SimThrust : SimFunctionalBlock, IMyThrust
    {
        private readonly Vector3I _direction;
        private readonly float _maxThrust;
        private readonly float _effectiveness;

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
        /// The body-space direction this thruster pushes the grid along
        /// (the template's thruster direction). For a single rigid grid
        /// this is the grid's forward axis — the body axis the thruster
        /// pushes along. Vector3I.Zero when the template supplied no
        /// direction (a thruster block without thruster params).
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
