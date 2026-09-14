using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// A gyroscope, implementing the real <see cref="IMyGyro"/> member by
    /// member (ground truth:
    /// Decompiled/Sandbox.Common/Sandbox/ModAPI/Ingame/IMyGyro.cs — exactly
    /// <c>GyroPower</c>, <c>GyroOverride</c>, <c>Yaw</c>, <c>Pitch</c>,
    /// <c>Roll</c>). Inherited working state and the power stack come from
    /// <see cref="SimFunctionalBlock"/>. <see cref="GyroPower"/> (clamped
    /// to [0, 1]) scales <see cref="MaxGyroForce"/> (template force x power,
    /// the game's <c>ForceMagnitude * m_gyroPower</c>) and the
    /// <see cref="RequiredPowerInput"/> that feeds <c>PowerDemand</c>
    /// (so the phase-4 bus allocates it and the delivered ratio scales the
    /// delivered torque — single source of truth, ).
    /// <para>
    /// <see cref="Yaw"/>/<see cref="Pitch"/>/<see cref="Roll"/> hold a
    /// single rotated override velocity with the source-verified
    /// <b>all-three-negated</b> ingame mapping (MyGyro.cs:141-190): each
    /// property setter stores the negation of its value on the
    /// corresponding override-velocity axis, so
    /// <c>m_gyroOverrideVelocity = (-Pitch, -Yaw, -Roll)</c> in the
    /// block's local frame, clamped to ± the grid's configured
    /// <see cref="SimRigidBody.MaxAngularVelocity"/> (the game's
    /// <c>MaxAngularRadiansPerSecond</c>). The grid-space target rate is
    /// <see cref="GyroOverrideVelocityGrid"/> — the local override
    /// velocity transformed by the block's orientation into grid space
    /// (MyGyro.cs:139), which is what the grid's <see cref="GyroRateSystem"/>
    /// force-weights when aggregating. Setters only take effect while
    /// <see cref="GyroOverride"/> is true (the game's setter gate).
    /// </para>
    /// </summary>
    public class SimGyro : SimFunctionalBlock, IMyGyro
    {
        private readonly float _maxForce;
        private readonly Matrix _orientationMatrix;
        private readonly float _basePowerDemand;

        // The game's m_gyroOverrideVelocity (local-space), MyGyro.cs:97.
        private Vector3 _overrideVelocity;
        private bool _gyroOverride;
        private float _gyroPower = 1f;

        // ModAPI-only multipliers (ingame IMyGyro has no way to set them);
        // both stay at 1 like the game's defaults.
        private float _gyroMultiplier = 1f;
        private float _powerConsumptionMultiplier = 1f;

        // Telemetry written by the grid's GyroRateSystem in tick phase 8.
        private float _lastDeliveredTorque;
        private bool _saturated;

        internal SimGyro(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            _maxForce = template.GyroForce;
            _basePowerDemand = template.GyroPowerDemand;
            _overrideVelocity = Vector3.Zero;
            // The block's local orientation (block-local -> grid rotation),
            // matching template.LocalTransform's rotation part and the
            // game's base.Orientation (MyGyro.cs:139).
            _orientationMatrix = Matrix.CreateFromQuaternion(template.Orientation);
        }

        // — IMyGyro —

        /// <inheritdoc />
        public float GyroPower
        {
            get { return _gyroPower; }
            set
            {
                value = value < 0f ? 0f : value > 1f ? 1f : value;
                _gyroPower = value;
            }
        }

        /// <inheritdoc />
        public bool GyroOverride
        {
            get { return _gyroOverride; }
            set
            {
                _gyroOverride = value;
                // The game's SetGyroOverride marks the gyro system dirty
                // (MyGyro.cs), which re-arms the override ramp.
                if (Grid != null)
                {
                    Grid.GyroRateSystem.MarkDirty();
                }
            }
        }

        /// <inheritdoc />
        public float Yaw
        {
            get { return -_overrideVelocity.Y; }
            set
            {
                if (_gyroOverride)
                {
                    _overrideVelocity.Y = -ClampToMaxAngular(value);
                    MarkOverrideChanged();
                }
            }
        }

        /// <inheritdoc />
        public float Pitch
        {
            get { return -_overrideVelocity.X; }
            set
            {
                if (_gyroOverride)
                {
                    _overrideVelocity.X = -ClampToMaxAngular(value);
                    MarkOverrideChanged();
                }
            }
        }

        /// <inheritdoc />
        public float Roll
        {
            get { return -_overrideVelocity.Z; }
            set
            {
                if (_gyroOverride)
                {
                    _overrideVelocity.Z = -ClampToMaxAngular(value);
                    MarkOverrideChanged();
                }
            }
        }

        // — game-derived properties —

        /// <summary>
        /// Maximum gyroscopic force at the current power setting
        /// (<c>ForceMagnitude * m_gyroPower * m_gyroMultiplier</c>,
        /// MyGyro.cs:107). In N·m.
        /// </summary>
        public float MaxGyroForce => _maxForce * _gyroPower * _gyroMultiplier;

        /// <summary>
        /// Power the gyro needs (MW) at the current power setting
        /// (<c>RequiredPowerInput = RequiredPowerInput · m_gyroPower ·
        /// _m_powerConsumptionMultiplier</c>, MyGyro.cs:110). This is
        /// what the bus allocates (via <see cref="PowerDemand"/>).
        /// </summary>
        public float RequiredPowerInput => _basePowerDemand * _gyroPower * _powerConsumptionMultiplier;

        /// <summary>Override the power demand: the gyro's required input scales with its power setting.</summary>
        public override float PowerDemand
        {
            get { return RequiredPowerInput; }
            set { /* gyro demand is derived from GyroPower, not scenario-set */ }
        }

        /// <summary>
        /// The grid-space override target rate: the local override
        /// velocity transformed by the block's orientation
        /// (<c>Vector3.TransformNormal(m_gyroOverrideVelocity,
        /// base.Orientation)</c>, MyGyro.cs:139). This is what the
        /// grid's <see cref="GyroRateSystem"/> force-weights when
        /// aggregating its grid-frame target.
        /// </summary>
        public Vector3 GyroOverrideVelocityGrid => Vector3.TransformNormal(_overrideVelocity, _orientationMatrix);

        /// <summary>
        /// The override target rate in the grid frame (grid-space version
        /// of <see cref="GyroOverrideVelocityGrid"/>). For observation
        /// ( "requested gyro rates").
        /// </summary>
        public Vector3 RequestedGridRate => GyroOverrideVelocityGrid;

        // — aggregation surface (the grid GyroRateSystem) —

        /// <summary>True while this gyro is contributing to the (non-override) authority pool.</summary>
        internal bool IsOverridden => _gyroOverride;

        /// <summary>True when the block is used by the gyro system (enabled + functional, the game's IsUsed).</summary>
        internal bool IsUsed => Enabled && IsFunctional;

        /// <summary>The delivered torque written by the gyro system in phase 8 (last applied interval), in N·m.</summary>
        internal float LastDeliveredTorque => _lastDeliveredTorque;

        /// <summary>True when the last applied torque was clamped by the gyro-force sphere.</summary>
        internal bool IsSaturated => _saturated;

        /// <summary>Writes this gyro's delivered torque and saturation for the phase-8 telemetry ().</summary>
        internal void CommitTelemetry(float deliveredTorque, bool saturated)
        {
            _lastDeliveredTorque = deliveredTorque;
            _saturated = saturated;
        }

        private float ClampToMaxAngular(float value)
        {
            float max = MaxAngularGrid;
            if (value < -max)
            {
                return -max;
            }
            if (value > max)
            {
                return max;
            }
            return value;
        }

        /// <summary>
        /// The game's <c>SetGyroTorque</c> marks the gyro system dirty
        /// (MyGridGyroSystem.MarkDirty), which re-arms the override
        /// acceleration ramp. Mirror that so an override-velocity change
        /// mid-rotation re-arms the ramp like the game.
        /// </summary>
        private void MarkOverrideChanged()
        {
            if (Grid != null)
            {
                Grid.GyroRateSystem.MarkDirty();
            }
        }

        private float MaxAngularGrid
        {
            get
            {
                SimRigidBody body = Grid == null ? null : Grid.Body;
                return body == null ? 0f : body.MaxAngularVelocity;
            }
        }
    }
}
