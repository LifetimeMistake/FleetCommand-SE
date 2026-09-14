using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The grid gyro-rate system (,  steps 1–9) — the
    /// boundary where per-gyro override channels become one body-space
    /// angular impulse, reproducing the game's override loop
    /// (<c>MyGridGyroSystem.UpdateOverriddenGyros</c>). Owned by
    /// <see cref="SimCubeGrid"/>; the world drives it in three tick phases:
    /// <list type="number">
    /// <item><b>Phase 5</b> <see cref="Aggregate"/> — for each used gyro
    /// (enabled + functional, the game's <c>IsUsed</c>), divert
    /// override gyros into the force-weighted grid-frame target rate (or,
    /// when <see cref="AutopilotEnabled"/>, into the non-override
    /// authority pool) and sum the authority pool, exactly as
    /// <c>RecomputeGyroParameters</c> (MyGridGyroSystem.cs:447-476).</item>
    /// <item><b>Phase 6</b> <see cref="ApplyTorque"/> — read the body's
    /// world angular velocity into the grid frame, apply the SE override
    /// acceleration-ramp policy, convert the rate error into a desired
    /// torque via the inverse inertia diagonal, clamp to the aggregate
    /// gyro-force sphere and supplied power, and apply ONE resultant
    /// body-space angular impulse (steps 3–8). The grid angular-rate cap
    /// (step 9) is the body's configured
    /// <see cref="SimRigidBody.MaxAngularVelocity"/>, already applied by
    /// Havok — not re-implemented here.</item>
    /// <item><b>Phase 8</b> <see cref="UpdateGyroTelemetry"/> — write each
    /// gyro's delivered torque and saturation ().</item>
    /// </list>
    /// The override is the autopilot's rate-command path: Yaw/Pitch/Roll
    /// command a grid target rate and the loop closes the torque against it
    /// at the gyro-force ceiling — no position servo. <see cref="ControlTorque"/>
    /// is the scenario's non-override stabilization input (zero by
    /// default), added as <c>ControlTorque * m_maxGyroForce</c>. Power
    /// scaling reads the gyros' phase-4 <see cref="SimFunctionalBlock.DeliveredRatio"/>
    /// (single source of truth), force-weighted into one gyro-system ratio —
    /// the exact per-gyro phase-4 value when the gyros share a tier.
    /// </summary>
    public sealed class GyroRateSystem
    {
        /// <summary>Tiny torque threshold below which nothing is applied (game: TorqueSqLenTh = 0.0001).</summary>
        private const float TorqueSqLenThreshold = 0.0001f;

        /// <summary>Large-rate override ramp frame count (MyGridGyroSystem.cs:313).</summary>
        private const int LargeRateRampFrames = 120;

        /// <summary>Ramp frame scale below which the (short) ramp applies (MyGridGyroSystem.cs:311).</summary>
        private const float RampFrameSqThreshold = 2.467401f;

        /// <summary>Ramp frame count per unit squared rate error (MyGridGyroSystem.cs:317).</summary>
        private const float RampFrameScale = 48.228886f;

        private readonly SimCubeGrid _grid;
        private readonly List<SimGyro> _gyros = new List<SimGyro>();

        private float _maxGyroForce;      // non-override authority pool (sum of MaxGyroForce)
        private float _maxOverrideForce;  // override pool (sum of MaxGyroForce)
        private Vector3 _overrideTargetVelocity; // force-weighted grid-frame target rate
        private int _overrideAccelerationRampFrames = -1; // -1 = unset (game: int?)
        private Vector3 _controlTorque;

        private Vector3 _lastOverrideTarget;
        private Vector3 _lastGridTorque;
        private bool _lastSaturated;

        /// <summary>The rigid grid (ship) this rate system serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>
        /// When true, override gyros are diverted into the non-override
        /// authority pool instead of the override pool (the game's
        /// <c>AutopilotEnabled</c>). Autopilot tests drive gyros through
        /// the override path (flag false); this models the flag's only
        /// effect here.
        /// </summary>
        public bool AutopilotEnabled;

        /// <summary>
        /// The scenario's non-override stabilization input (the game's
        /// <c>ControlTorque</c>), added as
        /// <c>ControlTorque * m_maxGyroForce</c>. Zero by default.
        /// </summary>
        public Vector3 ControlTorque
        {
            get { return _controlTorque; }
            set { _controlTorque = value; }
        }

        /// <summary>The force-weighted grid-frame target rate from the most recent <see cref="Aggregate"/> ().</summary>
        public Vector3 OverrideTargetVelocity => _lastOverrideTarget;

        /// <summary>The grid-frame torque applied in the most recent <see cref="ApplyTorque"/> ().</summary>
        public Vector3 LastTorque => _lastGridTorque;

        /// <summary>True when the last applied torque was clamped by the aggregate gyro-force sphere.</summary>
        public bool LastSaturated => _lastSaturated;

        internal GyroRateSystem(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers a gyro block with this rate system (spawn).</summary>
        internal void RegisterGyro(SimGyro gyro)
        {
            if (gyro == null)
            {
                throw new ArgumentNullException(nameof(gyro));
            }
            _gyros.Add(gyro);
        }

        /// <summary>
        /// Tick phase 5: aggregate the force-weighted override target rate
        /// from overridden gyros and the non-override authority pool sum
        /// ( step 2; the game's <c>RecomputeGyroParameters</c>,
        /// MyGridGyroSystem.cs:447-476). Resets the acceleration ramp.
        /// </summary>
        public void Aggregate()
        {
            _maxGyroForce = 0f;
            _maxOverrideForce = 0f;
            _overrideTargetVelocity = Vector3.Zero;
            _overrideAccelerationRampFrames = -1;

            for (int i = 0; i < _gyros.Count; i++)
            {
                SimGyro gyro = _gyros[i];
                if (gyro.Closed)
                {
                    continue;
                }
                if (!gyro.IsUsed)
                {
                    continue;
                }
                if (!gyro.IsOverridden || AutopilotEnabled)
                {
                    _maxGyroForce += gyro.MaxGyroForce;
                }
                else
                {
                    _overrideTargetVelocity += gyro.GyroOverrideVelocityGrid * gyro.MaxGyroForce;
                    _maxOverrideForce += gyro.MaxGyroForce;
                }
            }
            if (_maxOverrideForce != 0f)
            {
                _overrideTargetVelocity /= _maxOverrideForce;
            }
            _lastOverrideTarget = _overrideTargetVelocity;
        }

        /// <summary>
        /// Tick phase 6: reproduce the game's override loop
        /// (<c>MyGridGyroSystem.UpdateOverriddenGyros</c>, lines 274-304):
        /// read the body's world angular velocity into the grid frame, apply
        /// the SE override acceleration-ramp policy, convert the rate error
        /// into a desired torque via the inverse inertia diagonal, clamp to
        /// the aggregate gyro-force sphere and supplied power, and apply ONE
        /// resultant body-space angular impulse (matching how  applies
        /// the resultant force).
        /// </summary>
        public void ApplyTorque(float dtSeconds)
        {
            if (_grid.IsDisposed)
            {
                return;
            }
            SimRigidBody body = _grid.Body;
            if (body == null || body.IsDisposed || !body.IsDynamic)
            {
                return;
            }

            // The override loop only runs when there is override force
            // (the game's Update() routes to UpdateOverriddenGyros only
            // when m_maxOverrideForce != 0; with none it uses the
            // separate manual path, which we model via ControlTorque).
            if (_maxOverrideForce <= 0f)
            {
                return;
            }

            // The gyro-system supplied power ratio: the force-weighted
            // phase-4 DeliveredRatio of the used gyros (the exact
            // per-gyro value when they share a tier; the game's single
            // ResourceSink.SuppliedRatioByType). Gate on power ("num > 0").
            float num = GyroPowerRatio();
            if (!(num > 0f))
            {
                return;
            }

            // 3. Read the body's angular velocity into the grid frame:
            // R^T * w_world (MyGridGyroSystem.cs:285).
            Matrix orientation = ((Matrix)_grid.WorldMatrix).GetOrientation();
            Vector3 vector = Vector3.Transform(body.AngularVelocity, orientation);

            // 4-5. Rate error, ramp policy, error -> desired torque.
            Vector3 vector2 = _overrideTargetVelocity - vector;
            if (vector2 == Vector3.Zero)
            {
                return;
            }
            UpdateOverrideAccelerationRampFrames(vector2);
            float rampFrames = _overrideAccelerationRampFrames <= 0 ? 1f : _overrideAccelerationRampFrames;
            Vector3 vector3 = vector2 * (60f / rampFrames);
            Vector3 vector4 = body.InverseInertiaDiagonal;
            Vector3 vector5 = vector3 / vector4; // / (1/I) = x I: desired torque

            // 7. Clamp to the aggregate gyro-force sphere and add the
            // non-override authority (ControlTorque), then scale by power.
            float radius = _maxOverrideForce + _maxGyroForce * (1f - _controlTorque.Length());
            Vector3 vector6 = Vector3.ClampToSphere(vector5, radius);
            _lastSaturated = vector5.Length() > radius;
            Vector3 torque = _controlTorque * _maxGyroForce + vector6;
            torque *= num;

            // 8. Apply ONE resultant body-space angular impulse (grid ->
            // world), torque over the fixed step (game: AddForce torque,
            // which integrates over the frame like 's force).
            _lastGridTorque = torque;
            if (torque.LengthSquared() >= TorqueSqLenThreshold && dtSeconds > 0f)
            {
                Vector3 worldTorque = Vector3.TransformNormal(torque, (Matrix)_grid.WorldMatrix);
                body.ApplyAngularImpulse(worldTorque * dtSeconds);
            }
        }

        /// <summary>
        /// Tick phase 8: publish each used gyro's delivered torque and
        /// saturation for observation (). Override gyros receive
        /// the aggregate torque scaled by their force share of the override
        /// pool; non-override gyros report none (their force is pooled
        /// into the authority, not an individual block torque).
        /// </summary>
        public void UpdateGyroTelemetry()
        {
            float torqueMag = _lastGridTorque.Length();
            for (int i = 0; i < _gyros.Count; i++)
            {
                SimGyro gyro = _gyros[i];
                if (gyro.Closed)
                {
                    continue;
                }
                if (!gyro.IsUsed || !gyro.IsOverridden || _maxOverrideForce <= 0f)
                {
                    gyro.CommitTelemetry(0f, false);
                    continue;
                }
                float share = gyro.MaxGyroForce / _maxOverrideForce;
                gyro.CommitTelemetry(torqueMag * share, _lastSaturated);
            }
        }

        /// <summary>
        /// The SE override acceleration-ramp frame count for a NEW
        /// velocity difference (the game's
        /// <c>UpdateOverrideAccelerationRampFrames</c> initial value,
        /// MyGridGyroSystem.cs:306-324): 120 frames for a large error,
        /// else (int)(|diff|² · 48.228886) + 1. Exposed for the 
        /// ramp conformance.
        /// </summary>
        internal static int ComputeOverrideRampFrames(Vector3 velocityDiff)
        {
            float sq = velocityDiff.LengthSquared();
            if (sq > RampFrameSqThreshold)
            {
                return LargeRateRampFrames;
            }
            return (int)(sq * RampFrameScale) + 1;
        }

        /// <summary>
        /// The game's stateful ramp: on a NEW error the frame count is
        /// <see cref="ComputeOverrideRampFrames"/>; on subsequent ticks it
        /// counts down (but not below 1).
        /// </summary>
        private void UpdateOverrideAccelerationRampFrames(Vector3 velocityDiff)
        {
            if (_overrideAccelerationRampFrames == -1)
            {
                _overrideAccelerationRampFrames = ComputeOverrideRampFrames(velocityDiff);
            }
            else if (_overrideAccelerationRampFrames > 1)
            {
                _overrideAccelerationRampFrames--;
            }
        }

        /// <summary>
        /// The force-weighted phase-4 delivered power ratio of the used
        /// gyros: (sum MaxGyroForce·DeliveredRatio) / (sum MaxGyroForce).
        /// Reduces to the exact per-gyro phase-4 value when all gyros
        /// share a tier (the common case); the single source of truth the
        /// brief's "shared power shortfall scales available gyro force".
        /// </summary>
        private float GyroPowerRatio()
        {
            float weighted = 0f;
            float total = 0f;
            for (int i = 0; i < _gyros.Count; i++)
            {
                SimGyro gyro = _gyros[i];
                if (gyro.Closed || !gyro.IsUsed)
                {
                    continue;
                }
                float f = gyro.MaxGyroForce;
                weighted += f * gyro.DeliveredRatio;
                total += f;
            }
            return total <= 0f ? 0f : weighted / total;
        }
    }
}
