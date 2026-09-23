using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The grid gyro-rate system — the boundary where per-gyro override channels become
    /// one body-space angular impulse, reproducing the game's override loop
    /// (<c>MyGridGyroSystem.UpdateOverriddenGyros</c>). Owned by <see cref="SimCubeGrid"/>; the
    /// world drives it in three tick phases: phase 5 <see cref="Aggregate"/> (for each used gyro —
    /// enabled + functional, the game's <c>IsUsed</c> — divert override gyros into the
    /// force-weighted grid-frame target rate, or into the non-override authority pool when
    /// <see cref="AutopilotEnabled"/>, and sum the authority pool, as
    /// <c>RecomputeGyroParameters</c> does, MyGridGyroSystem.cs:447-476); phase 6
    /// <see cref="ApplyTorque"/> (read the body's world angular velocity into the grid frame, apply
    /// the SE override acceleration-ramp policy, convert the rate error into a desired torque via
    /// the inverse inertia diagonal, clamp to the aggregate gyro-force sphere and supplied power,
    /// and apply ONE resultant body-space angular impulse); phase 8
    /// <see cref="UpdateGyroTelemetry"/> (write each gyro's delivered torque and saturation).
    /// <see cref="ControlTorque"/> is the scenario's non-override stabilization input (zero by
    /// default), added as <c>ControlTorque * m_maxGyroForce</c>. Power is a binary gate: a gyro is
    /// either powered (<see cref="SimFunctionalBlock.IsPowered"/>) and contributes its FULL force, or
    /// unpowered and contributes nothing — no partial torque from a partial supply (the game's gyro
    /// resource sink gates on <c>IsPoweredByType</c>, MyResourceSinkComponent.cs:353-362, whose
    /// supplied ratio is 0 on a non-adaptible shortfall and 1 on full supply).</summary>
    public sealed class GyroRateSystem
    {
        /// <summary>Tiny torque threshold below which nothing is applied (game: TorqueSqLenTh =
        /// 0.0001).</summary>
        public const float TorqueSqLenThreshold = 0.0001f;

        /// <summary>The manual path's angular-velocity dampening blend (game:
        /// MyGridGyroSystem.MAX_SLOWDOWN = WELD_LANDING_GEARS ? 0.8 : 0.93).</summary>
        public const float MaxSlowdown = 0.93f;

        /// <summary>Inertia-tensor-limit clamp divisor (game: MyGridGyroSystem.INV_TENSOR_MAX_LIMIT
        /// = 125000).</summary>
        public const float InvTensorMaxLimit = 125000f;

        /// <summary>Slowdown torque multiplier for small and large grids (game: MyFakes 5f /
        /// 2f).</summary>
        public const float SlowdownTorqueMultiplierSmall = 5f;
        public const float SlowdownTorqueMultiplierLarge = 2f;

        /// <summary>Large-rate override ramp frame count (MyGridGyroSystem.cs:313).</summary>
        public const int LargeRateRampFrames = 120;

        /// <summary>Ramp frame scale below which the (short) ramp applies
        /// (MyGridGyroSystem.cs:311).</summary>
        public const float RampFrameSqThreshold = 2.467401f;

        /// <summary>Ramp frame count per unit squared rate error
        /// (MyGridGyroSystem.cs:317).</summary>
        public const float RampFrameScale = 48.228886f;

        private readonly SimCubeGrid _grid;
        private readonly List<SimGyro> _gyros = new List<SimGyro>();

        private float _maxGyroForce;      // non-override authority pool (sum of MaxGyroForce)
        private float _maxOverrideForce;  // override pool (sum of MaxGyroForce)
        private Vector3 _overrideTargetVelocity; // force-weighted grid-frame target rate
        private int _overrideAccelerationRampFrames = -1; // -1 = unset (game: int?)
        private bool _gyrosChanged = true; // re-arm the ramp on the first Aggregate (game: m_gyrosChanged)
        private Vector3 _controlTorque;

        private Vector3 _lastOverrideTarget;
        private Vector3 _lastGridTorque;
        private bool _lastSaturated;

        /// <summary>The rigid grid (ship) this rate system serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>When true, override gyros are diverted into the non-override authority pool
        /// instead of the override pool (the game's <c>AutopilotEnabled</c>). Autopilot tests drive
        /// gyros through the override path (flag false); this models the flag's only effect
        /// here.</summary>
        public bool AutopilotEnabled;

        /// <summary>The scenario's non-override stabilization input (the game's
        /// <c>ControlTorque</c>), added as <c>ControlTorque * m_maxGyroForce</c>. Zero by
        /// default.</summary>
        public Vector3 ControlTorque
        {
            get { return _controlTorque; }
            set { _controlTorque = value; }
        }

        /// <summary>The force-weighted grid-frame target rate from the most recent
        /// <see cref="Aggregate"/> (observation).</summary>
        public Vector3 OverrideTargetVelocity => _lastOverrideTarget;

        /// <summary>The grid-frame torque applied in the most recent <see cref="ApplyTorque"/>
        /// (observation).</summary>
        public Vector3 LastTorque => _lastGridTorque;

        /// <summary>True when the last applied torque was clamped by the aggregate gyro-force
        /// sphere.</summary>
        public bool LastSaturated => _lastSaturated;

        /// <summary>The current override acceleration-ramp frame count (-1 = unset). Observation
        /// surface so conformance can lock the game's cross-tick countdown
        /// (MyGridGyroSystem.cs:306-324): armed once, counts down each tick, re-armed only on a
        /// gyro-set change.</summary>
        internal int OverrideRampFrames => _overrideAccelerationRampFrames;

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
            _gyrosChanged = true;
        }

        /// <summary>Removes a gyro block from this rate system (scenario block removal).</summary>
        internal void UnregisterGyro(SimGyro gyro)
        {
            if (gyro != null)
            {
                _gyros.Remove(gyro);
                _gyrosChanged = true;
            }
        }

        /// <summary>Marks the gyro set as changed so the next <see cref="Aggregate"/> re-arms the
        /// override acceleration ramp (the game's <c>MarkDirty()</c> →
        /// <c>RecomputeGyroParameters</c>, which resets <c>m_overrideAccelerationRampFrames</c>).
        /// Called by <see cref="SimGyro"/> when its override flag or override velocity
        /// changes.</summary>
        internal void MarkDirty()
        {
            _gyrosChanged = true;
        }

        /// <summary>Tick phase 5: aggregate the force-weighted override target rate from overridden
        /// gyros and the non-override authority pool sum (the game's
        /// <c>RecomputeGyroParameters</c>, MyGridGyroSystem.cs:447-476). Re-arms the acceleration
        /// ramp ONLY when the gyro set changed (the game resets
        /// <c>m_overrideAccelerationRampFrames</c> only on <c>RecomputeGyroParameters</c>, not
        /// every tick), so the ramp's cross-tick countdown persists while the configuration is
        /// stable.</summary>
        public void Aggregate()
        {
            _maxGyroForce = 0f;
            _maxOverrideForce = 0f;
            _overrideTargetVelocity = Vector3.Zero;
            if (_gyrosChanged)
            {
                _overrideAccelerationRampFrames = -1;
                _gyrosChanged = false;
            }

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
                if (!gyro.IsPowered)
                {
                    // Binary power gate: an unpowered gyro contributes no
                    // force to either pool; a powered gyro contributes its
                    // full MaxGyroForce. There is no partial contribution
                    // from a partial supply (the game's gyro resource sink
                    // is powered or not — IsPoweredByType,
                    // MyResourceSinkComponent.cs:353-362 — and unpowered
                    // gyros are not working, MyGyro.cs:105,236).
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

        /// <summary>Tick phase 6: reproduce the game's override loop
        /// (<c>MyGridGyroSystem.UpdateOverriddenGyros</c>, lines 274-304): read the body's world
        /// angular velocity into the grid frame, apply the SE override acceleration-ramp policy,
        /// convert the rate error into a desired torque via the inverse inertia diagonal, clamp to
        /// the aggregate gyro-force sphere and supplied power, and apply ONE resultant body-space
        /// angular impulse.</summary>
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

            // The game's Update() routes on maxOverrideForce
            // (MyGridGyroSystem.cs:385-404): override gyros present ->
            // the override loop; otherwise the separate manual path
            // (UpdateGyros, :201-272) which consumes ControlTorque and
            // provides angular-velocity dampening. This is the route the
            // controller's rotation control (Part 1) and the autopilot's
            // turn (Part 2) flow through.
            if (_maxOverrideForce > 0f)
            {
                ApplyOverrideTorque(body, dtSeconds);
            }
            else if (_maxGyroForce > 0f)
            {
                ApplyManualTorque(body, dtSeconds);
            }
        }

        /// <summary>The game's override loop (<c>MyGridGyroSystem.UpdateOverriddenGyros</c>,
        /// :274-304) — the override-acceleration-ramp policy when override gyros are present.
        /// Extracted from the former single <see cref="ApplyTorque"/> body.</summary>
        private void ApplyOverrideTorque(SimRigidBody body, float dtSeconds)
        {
            // Binary power gate: 1 while any used gyro is powered, 0 when
            // none is (the game gates on the resource-sink supplied ratio
            // being > 0, MyGridGyroSystem.cs:276-277, which is 0 or 1 for
            // the non-adaptible gyro sink). Since Aggregate already
            // excludes unpowered gyros from the force pool, the multiplier
            // here is full (1) — an unpowered system returned above.
            float num = GyroPowerRatio();
            if (!(num > 0f))
            {
                return;
            }

            // 3. Read the body's angular velocity into the grid frame:
            // R^T * w_world (MyGridGyroSystem.cs:285; the game reads via
            // WorldMatrixInvScaled = R^T). The mis-transpose (R * w) here
            // was an accumulated rotated-mount sign error, masked by
            // identity-orientation test grids (R = I).
            Matrix orientation = ((Matrix)_grid.WorldMatrix).GetOrientation();
            Vector3 vector = Vector3.Transform(body.AngularVelocity, Matrix.Transpose(orientation));

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
            // which integrates over the frame).
            _lastGridTorque = torque;
            if (torque.LengthSquared() >= TorqueSqLenThreshold && dtSeconds > 0f)
            {
                Vector3 worldTorque = Vector3.TransformNormal(torque, (Matrix)_grid.WorldMatrix);
                body.ApplyAngularImpulse(worldTorque * dtSeconds);
            }
        }

        /// <summary>The game's manual path (<c>MyGridGyroSystem.UpdateGyros</c>, :201-272), routed
        /// to when there are no override gyros: this is what consumes the controller's rotation
        /// control (Part 1) and the autopilot's turn (Part 2) — both write
        /// <see cref="ControlTorque"/> and the single, non-override authority pool drives the grid.
        /// It applies the angular-velocity slowdown (dampening) plus the ControlTorque torque:
        /// <c>torque = clamp(ControlTorque, ±1) · m_maxGyroForce / max(1, num4·125000)</c>, scaled
        /// by the normalized inertia-per-axis and the supplied power (:254-266), with the slowdown
        /// dampening clamped per-gyro-force and gated off on axes the control actively drives
        /// (:231-250).</summary>
        private void ApplyManualTorque(SimRigidBody body, float dtSeconds)
        {
            float num = GyroPowerRatio();
            if (!(num > 0f))
            {
                return;
            }
            if (!_controlTorque.IsValid())
            {
                _controlTorque = Vector3.Zero;
            }

            // world -> grid so the control and the angular velocity share a
            // frame (game :227-229).
            Matrix worldToGrid = _grid.WorldToLocal;
            Vector3 vector = Vector3.Transform(body.AngularVelocity, worldToGrid);
            if (!vector.IsValid())
            {
                vector = Vector3.Zero;
            }

            // num2 = (1 - MAX_SLOWDOWN)·(1 - power) + MAX_SLOWDOWN (:230).
            float num2 = (1f - MaxSlowdown) * (1f - num) + MaxSlowdown;
            Vector3 slowdownTorque = -vector;

            // num3 = per-grid-size slowdown torque multiplier (MyFakes:
            // small 5, large 2) (:232-233). vector2 = maxGyroForce·num3.
            float num3 = _grid.GridSizeEnum == VRage.Game.MyCubeSize.Large
                ? SlowdownTorqueMultiplierLarge
                : SlowdownTorqueMultiplierSmall;
            Vector3 vector2 = new Vector3(_maxGyroForce * num3);

            // Per-axis gating (:243): slowdown is removed on any axis the
            // control torque actively drives (same sign), so the control
            // isn't fighting the dampener.
            Vector3 vector3 = Vector3.One - OneWhereZero(Vector3.Sign(vector) - Vector3.Sign(_controlTorque));

            Vector3 invInertia = body.InverseInertiaDiagonal;
            slowdownTorque *= num3;
            slowdownTorque = DivideComponentWise(slowdownTorque, invInertia); // / inv-inertia = x inertia -> torque
            slowdownTorque = Vector3.Clamp(slowdownTorque, -vector2, vector2) * vector3;
            if (slowdownTorque.LengthSquared() > 0.0001f && dtSeconds > 0f)
            {
                Vector3 worldSlowdown = Vector3.TransformNormal(slowdownTorque * num2, (Matrix)_grid.WorldMatrix);
                body.ApplyAngularImpulse(worldSlowdown * dtSeconds);
            }

            // The ControlTorque torque (:251-266): clamp to ±1, scale by
            // gyro force and the inverse-tensor limit, then by power and
            // the normalized per-axis inertia.
            float iMax = MaxInertia(invInertia);
            float num4 = iMax > 0f ? 1f / iMax : 0f;
            float num5 = Math.Max(1f, num4 * InvTensorMaxLimit);
            Vector3 torque = Vector3.Clamp(_controlTorque, -Vector3.One, Vector3.One) * _maxGyroForce / num5;
            torque *= num;

            Vector3 scale = NormalizedInertiaScale(invInertia);
            torque *= scale;

            _lastGridTorque = torque;
            if (torque.LengthSquared() >= TorqueSqLenThreshold && dtSeconds > 0f)
            {
                Vector3 worldTorque = Vector3.TransformNormal(torque, (Matrix)_grid.WorldMatrix);
                body.ApplyAngularImpulse(worldTorque * dtSeconds);
            }
            _lastSaturated = false;
        }

        /// <summary>The game's <c>Vector3.IsZeroVector</c> on a vector of signed axis differences
        /// (values in {-2,-1,0,1,2}): 1 where the difference is exactly zero, else 0. Used for the
        /// manual path's per-axis slowdown gating.</summary>
        private static Vector3 OneWhereZero(Vector3 signDiff)
        {
            return new Vector3(
                signDiff.X == 0f ? 1f : 0f,
                signDiff.Y == 0f ? 1f : 0f,
                signDiff.Z == 0f ? 1f : 0f);
        }

        /// <summary>Inertia per axis computed from the inverse-inertia diagonal (the sim's diagonal
        /// inertia model).</summary>
        private static float MaxInertia(Vector3 invInertia)
        {
            float ix = AxisInertia(invInertia.X);
            float iy = AxisInertia(invInertia.Y);
            float iz = AxisInertia(invInertia.Z);
            return Math.Max(Math.Max(ix, iy), iz);
        }

        private static float AxisInertia(float inv)
        {
            return inv <= 1e-9f ? float.MaxValue : 1f / inv;
        }

        /// <summary>The game's normalized inertia-per-axis scale (<c>scale = Abs(inertia /
        /// inertia.AbsMax())</c>, :256-257): the largest inertia axis maps to 1, smaller axes are
        /// scaled proportionally, so the torque follows the body's rotational anisotropy. The sim
        /// models inertia as diagonal-only.</summary>
        private static Vector3 NormalizedInertiaScale(Vector3 invInertia)
        {
            Vector3 inertia = new Vector3(
                AxisInertia(invInertia.X),
                AxisInertia(invInertia.Y),
                AxisInertia(invInertia.Z));
            float absMax = Math.Max(Math.Max(Math.Abs(inertia.X), Math.Abs(inertia.Y)), Math.Abs(inertia.Z));
            if (absMax <= 0f || float.IsInfinity(absMax))
            {
                return Vector3.One;
            }
            return new Vector3(
                Math.Abs(inertia.X) / absMax,
                Math.Abs(inertia.Y) / absMax,
                Math.Abs(inertia.Z) / absMax);
        }

        private static Vector3 DivideComponentWise(Vector3 a, Vector3 b)
        {
            return new Vector3(
                SafeDiv(a.X, b.X),
                SafeDiv(a.Y, b.Y),
                SafeDiv(a.Z, b.Z));
        }

        private static float SafeDiv(float a, float b)
        {
            return b == 0f ? float.MaxValue : a / b;
        }

        /// <summary>Tick phase 8: publish each used gyro's delivered torque and saturation for
        /// observation. Override gyros receive the aggregate torque scaled by their force share of
        /// the override pool; non-override gyros report none (their force is pooled into the
        /// authority, not an individual block torque).</summary>
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
                if (!gyro.IsUsed || !gyro.IsPowered || !gyro.IsOverridden || _maxOverrideForce <= 0f)
                {
                    gyro.CommitTelemetry(0f, false);
                    continue;
                }
                float share = gyro.MaxGyroForce / _maxOverrideForce;
                gyro.CommitTelemetry(torqueMag * share, _lastSaturated);
            }
        }

        /// <summary>The SE override acceleration-ramp frame count for a NEW velocity difference
        /// (the game's <c>UpdateOverrideAccelerationRampFrames</c> initial value,
        /// MyGridGyroSystem.cs:306-324): 120 frames for a large error, else (int)(|diff|² ·
        /// 48.228886) + 1. Exposed for ramp conformance.</summary>
        internal static int ComputeOverrideRampFrames(Vector3 velocityDiff)
        {
            float sq = velocityDiff.LengthSquared();
            if (sq > RampFrameSqThreshold)
            {
                return LargeRateRampFrames;
            }
            return (int)(sq * RampFrameScale) + 1;
        }

        /// <summary>The game's stateful ramp: on a NEW error the frame count is
        /// <see cref="ComputeOverrideRampFrames"/>; on subsequent ticks it counts down (but not
        /// below 1).</summary>
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

        /// <summary>The gyro system's BINARY powered gate (the game's
        /// <c>ResourceSink.SuppliedRatioByType</c> is 0 or 1 for the non-adaptible gyro sink —
        /// MyResourceSinkComponent.SetInputFromDistributor sets 0 on a shortfall and 1 on full
        /// supply, :353-362 — and <c>MyGridGyroSystem.UpdateOverriddenGyros</c> returns when it is
        /// not &gt; 0, :276-277): 1 while any used gyro is powered, else 0. Because
        /// <see cref="Aggregate"/> already excludes unpowered gyros from the force pools, a powered
        /// system here applies its FULL torque — no partial-torque multiplier from a partial
        /// supply.</summary>
        private float GyroPowerRatio()
        {
            for (int i = 0; i < _gyros.Count; i++)
            {
                SimGyro gyro = _gyros[i];
                if (gyro.Closed || !gyro.IsUsed)
                {
                    continue;
                }
                if (gyro.IsPowered)
                {
                    return 1f;
                }
            }
            return 0f;
        }
    }
}
