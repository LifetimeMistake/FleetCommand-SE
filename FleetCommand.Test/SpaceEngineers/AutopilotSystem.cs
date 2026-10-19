using System;
using System.Collections.Generic;
using FleetCommand.Common;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The grid's remote-control autopilot flight: when the grid's designated remote
    /// (<see cref="SimRemoteControl.IsAutoPilotEnabled"/>) has a target waypoint, it turns the grid
    /// toward the target (<c>UpdateGyro</c> → <c>gyroSystem.ControlTorque</c>,
    /// MyAutopilotComponent.cs:2472-2497) and translates toward it through the AI thrust command
    /// path (<c>UpdateThrust</c> → <c>AutoPilotControlThrust</c> →
    /// <c>MyEntityThrustComponent.ComputeAiThrust</c>, :2878 / MyEntityThrustComponent.cs:1657),
    /// with <see cref="SimRemoteControl.SpeedLimit"/> capping the forward throttle (<c>|v|² &lt;
    /// maxSpeed²</c>, :2805). <see cref="ControlTorque"/> flows through the SAME gyro channel as
    /// the controller's rotation control, so the two are mutually exclusive on the grid the way the
    /// game's last-writer-wins per-frame write is.</summary>
    public sealed class AutopilotSystem
    {
        // The turn is the game's adaptive-rotation-speed + deadzone + angular-authority law from
        // MyAutopilotComponent.UpdateGyro (:2439-2500), using the flight-movement definition's
        // constants (EntityComponents.sbc "SmallFlightMovement"/"LargeFlightMovement"): the
        // rotation speed is a linear ramp from MinRotationSpeed (at/below MinRotationSpeedThreshold)
        // to MaxRotationSpeed (at/above MaxRotationSpeedThreshold) (:2451-2471), cap the demanded
        // rate by the grid's angular authority (drive only while the time to finish the turn at the
        // current rate still exceeds the time to stop it, :2479-2482), and stop below the rotation
        // deadzone (:2474-2478). The rate is written to ControlTorque unmodified;
        // UpdateGyros later clamps its components, not its magnitude, to ±1.

        /// <summary>Minimum adaptive rotation speed cap (rad/s): at remaining angles at or below
        /// <see cref="MinRotationSpeedThreshold"/> the commanded turn rate is this value
        /// (EntityComponents.sbc <c>MinRotationSpeed</c>, MyAutopilotComponent.cs:2457).</summary>
        private const double MinRotationSpeed = 0.5;

        /// <summary>Maximum adaptive rotation speed cap (rad/s): at remaining angles at or above
        /// <see cref="MaxRotationSpeedThreshold"/> the commanded turn rate is this value — since it
        /// is ≥ 1 its driven components can saturate the gyro's applied torque
        /// (EntityComponents.sbc <c>MaxRotationSpeed</c>, MyAutopilotComponent.cs:2461).</summary>
        private const double MaxRotationSpeed = 2.0;

        /// <summary>Angle (rad) at or below which the commanded turn rate is
        /// <see cref="MinRotationSpeed"/> (EntityComponents.sbc
        /// <c>MinRotationSpeedThresholdRadians</c>).</summary>
        private const double MinRotationSpeedThreshold = 0.2;

        /// <summary>Angle (rad) at or above which the commanded turn rate is
        /// <see cref="MaxRotationSpeed"/> (EntityComponents.sbc
        /// <c>MaxRotationSpeedTresholdRadians</c>).</summary>
        private const double MaxRotationSpeedThreshold = 1.0;

        /// <summary>Rotation deadzone (rad): below this remaining angle the turn is considered
        /// aligned and no torque is commanded (EntityComponents.sbc <c>RotationDeadzone</c>,
        /// MyAutopilotComponent.cs:2474).</summary>
        private const double RotationDeadzone = 0.03;

        /// <summary>The waypoint's stopping-time multiplier (game: <c>StoppingTimeMultiplier</c>,
        /// default 1).</summary>
        private const double StoppingTimeMultiplier = 1.0;

        /// <summary>Threshold below which the AI thrust num check is treated as none (game:
        /// 0.001).</summary>
        private const float AiThrustEpsilon = 0.001f;

        private readonly SimCubeGrid _grid;
        private readonly List<SimRemoteControl> _remotes = new List<SimRemoteControl>();

        private Vector3 _lastControlTorque;
        private Vector3 _lastAutoPilotThrust;
        private float _movementBlockMaxSpeed;
        private bool _flightActive;

        /// <summary>The rigid grid (ship) this autopilot serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>True when the autopilot produced flight commands on the last
        /// <see cref="ComputeAutopilot"/>.</summary>
        public bool FlightActive => _flightActive;

        /// <summary>The turn command written to <see cref="GyroRateSystem.ControlTorque"/> (grid
        /// frame).</summary>
        public Vector3 LastControlTorque => _lastControlTorque;

        /// <summary>The grid-frame AI thrust force (N) fed to the aggregator on the last
        /// <see cref="ComputeAutopilot"/> (the game's <c>FuelTypeData.CurrentThrust</c> from
        /// <c>ComputeAiThrust</c>), before normalization to group demand.</summary>
        public Vector3 LastAutoPilotThrust => _lastAutoPilotThrust;

        /// <summary>The published <c>MovementBlockMaxSpeed</c> (the remote's
        /// <see cref="SimRemoteControl.SpeedLimit"/>) while the autopilot flies, else 0
        /// (MyAutopilotComponent.cs:1322/1333).</summary>
        public float MovementBlockMaxSpeed => _movementBlockMaxSpeed;

        internal AutopilotSystem(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers a remote control block (spawn).</summary>
        internal void RegisterRemote(SimRemoteControl remote)
        {
            if (remote == null)
            {
                throw new ArgumentNullException(nameof(remote));
            }
            _remotes.Add(remote);
        }

        /// <summary>Removes a remote control block (scenario block removal).</summary>
        internal void UnregisterRemote(SimRemoteControl remote)
        {
            if (remote != null)
            {
                _remotes.Remove(remote);
            }
        }

        /// <summary>Tick phase 5 (after <see cref="BaseControlSystem"/>, before
        /// <see cref="ForceAggregator.Aggregate"/> and <see cref="GyroRateSystem.Aggregate"/>): fly
        /// the enabled remote's autopilot — write the turn into
        /// <see cref="GyroRateSystem.ControlTorque"/> and the AI thrust force into the
        /// <see cref="ForceAggregator"/> group demand. With no enabled remote (or no target) it
        /// contributes nothing (the ship coasts and the controller's rotation stands).</summary>
        public void ComputeAutopilot()
        {
            _flightActive = false;
            _lastControlTorque = Vector3.Zero;
            _lastAutoPilotThrust = Vector3.Zero;

            SimRemoteControl remote = EnabledRemote();
            if (remote == null)
            {
                _movementBlockMaxSpeed = 0f;
                return;
            }
            _movementBlockMaxSpeed = remote.SpeedLimit;

            SimRigidBody body = _grid.Body;
            if (body == null || body.IsDisposed || !body.IsDynamic)
            {
                return;
            }

            if (!remote.HasTarget)
            {
                return;
            }

            // CalculateDeltaPos uses ReferencePosition, i.e. the remote block,
            // rather than the grid origin or physical center of mass.
            Vector3D refPos = remote.GetPosition();
            Vector3D targetPos = remote.CurrentWaypoint.Coords;
            Vector3D delta = targetPos - refPos;
            double dist = delta.Length();
            // The waypoint-advance radius: the profile's override when set, else the game's
            // default, gridSize * 3 for intermediate waypoints. Final OneWay
            // destinations use the smaller grid-size-specific trigger radius.
            float arrival = ArrivalDistance(remote);
            if (dist < arrival)
            {
                remote.AdvanceWaypoint();
                return;
            }

            _flightActive = true;

            // The grid's thrust capacities must be current before the AI
            // thrust reads them (mirror BaseControlSystem's recalc).
            _grid.ForceAggregator.RecalculateCapacities();

            // ---- the turn: UpdateGyro -> gyroSystem.ControlTorque (:2472-2497) ----
            _grid.GyroRateSystem.ControlTorque = ComputeTurnTorque(body, remote, delta);

            // ---- the translate: UpdateThrust -> ComputeAiThrust (:2878/:1657) ----
            Vector3 controlThrust = ComputeAutoPilotControlThrust(body, delta, remote.SpeedLimit);
            // AutoPilotControlThrust's setter marks the game's thrust component
            // dirty even for a zero command (via its dampener setter). The dirty
            // update activates physics before the activeOnly force application.
            body.Activate();
            // A zero drive command still needs ComputeAiThrust: it brakes the
            // complete velocity when the speed/stopping-distance gate closes.
            Vector3 aiForce = ComputeAiThrust(controlThrust, body, remote);
            _lastAutoPilotThrust = aiForce;
            Vector3 maxPos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 maxNeg = _grid.ForceAggregator.MaxNegativeThrust;
            _grid.ForceAggregator.SetGroupDemand(MathHelpers.Normalize(aiForce, maxPos, maxNeg));
        }

        /// <summary>The turn: an angular command about the rotation-from-current-forward-to-target
        /// axis, magnitude the game's adaptive rotation speed (<see cref="AdaptiveRotationSpeed"/>)
        /// capped by the grid's angular authority, stopped below the rotation deadzone, written
        /// into the gyro's ControlTorque channel. Implements the ordinary vacuum,
        /// Forward-facing path of MyAutopilotComponent.UpdateGyro, including its
        /// quaternion error and GetAngularVelocity stopping prediction.</summary>
        private Vector3 ComputeTurnTorque(SimRigidBody body, SimRemoteControl remote, Vector3D delta)
        {
            MatrixD orientation = remote.WorldMatrix.GetOrientation();
            Vector3D toTarget = Vector3D.Normalize(delta);
            double angle = Math.Acos(MathHelper.Clamp(Vector3D.Dot(toTarget, orientation.Forward), -1.0, 1.0));
            if (angle < RotationDeadzone) return Vector3.Zero;

            // Ordinary vacuum UpdateGyro/GetAngleVelocity: use quaternion error
            // so opposite headings and rotated remote mounts remain well defined.
            QuaternionD current = QuaternionD.CreateFromRotationMatrix(orientation);
            QuaternionD target = QuaternionD.CreateFromForwardUp(toTarget, orientation.Up);
            current.Conjugate();
            QuaternionD error = target * current;
            double errorAngle = 2 * Math.Acos(MathHelper.Clamp(error.W, -1.0, 1.0));
            if (errorAngle > Math.PI) errorAngle -= 2 * Math.PI;
            Vector3D axis = new Vector3D(error.X, error.Y, error.Z);
            Vector3D angularCommandWorld = errorAngle * axis / axis.Length();
            Vector3D velocity = body.AngularVelocity;
            Vector3D closingVelocity = angularCommandWorld * Vector3D.Dot(velocity, angularCommandWorld);
            Matrix worldToGrid = WorldToGrid();
            Vector3D command = Vector3D.Transform(angularCommandWorld, worldToGrid);
            command = Vector3D.ClampToSphere(command, (float)AdaptiveRotationSpeed(angle), true);
            Vector3D predicted = _grid.GyroRateSystem.PredictAngularVelocity(-command);
            double stopping = (velocity / (velocity - predicted)).Max();
            double closing = angle / closingVelocity.Length() * angle;
            if (double.IsNaN(stopping) || double.IsInfinity(closing) || closing > stopping)
                _lastControlTorque = command;
            // The game's command is not clamped here to a unit sphere. Its gyro
            // application path clamps each torque component later.
            return _lastControlTorque;
        }

        /// <summary>The game's adaptive rotation speed (rad/s): <see cref="MinRotationSpeed"/>
        /// at or below <see cref="MinRotationSpeedThreshold"/>, <see cref="MaxRotationSpeed"/>
        /// at or above <see cref="MaxRotationSpeedThreshold"/>, linear between — the piecewise
        /// ramp from MyAutopilotComponent.UpdateGyro :2451-2471.</summary>
        private double AdaptiveRotationSpeed(double angle)
        {
            if (angle <= MinRotationSpeedThreshold)
            {
                return MinRotationSpeed;
            }
            if (angle >= MaxRotationSpeedThreshold)
            {
                return MaxRotationSpeed;
            }
            double t = (angle - MinRotationSpeedThreshold) / (MaxRotationSpeedThreshold - MinRotationSpeedThreshold);
            return MinRotationSpeed + t * (MaxRotationSpeed - MinRotationSpeed);
        }

        /// <summary>The forward thrust command — the game's <c>UpdateThrust</c> (:2732-2885) no-CA
        /// core, in the grid frame: a unit direction toward the target minus the cross-track
        /// velocity, gated by the stopping-time vs time-to-target braking rule and the <c>|v|² &lt;
        /// maxSpeed²</c> speed cap (:2805).</summary>
        private Vector3 ComputeAutoPilotControlThrust(SimRigidBody body, Vector3D delta, float maxSpeed)
        {
            if (maxSpeed <= 0f)
            {
                return Vector3.Zero;
            }
            Matrix worldToGrid = WorldToGrid();
            Vector3D v = delta;
            v.Normalize();
            Vector3D vel = body.LinearVelocity;

            Vector3 vector3 = Vector3.Transform((Vector3)v, worldToGrid);
            Vector3 zero = new Vector3(
                MaxThrustAlong(vector3.X >= 0f),
                MaxThrustAlongY(vector3.Y >= 0f),
                MaxThrustAlongZ(vector3.Z >= 0f));
            float num = Math.Abs(vector3.X) * zero.X + Math.Abs(vector3.Y) * zero.Y + Math.Abs(vector3.Z) * zero.Z;
            if (num <= 0f)
            {
                return Vector3.Zero; // no thrust in the needed direction
            }
            if (vel.Length() > 3.0 && Vector3D.Dot(vel, v) < 0.0)
            {
                return Vector3.Zero; // fast and moving away: coast (:2779)
            }

            Vector3D v2 = Vector3D.CalculatePerpendicularVector(v);
            Vector3D v3 = Vector3D.Cross(v, v2);
            Vector3D velAlong = v * Vector3D.Dot(vel, v);
            Vector3D velPerp = v2 * Vector3D.Dot(vel, v2) + v3 * Vector3D.Dot(vel, v3);
            double velAlongMag = velAlong.Length();
            double timeToTarget = velAlongMag > 1e-9 ? delta.Length() / velAlongMag : double.PositiveInfinity;
            Vector3D gravity = (Vector3D)_grid.World.Gravity.GetNaturalGravity(body.PositionWorld);
            double timeToStop = StoppingTimeMultiplier * (vel + gravity).Length() * (double)body.Mass / (double)num;

            if ((double.IsInfinity(timeToTarget) || double.IsInfinity(timeToStop) || double.IsNaN(timeToStop) || timeToTarget > timeToStop)
                && vel.LengthSquared() < (double)maxSpeed * (double)maxSpeed)
            {
                // Transform(delta) - Transform(velPerp), normalized: the
                // grid-frame thrust direction toward the target minus the
                // cross-track drift (rotation-invariant difference).
                // UpdateThrust keeps delta and cross-track velocity in double
                // precision through both transforms, casting only their difference.
                Vector3 dir = Vector3D.Transform(delta, worldToGrid) - Vector3D.Transform(velPerp, worldToGrid);
                if (dir.LengthSquared() <= 1e-12f)
                {
                    return Vector3.Zero;
                }
                dir.Normalize();
                return dir;
            }
            return Vector3.Zero;
        }

        /// <summary>The AI thrust force (grid frame, N) — the game's <c>ComputeAiThrust</c>
        /// (MyEntityThrustComponent.cs:1657) for the framework's single-fuel-group case: the
        /// direction's thrust plus gravity compensation, with the cross-track velocity-dampening
        /// term (applyDampeners always true for autopilot control —
        /// AutoPilotControlThrustDampenersEnabled, :329-363), all clamped to
        /// ±Max·SlowdownFactor.</summary>
        private Vector3 ComputeAiThrust(Vector3 direction, SimRigidBody body, SimRemoteControl remote)
        {
            Matrix worldToGrid = WorldToGrid();
            Vector3 vector = Vector3.Clamp(direction, Vector3.Zero, Vector3.One);
            Vector3 vector2 = Vector3.Clamp(direction, -Vector3.One, Vector3.Zero);
            Vector3 maxPos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 maxNeg = _grid.ForceAggregator.MaxNegativeThrust;
            Vector3 gravity = (Vector3)_grid.World.Gravity.GetNaturalGravity(body.PositionWorld);
            Vector3 gLocal = Vector3.Transform(gravity, worldToGrid);
            Vector3 value = -gLocal * body.Mass;
            Vector3 positiveGravity = Vector3.Clamp(value, Vector3.Zero, Vector3.PositiveInfinity);
            Vector3 negativeGravity = Vector3.Clamp(value, Vector3.NegativeInfinity, Vector3.Zero);

            Vector3 vector3 = Vector3.Clamp(maxPos - positiveGravity, Vector3.Zero, Vector3.PositiveInfinity);
            Vector3 vector4 = Vector3.Clamp(maxNeg + negativeGravity, Vector3.Zero, Vector3.PositiveInfinity);
            Vector3 vector5 = vector3 * vector;
            Vector3 vector6 = vector4 * -vector2;
            float num = Math.Max(MaxComponent(vector5), MaxComponent(vector6));
            Vector3 vector7 = Vector3.Zero;
            if (num > AiThrustEpsilon)
            {
                Vector3 vector8 = vector * vector5;
                Vector3 vector9 = -vector2 * vector6;
                Vector3 vector10 = MathHelpers.SafeRatio(vector3, vector8);
                Vector3 vector11 = MathHelpers.SafeRatio(vector4, vector9);
                vector7 = -vector9 * vector11 + vector8 * vector10;
                vector7 += positiveGravity + negativeGravity;
                vector7 = Vector3.Clamp(vector7, -maxNeg, maxPos);
            }

            // Single-fuel-group dampening (:1681-1695): cancel the
            // velocity (plus half gravity) perpendicular to the thrust
            // direction, with the game's /0.5 time constant.
            Vector3 vector12 = Vector3.Transform((Vector3)body.LinearVelocity + gravity * 0.5f, worldToGrid);
            Vector3D reject;
            if (!Vector3.IsZero(direction))
            {
                // ComputeAiThrust normalizes/rejects as Vector3, then widens
                // the result for the mass/time calculation.
                Vector3 dirNorm = Vector3.Normalize(direction);
                reject = Vector3.Reject(vector12, dirNorm);
            }
            else
            {
                reject = new Vector3D(vector12);
            }
            Vector3D v2 = -reject / 0.5;
            Vector3D v3 = v2 * (double)body.Mass;
            float slowdown = _grid.ForceAggregator.SlowdownFactor;
            vector7 = Vector3.Clamp(vector7 + (Vector3)v3, -maxNeg * slowdown, maxPos * slowdown);
            return vector7;
        }

        /// <summary>The max thrust (N) the grid can deliver along +X (positive) or -X.</summary>
        private float MaxThrustAlong(bool positive)
        {
            Vector3 pos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 neg = _grid.ForceAggregator.MaxNegativeThrust;
            return positive ? pos.X : neg.X;
        }

        private float MaxThrustAlongY(bool positive)
        {
            Vector3 pos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 neg = _grid.ForceAggregator.MaxNegativeThrust;
            return positive ? pos.Y : neg.Y;
        }

        private float MaxThrustAlongZ(bool positive)
        {
            Vector3 pos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 neg = _grid.ForceAggregator.MaxNegativeThrust;
            return positive ? pos.Z : neg.Z;
        }

        private SimRemoteControl EnabledRemote()
        {
            for (int i = 0; i < _remotes.Count; i++)
            {
                SimRemoteControl r = _remotes[i];
                if (!r.Closed && r.IsAutoPilotEnabled)
                {
                    return r;
                }
            }
            return null;
        }

        /// <summary>The waypoint-advance radius (m): the profile's override when positive,
        /// else GetWaypointAdvanceDistance's intermediate gridSize*3 radius or
        /// its smaller final OneWay destination trigger.</summary>
        private float ArrivalDistance(SimRemoteControl remote)
        {
            float overrideDistance = _grid.World.Profile.AutopilotArrivalDistance;
            if (overrideDistance > 0f) return overrideDistance;
            if (remote.FlightMode == Sandbox.ModAPI.Ingame.FlightMode.OneWay && remote.IsLastWaypoint)
                return _grid.GridSize * (_grid.GridSizeEnum == VRage.Game.MyCubeSize.Large ? 0.25f : 1f);
            return _grid.GridSize * 3f;
        }

        private static float MaxComponent(Vector3 v)
        {
            return Math.Max(Math.Max(v.X, v.Y), v.Z);
        }

        Matrix WorldToGrid()
        {
            // WorldMatrixNormalizedInv in the game inverts the double world
            // matrix before narrowing its rotation to Matrix. Transposing an
            // already narrowed matrix is not equivalent for a Havok float pose.
            return (Matrix)MatrixD.Invert(_grid.WorldMatrix.GetOrientation());
        }
    }
}
