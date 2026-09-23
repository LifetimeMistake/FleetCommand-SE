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
        /// <summary>Proportional rotation gain (per-second), remapped through the gyro torque
        /// channel.</summary>
        private const double RotationGain = 0.8;

        /// <summary>Maximum commanded rotation (rad/s), clamped into the gyro's unit control
        /// sphere.</summary>
        private const double MaxRotationSpeed = 4.0;

        /// <summary>Velocity damping on the turn: the command is reduced by this factor times the
        /// current grid-frame rotation about the error axis, so the turn decelerates as it
        /// approaches the target — the rotational analog of the game's <c>GetAngleVelocity</c>
        /// smooth-rotation deceleration.</summary>
        private const double RotationDamping = 0.5;

        /// <summary>Rotation deadzone (rad): below this the turn is considered aligned and
        /// stops.</summary>
        private const double RotationDeadzone = 0.02;

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

            Vector3D refPos = body.PositionWorld;
            Vector3D targetPos = remote.CurrentWaypoint.Coords;
            Vector3D delta = targetPos - refPos;
            double dist = delta.Length();
            if (dist < _grid.World.Profile.AutopilotArrivalDistance)
            {
                remote.AdvanceWaypoint();
                return;
            }

            _flightActive = true;

            // The grid's thrust capacities must be current before the AI
            // thrust reads them (mirror BaseControlSystem's recalc).
            _grid.ForceAggregator.RecalculateCapacities();

            // ---- the turn: UpdateGyro -> gyroSystem.ControlTorque (:2472-2497) ----
            _grid.GyroRateSystem.ControlTorque = ComputeTurnTorque(body, delta);

            // ---- the translate: UpdateThrust -> ComputeAiThrust (:2878/:1657) ----
            Vector3 controlThrust = ComputeAutoPilotControlThrust(body, delta, remote.SpeedLimit);
            if (controlThrust == Vector3.Zero)
            {
                return; // coasting this tick (at speed cap / braking)
            }
            Vector3 aiForce = ComputeAiThrust(controlThrust, body, remote);
            _lastAutoPilotThrust = aiForce;
            Vector3 maxPos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 maxNeg = _grid.ForceAggregator.MaxNegativeThrust;
            _grid.ForceAggregator.SetGroupDemand(MathHelpers.Normalize(aiForce, maxPos, maxNeg));
        }

        /// <summary>Tick phase 8 (after the physics interval): clamp the grid's velocity to the
        /// enabled remote's <see cref="SimRemoteControl.SpeedLimit"/> — the game's
        /// <c>MovementBlockMaxSpeed = SpeedLimit</c> (MyAutopilotComponent.cs:1322) plus the
        /// per-frame grid velocity clamp (enforced by Havok via <c>RigidBody.MaxLinearVelocity</c>,
        /// MyGridPhysics.SetDefaultRigidBodyMaxVelocities :538-555). Applied after physics so the
        /// cap is tight on the observed post-step velocity (the per-tick thrust alone can overshoot
        /// by one frame's impulse). With no enabled remote (autopilot off) it is a no-op.</summary>
        public void ApplySpeedLimitAfterPhysics()
        {
            SimRemoteControl remote = EnabledRemote();
            if (remote == null)
            {
                return;
            }
            float limit = remote.SpeedLimit;
            if (limit <= 0f)
            {
                return;
            }
            SimRigidBody body = _grid.Body;
            if (body == null || body.IsDisposed || !body.IsDynamic)
            {
                return;
            }
            body.ClampLinearSpeed(limit);
        }

        /// <summary>The turn: an angular command about the rotation-from-current-forward-to-target
        /// axis, magnitude proportional to the remaining angle (capped, with a deadzone), written
        /// into the gyro's ControlTorque channel. This is the game's end state — "drive
        /// ControlTorque to turn toward the target" (:2472-2497) — using a proportional-rate law
        /// for the required rotation.</summary>
        private Vector3 ComputeTurnTorque(SimRigidBody body, Vector3D delta)
        {
            Vector3D fwd = (Vector3D)_grid.WorldMatrix.Forward;
            Vector3D toTarget = Vector3D.Normalize(delta);
            Vector3D axisW = Vector3D.Cross(fwd, toTarget);
            double sinA = axisW.Length();
            double cosA = Vector3D.Dot(fwd, toTarget);
            double angle = Math.Atan2(sinA, cosA);
            if (angle < RotationDeadzone || sinA <= 1e-12)
            {
                return Vector3.Zero;
            }
            Matrix worldToGrid = _grid.WorldToLocal;
            Vector3 axisGrid = Vector3.TransformNormal((Vector3)(axisW / sinA), worldToGrid);
            Vector3 wGrid = Vector3.Transform(body.AngularVelocity, worldToGrid);
            double wAlong = Vector3.Dot(wGrid, axisGrid);
            double rate = angle * RotationGain - wAlong * RotationDamping;
            if (rate < 0.0)
            {
                rate = 0.0;
            }
            else if (rate > MaxRotationSpeed)
            {
                rate = MaxRotationSpeed;
            }
            Vector3 cmd = axisGrid * (float)rate;
            cmd = Vector3.Clamp(cmd, -Vector3.One, Vector3.One);
            _lastControlTorque = cmd;
            return cmd;
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
            Matrix worldToGrid = _grid.WorldToLocal;
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
                Vector3 gridDelta = Vector3.Transform((Vector3)delta, worldToGrid);
                Vector3 gridPerp = Vector3.Transform((Vector3)velPerp, worldToGrid);
                Vector3 dir = gridDelta - gridPerp;
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
            Matrix worldToGrid = _grid.WorldToLocal;
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
            Vector3D dirNorm;
            if (!Vector3.IsZero(direction))
            {
                dirNorm = Vector3D.Normalize(new Vector3D(direction));
                reject = Vector3D.Reject(new Vector3D(vector12), dirNorm);
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

        private static float MaxComponent(Vector3 v)
        {
            return Math.Max(Math.Max(v.X, v.Y), v.Z);
        }
    }
}
