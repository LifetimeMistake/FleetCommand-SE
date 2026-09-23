using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The grid thrust aggregation step: the boundary where per-thruster commands
    /// (override or group demand) become one resultant body-space force applied at the grid's
    /// center of mass. Owned by <see cref="SimCubeGrid"/>; the world drives it in three tick
    /// phases: phase 5 <see cref="Aggregate"/> (compute <see cref="SimThrust.MaxEffectiveThrust"/>,
    /// convert override or normalized group demand into delivered thrust scaled by
    /// <see cref="SimFunctionalBlock.DeliveredRatio"/>, and sum by grid-body direction); phase 6
    /// <see cref="ApplyThrust"/> (apply ONE resultant body-space linear impulse at the grid COM —
    /// the source-verified application point: MyThrusterBlockThrustComponent.cs:201 AddForce at the
    /// grid's CoM, so fixed grid thrusters generate no torque from block position); phase 8
    /// <see cref="UpdateThrusterTelemetry"/> (commit each thruster's
    /// <see cref="SimThrust.CurrentThrust"/>). A thruster's
    /// <see cref="SimThrust.GridThrustDirection"/> is its grid-body FORWARD (exhaust) axis and the
    /// force is applied along its negation, matching the game's <c>ThrustForce =
    /// -ThrustForwardVector</c> (MyThrust.cs:188). The group demand is a normalized per-body-axis
    /// throttle in [-1, 1]; overridden thrusters answer their override instead of the group demand
    /// (matching the game's IsOverridden / IsUsed). Asymmetric and missing axes need no craft-type
    /// branches — thrusters carry their push direction as data.</summary>
    public sealed class ForceAggregator
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimThrust> _thrusters = new List<SimThrust>();

        private Vector3 _groupDemand;
        private Vector3 _netBodyForce;

        // Per-direction delivered-thrust capacity, recomputed from the
        // registered thrusters so the base-control path (BaseControlSystem)
        // can reproduce the game's ComputeBaseThrust.
        private Vector3 _maxPositiveThrust;
        private Vector3 _maxNegativeThrust;
        private float _slowdownFactor = 1f;

        /// <summary>The rigid grid (ship) this aggregator serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>The normalized group (control) demand, body-frame, components in [-1, 1]: a
        /// positive component fires the thrusters pushing along that axis at the given throttle; a
        /// negative component fires the opposite axis. Scenario-set.</summary>
        public Vector3 GroupDemand => _groupDemand;

        /// <summary>The resultant body-space thrust force (N) from the most recent
        /// <see cref="Aggregate"/>: the net of every thruster's delivered thrust summed by body
        /// direction; zero when nothing fired.</summary>
        public Vector3 LastNetForce => _netBodyForce;

        /// <summary>The grid's maximum deliverable force along each +body axis (N), summed from the
        /// registered thrusters' <see cref="SimThrust.MaxEffectiveThrust"/> (the game's per-group
        /// <c>MaxPositiveThrust</c>, summed from <c>ForceMagnitude</c>). Recomputed by
        /// <see cref="RecalculateCapacities"/>.</summary>
        public Vector3 MaxPositiveThrust => _maxPositiveThrust;

        /// <summary>The grid's maximum deliverable force along each -body axis (N); the counterpart
        /// of <see cref="MaxPositiveThrust"/>.</summary>
        public Vector3 MaxNegativeThrust => _maxNegativeThrust;

        /// <summary>The grid's representative thrust-definition <c>SlowdownFactor</c> (default 1),
        /// used by the base-control path's final clamp
        /// (<c>MyEntityThrustComponent.ComputeBaseThrust</c> :1648). Set by
        /// <see cref="RecalculateCapacities"/> from the first functional thruster's definition
        /// value.</summary>
        public float SlowdownFactor => _slowdownFactor;

        internal ForceAggregator(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Sets the normalized group (control) demand that non-overridden thrusters
        /// answer, clamping each body-axis component to [-1, 1].</summary>
        public void SetGroupDemand(Vector3 bodyDemand)
        {
            _groupDemand = ClampComponents(bodyDemand);
        }

        /// <summary>Registers a thruster block with this aggregator (spawn).</summary>
        internal void RegisterThruster(SimThrust thruster)
        {
            if (thruster == null)
            {
                throw new ArgumentNullException(nameof(thruster));
            }
            _thrusters.Add(thruster);
        }

        /// <summary>Removes a thruster block from this aggregator (scenario block
        /// removal).</summary>
        internal void UnregisterThruster(SimThrust thruster)
        {
            if (thruster != null)
            {
                _thrusters.Remove(thruster);
            }
        }

        /// <summary>Tick phase 5: gate each thruster, convert its override or group demand into
        /// delivered thrust (scaled by the bus's delivered ratio), sum by grid-body direction, and
        /// return the one resultant body-space force. Also stashes each thruster's delivered force
        /// for the phase-8 CurrentThrust write.</summary>
        public Vector3 Aggregate()
        {
            Vector3 net = Vector3.Zero;
            for (int i = 0; i < _thrusters.Count; i++)
            {
                SimThrust thruster = _thrusters[i];
                if (thruster.Closed)
                {
                    continue;
                }
                // The grid is pushed OPPOSITE the thruster's forward, as
                // the game applies ThrustForce = -ThrustForwardVector
                // (MyThrust.cs:188). The force axis is -forward; the net
                // is the sum over thrusters of (force axis x delivered).
                Vector3 forceAxis = -new Vector3(thruster.GridThrustDirection);
                float delivered = ComputeDelivered(thruster, forceAxis);
                thruster.SetLastDelivered(delivered);
                if (delivered > 0f)
                {
                    net += forceAxis * delivered;
                }
            }
            _netBodyForce = net;
            return net;
        }

        /// <summary>Tick phase 6: apply the phase-5 resultant as ONE body-space linear impulse at
        /// the grid center of mass — transformed to the world frame and timed by the fixed step.
        /// Applied at the COM, so no torque arises from any thruster's block position.</summary>
        public void ApplyThrust(float dtSeconds)
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
            Vector3 net = _netBodyForce;
            if (net == Vector3.Zero || dtSeconds <= 0f)
            {
                return;
            }
            // World frame = grid (body-to-world) rotation applied to the
            // body-space resultant; the body's ApplyLinearImpulse applies
            // at the body's center of mass.
            Vector3 worldForce = Vector3.TransformNormal(net, (Matrix)_grid.WorldMatrix);
            body.ApplyLinearImpulse(worldForce * dtSeconds);
        }

        /// <summary>Tick phase 8: after the physics interval, publish each thruster's phase-5
        /// delivered force as its <see cref="SimThrust.CurrentThrust"/>.</summary>
        public void UpdateThrusterTelemetry()
        {
            for (int i = 0; i < _thrusters.Count; i++)
            {
                SimThrust thruster = _thrusters[i];
                if (thruster.Closed)
                {
                    continue;
                }
                thruster.CommitCurrentThrust();
            }
        }

        private float ComputeDelivered(SimThrust thruster, Vector3 forceAxis)
        {
            // 1. Gate: enabled, functional, powered, environmentally
            // effective (effectiveness > 0).
            if (!thruster.Enabled || !thruster.IsFunctional || !thruster.IsPowered)
            {
                return 0f;
            }
            // The template effectiveness is scaled by the world's
            // atmosphere effectiveness: an atmo thruster in vacuum
            // (ratio 0) has no effect.
            float effectiveness = thruster.Effectiveness * _grid.World.AtmosphereEffectiveness;
            if (effectiveness <= 0f)
            {
                return 0f;
            }

            // 3. Convert override or normalized group demand into a
            // demanded force (N). Override is absolute; the group demand
            // is the throttle along the thruster's FORCE axis (its
            // forward) times this thruster's max. Both stay within
            // MaxThrust, so delivered never exceeds MaxEffectiveThrust x
            // the delivered ratio.
            float demand;
            if (thruster.IsOverridden)
            {
                demand = thruster.ThrustOverride;
            }
            else
            {
                demand = GroupDemandAlong(forceAxis) * thruster.MaxThrust;
            }
            if (demand <= 0f)
            {
                return 0f;
            }

            // Delivered force = demand x effectiveness x the bus's
            // delivered power ratio (this block's phase-4 value — the
            // single source of truth).
            return demand * effectiveness * thruster.DeliveredRatio;
        }

        private float GroupDemandAlong(Vector3 axis)
        {
            // The group demand's component along a body axis: for +X, the
            // demand fires thrusters pushing +X when the demand's X is
            // positive; for -X when it is negative. axis here is the
            // thruster's force axis (-forward).
            if (axis.X != 0f)
            {
                return axis.X > 0f ? _groupDemand.X : -_groupDemand.X;
            }
            if (axis.Y != 0f)
            {
                return axis.Y > 0f ? _groupDemand.Y : -_groupDemand.Y;
            }
            if (axis.Z > 0f)
            {
                return _groupDemand.Z;
            }
            return -_groupDemand.Z;
        }

        private static Vector3 ClampComponents(Vector3 value)
        {
            return new Vector3(
                ClampComponent(value.X),
                ClampComponent(value.Y),
                ClampComponent(value.Z));
        }

        /// <summary>Recomputes the grid's per-direction thrust capacity and the representative
        /// <see cref="SlowdownFactor"/> from the registered thrusters (the game's
        /// <c>RecomputeTypeThrustParameters</c>: MaxPositiveThrust / MaxNegativeThrust summed from
        /// effective thrust over used, powered thrusters). Called by
        /// <see cref="BaseControlSystem"/> each phase 5 before it computes the base demand, so
        /// <c>ComputeBaseThrust</c> sees the same capacities the aggregation consumes (no
        /// divergence).</summary>
        internal void RecalculateCapacities()
        {
            Vector3 pos = Vector3.Zero;
            Vector3 neg = Vector3.Zero;
            float factor = 1f;
            bool haveFactor = false;
            for (int i = 0; i < _thrusters.Count; i++)
            {
                SimThrust thruster = _thrusters[i];
                // Atmosphere scales effective thrust in the capacity too,
                // so the base-control path's capacities match what
                // aggregation delivers under the current atmosphere.
                float effective = thruster.Effectiveness * _grid.World.AtmosphereEffectiveness;
                if (thruster.Closed || !thruster.IsFunctional || effective <= 0f)
                {
                    continue;
                }
                // The grid is pushed OPPOSITE the thruster's forward; the
                // force axis is -forward (MyThrust.cs:188). Accumulate the
                // effective capacity by force direction.
                Vector3 forceAxis = -(Vector3)new Vector3(thruster.GridThrustDirection);
                float mag = thruster.MaxThrust * effective;
                AddCapacity(ref pos, ref neg, forceAxis, mag);
                if (!haveFactor)
                {
                    factor = thruster.SlowdownFactor;
                    haveFactor = true;
                }
            }
            _maxPositiveThrust = pos;
            _maxNegativeThrust = neg;
            _slowdownFactor = haveFactor ? factor : 1f;
        }

        private static void AddCapacity(ref Vector3 pos, ref Vector3 neg, Vector3 forceAxis, float mag)
        {
            if (forceAxis.X > 0f)
            {
                pos.X += mag;
            }
            else if (forceAxis.X < 0f)
            {
                neg.X += mag;
            }
            if (forceAxis.Y > 0f)
            {
                pos.Y += mag;
            }
            else if (forceAxis.Y < 0f)
            {
                neg.Y += mag;
            }
            if (forceAxis.Z > 0f)
            {
                pos.Z += mag;
            }
            else if (forceAxis.Z < 0f)
            {
                neg.Z += mag;
            }
        }

        private static float ClampComponent(float value)
        {
            return value < -1f ? -1f : value > 1f ? 1f : value;
        }
    }
}
