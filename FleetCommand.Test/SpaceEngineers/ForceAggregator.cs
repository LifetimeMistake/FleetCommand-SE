using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The grid thrust aggregation step (,  steps 1–6): the
    /// boundary where per-thruster commands (override or group demand) become
    /// one resultant body-space force applied at the grid's center of mass.
    /// Owned by <see cref="SimCubeGrid"/>; the world drives it in three
    /// tick phases:
    /// <list type="number">
    /// <item><b>Phase 5</b> <see cref="Aggregate"/> — for each thruster,
    /// determine enabled / functional / powered / environmentally effective,
    /// compute <see cref="SimThrust.MaxEffectiveThrust"/> (template maximum
    /// x effectiveness profile), convert the override or the per-axis
    /// normalized group demand into delivered thrust (scaled by the
    /// block's <see cref="SimFunctionalBlock.DeliveredRatio"/> — the
    /// bus-computed value from phase 4), and sum delivered thrust by
    /// grid-body direction into one resultant force.</item>
    /// <item><b>Phase 6</b> <see cref="ApplyThrust"/> — apply ONE
    /// resultant body-space linear impulse at the grid COM (the source
    /// verified application point:
    /// MyThrusterBlockThrustComponent.cs:201 AddForce at the grid's
    /// CoM). Because the force is applied at the COM, fixed grid
    /// thrusters generate no torque from block position ().</item>
    /// <item><b>Phase 8</b> <see cref="UpdateThrusterTelemetry"/> — update
    /// each thruster's <see cref="SimThrust.CurrentThrust"/> after the
    /// physics interval.</item>
    /// </list>
    /// A thruster's <see cref="SimThrust.GridThrustDirection"/> is its
    /// grid-body FORWARD (exhaust) axis — the value the game returns —
    /// and the force axis is its negation: delivered is applied along
    /// <c>-forward</c>, matching the game's <c>ThrustForce =
    /// -ThrustForwardVector x ForceMagnitude</c> (MyThrust.cs:188). Both
    /// the override and group demand paths resolve the group demand
    /// against this force axis (game 1:1).
    /// The group demand is a normalized per-body-axis throttle in [-1, 1] —
    /// the "control input" side of the control boundary. It is scenario
    /// input (there is no control system yet); overridden thrusters answer
    /// their override instead of the group demand, matching the game
    /// (MyEntityThrustComponent.IsOverridden / IsUsed). Asymmetric and
    /// missing axes need no craft-type branches: thrusters carry their push
    /// direction as data, so a grid simply has no thruster where there is
    /// no thrust.
    /// </summary>
    public sealed class ForceAggregator
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimThrust> _thrusters = new List<SimThrust>();

        private Vector3 _groupDemand;
        private Vector3 _netBodyForce;

        /// <summary>The rigid grid (ship) this aggregator serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>
        /// The normalized group (control) demand, body-frame, components in
        /// [-1, 1]: a positive component fires the thrusters pushing along
        /// that axis at the given throttle; a negative component fires the
        /// opposite axis. Scenario-set (the control system arrives later).
        /// </summary>
        public Vector3 GroupDemand => _groupDemand;

        /// <summary>
        /// The resultant body-space thrust force (N) from the most recent
        /// <see cref="Aggregate"/>: the net of every thruster's delivered
        /// thrust summed by body direction. Zero when nothing fired. (Design
        ///  "delivered thrust per block and direction" observation.)
        /// </summary>
        public Vector3 LastNetForce => _netBodyForce;

        internal ForceAggregator(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>
        /// Sets the normalized group (control) demand that non-overridden
        /// thrusters answer, clamping each body-axis component to [-1, 1].
        /// </summary>
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

        /// <summary>
        /// Tick phase 5: gate each thruster, convert its override or group
        /// demand into delivered thrust (scaled by the bus's delivered
        /// ratio), sum by grid-body direction, and return the one
        /// resultant body-space force. Also stashes each thruster's
        /// delivered force for the phase-8 CurrentThrust write.
        /// </summary>
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

        /// <summary>
        /// Tick phase 6: apply the phase-5 resultant as ONE body-space
        /// linear impulse at the grid center of mass — transformed to the
        /// world frame and timed by the fixed step. Applied at the COM,
        /// so no torque arises from any thruster's block position.
        /// </summary>
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

        /// <summary>
        /// Tick phase 8: after the physics interval, publish each
        /// thruster's phase-5 delivered force as its
        /// <see cref="SimThrust.CurrentThrust"/>.
        /// </summary>
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
            float effectiveness = thruster.Effectiveness;
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
            // single source of truth, ).
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

        private static float ClampComponent(float value)
        {
            return value < -1f ? -1f : value > 1f ? 1f : value;
        }
    }
}
