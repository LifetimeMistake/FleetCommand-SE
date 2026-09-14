using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The grid's controller-driven base thrust demand (.5): the
    /// MoveIndicator (manual control) + inertia dampening + anti-gravity
    /// that occupied ship controllers feed into the thrust base path — the
    /// game's <c>MyEntityThrustComponent.ComputeBaseThrust</c>
    /// (Decompiled/Sandbox.Game/Sandbox/Game/GameSystems/MyEntityThrustComponent.cs,
    /// lines 1543–1656) driven by the controller route
    /// (<c>MyShipController.MoveAndRotate</c>, :1780–1820 —
    /// <c>EntityThrustComponent.ControlThrust += Vector3.Transform(MoveIndicator,
    /// blockOrientation)</c>, <c>Enabled = ControlThrusters</c>).
    /// <para>
    /// It reproduces <c>ComputeBaseThrust</c> exactly: the dampening term
    /// cancels the grid-frame velocity error plus half of gravity (the
    /// <c>+ g/2</c> inside <c>vector2</c>), weighted per axis by the
    /// manual-control share <c>zero = 1 − |controlThrust|</c> scaled by
    /// <c>zero3/zero2</c> (the fraction of that axis's total thrust this
    /// fuel group can deliver in the needed direction — 1 for the single
    /// group), with the proportional constant <c>/0.5f</c> (a ~0.5 s time
    /// constant) scaled by <see cref="SimRigidBody.Mass"/>, and the
    /// whole demand clamped to <c>±Max · SlowdownFactor</c>. The resulting
    /// per-direction base force is normalized back to the grid-body group
    /// demand the <see cref="ForceAggregator"/> consumes, so effectiveness,
    /// power, and direction all apply exactly as for any group demand — no
    /// divergence between the base demand and phase 4.
    /// </para>
    /// <para>
    /// Gating: only an under-control (<see cref="SimShipController.IsUnderControl"/>,
    /// working) controller contributes; its <see cref="SimShipController.MoveIndicator"/>
    /// (transformed to the grid frame) becomes <c>ControlThrust</c>, its
    /// <see cref="SimShipController.ControlThrusters"/> gates the whole base
    /// computation (<c>EntityThrustComponent.Enabled</c>), and its
    /// <see cref="SimShipController.DampenersOverride"/> turns the
    /// dampening term on or off. With no under-control controller there is
    /// no base thrust — a free drifting ship stays drifting.
    /// </summary>
    public sealed class BaseControlSystem
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimShipController> _controllers = new List<SimShipController>();

        private Vector3 _lastControlThrust;   // grid-frame manual control throttle (per axis, [-1, 1])
        private Vector3 _lastSlowdownThrust;  // grid-frame dampening force (N)
        private Vector3 _lastBaseForce;       // grid-frame delivered base force (N), the game's CurrentThrust
        private bool _baseActive;

        /// <summary>The rigid grid (ship) this base-control system serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>
        /// The grid-frame manual control throttle from the last
        /// <see cref="ComputeBaseDemand"/> (the summed, transformed
        /// MoveIndicator of under-control controllers). Observation surface
        /// for conformance.
        /// </summary>
        public Vector3 LastControlThrust => _lastControlThrust;

        /// <summary>
        /// The grid-frame dampening force (N) from the last
        /// <see cref="ComputeBaseDemand"/> (the game's <c>slowdownThrust</c>).
        /// </summary>
        public Vector3 LastSlowdownThrust => _lastSlowdownThrust;

        /// <summary>
        /// The grid-frame delivered base force (N) from the last
        /// <see cref="ComputeBaseDemand"/> (the game's <c>CurrentThrust</c>),
        /// the demand the aggregator then turns into per-thruster delivery.
        /// </summary>
        public Vector3 LastBaseForce => _lastBaseForce;

        /// <summary>
        /// True when the last <see cref="ComputeBaseDemand"/> produced a
        /// base demand (an under-control controller with ControlThrusters);
        /// false when no controller was driving the base path.
        /// </summary>
        public bool BaseActive => _baseActive;

        internal BaseControlSystem(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers an under-control-capable ship controller (spawn).</summary>
        internal void RegisterController(SimShipController controller)
        {
            if (controller == null)
            {
                throw new ArgumentNullException(nameof(controller));
            }
            _controllers.Add(controller);
        }

        /// <summary>
        /// Tick phase 5 (before <see cref="ForceAggregator.Aggregate"/>):
        /// recompute the grid's directional thrust capacity, then reproduce
        /// <c>ComputeBaseThrust</c> from the under-control controllers'
        /// MoveIndicator, the body velocity, gravity, mass, and capacities,
        /// and set the aggregator's group demand from the normalized base
        /// force. With no under-control controller the base path is idle:
        /// the group demand is left as set (a free drifting ship stays
        /// drifting).
        /// </summary>
        public void ComputeBaseDemand()
        {
            _lastControlThrust = Vector3.Zero;
            _lastSlowdownThrust = Vector3.Zero;
            _lastBaseForce = Vector3.Zero;
            _baseActive = false;

            // The game's per-direction capacities (RecomputeTypeThrustParameters)
            // must be current before ComputeBaseThrust reads them.
            _grid.ForceAggregator.RecalculateCapacities();

            // The controller route: EntityThrustComponent.Enabled =
            // ControlThrusters (MyShipController.cs:1812); only under-control,
            // working controllers contribute their MoveIndicator.
            Vector3 controlThrust = Vector3.Zero;
            bool anyUnderControl = false;
            bool dampenersWanted = false;
            bool controlEnabled = false;
            for (int i = 0; i < _controllers.Count; i++)
            {
                SimShipController c = _controllers[i];
                if (c.Closed || !c.IsWorking || !c.IsUnderControl)
                {
                    continue;
                }
                anyUnderControl = true;
                if (!c.ControlThrusters)
                {
                    continue;
                }
                controlEnabled = true;
                if (c.DampenersOverride)
                {
                    dampenersWanted = true;
                }
                // Vector3.Transform(MoveIndicator, blockOrientation): the
                // block's grid-local orientation maps the pilot's control
                // input into the grid (body) frame.
                Matrix blockToGrid = Matrix.CreateFromQuaternion(c.Template.Orientation);
                controlThrust += Vector3.TransformNormal(c.MoveIndicator, blockToGrid);
            }

            if (!anyUnderControl || !controlEnabled)
            {
                return; // no base thrust (EntityThrustComponent.Enabled=false)
            }

            Vector3 maxPos = _grid.ForceAggregator.MaxPositiveThrust;
            Vector3 maxNeg = _grid.ForceAggregator.MaxNegativeThrust;
            float slowdownFactor = _grid.ForceAggregator.SlowdownFactor;

            SimRigidBody body = _grid.Body;
            if (body == null || body.IsDisposed || !body.IsDynamic)
            {
                return;
            }

            // ComputeBaseThrust (MyEntityThrustComponent.cs:1543-1656):
            // matrix = Entity.PositionComp.WorldMatrixNormalizedInv, the
            // world->grid rotation.
            Matrix worldToGrid = Matrix.Transpose(((Matrix)_grid.WorldMatrix).GetOrientation());
            Vector3 gravity = (Vector3)_grid.World.Gravity.GetNaturalGravity(body.PositionWorld) * 0.5f;
            Vector3 vector2 = Vector3.TransformNormal(
                (dampenersWanted ? (Vector3)body.LinearVelocity : Vector3.Zero) + gravity, worldToGrid);
            if (vector2.LengthSquared() < 1.0000001E-06f)
            {
                vector2 = Vector3.Zero;
            }
            Vector3 vector3 = Vector3.Zero; // no relative dampening
            Vector3 vector4 = vector3 - vector2;
            Vector3 slowdownThrust = Vector3.Zero;
            if (!Vector3.IsZero(vector4) || !Vector3.IsZero(controlThrust))
            {
                if (dampenersWanted) // DampenersEnabled
                {
                    Vector3 value = Vector3.Abs(Vector3.Clamp(controlThrust, -Vector3.One, Vector3.One));
                    Vector3 zero = Vector3.One - value; // per-axis manual-control share
                    // zero2 = per-axis total max thrust in the dampening
                    // direction; zero3 = this fuel group's max (= zero2 for
                    // the framework's single fuel group). The share
                    // zero3/zero2 is unity per axis where capacity exists,
                    // and 1 where it is invalid (0/0) — matching the game's
                    // !IsValid() -> 1.
                    Vector3 zero2 = zero2Direction(vector2, vector3, maxNeg, maxPos);
                    zero *= DirectionShare(zero2, zero2);
                    slowdownThrust = vector4 / 0.5f * body.Mass * zero;
                }
            }
            Vector3 vector6 = Vector3.Clamp(controlThrust, Vector3.Zero, Vector3.One) * maxPos
                              + Vector3.Clamp(controlThrust, -Vector3.One, Vector3.Zero) * maxNeg;
            vector6 = Vector3.Clamp(vector6, -maxNeg, maxPos);
            vector6 = Vector3.Clamp(vector6 + slowdownThrust, -maxNeg * slowdownFactor, maxPos * slowdownFactor);

            _lastControlThrust = controlThrust;
            _lastSlowdownThrust = slowdownThrust;
            _lastBaseForce = vector6;
            _baseActive = true;

            // Normalize the per-direction base force back to a grid-body
            // group demand the aggregator feeds (its delivered force equals
            // this force demand, then effectiveness + power + direction
            // apply as for any group demand).
            _grid.ForceAggregator.SetGroupDemand(Normalize(vector6, maxPos, maxNeg));
        }

        private static Vector3 zero2Direction(Vector3 vector2, Vector3 vector3, Vector3 maxNeg, Vector3 maxPos)
        {
            Vector3 r = Vector3.Zero;
            r.X = vector2.X > vector3.X ? maxNeg.X : (vector2.X < vector3.X ? maxPos.X : 0f);
            r.Y = vector2.Y > vector3.Y ? maxNeg.Y : (vector2.Y < vector3.Y ? maxPos.Y : 0f);
            r.Z = vector2.Z > vector3.Z ? maxNeg.Z : (vector2.Z < vector3.Z ? maxPos.Z : 0f);
            return r;
        }

        private static Vector3 DirectionShare(Vector3 a, Vector3 b)
        {
            Vector3 r = Vector3.Zero;
            r.X = SafeShare(a.X, b.X);
            r.Y = SafeShare(a.Y, b.Y);
            r.Z = SafeShare(a.Z, b.Z);
            return r;
        }

        private static float SafeShare(float numerator, float denominator)
        {
            if (denominator == 0f || float.IsNaN(numerator / denominator))
            {
                return 1f; // the game's !IsValid() -> 1
            }
            return numerator / denominator;
        }

        private static Vector3 Normalize(Vector3 force, Vector3 maxPos, Vector3 maxNeg)
        {
            return new Vector3(
                NormalizeAxis(force.X, force.X >= 0f ? maxPos.X : maxNeg.X),
                NormalizeAxis(force.Y, force.Y >= 0f ? maxPos.Y : maxNeg.Y),
                NormalizeAxis(force.Z, force.Z >= 0f ? maxPos.Z : maxNeg.Z));
        }

        private static float NormalizeAxis(float force, float max)
        {
            if (force == 0f)
            {
                return 0f;
            }
            if (max == 0f)
            {
                return 0f;
            }
            return force / max;
        }
    }
}
