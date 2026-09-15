using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Templates;
using Havok;
using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// One rigid body backing one dynamic grid (). The body's
    /// <see cref="HkRigidBody"/> is created mirroring
    /// <c>MyGridPhysics.CreateBody</c> (MyGridPhysics.cs:2426-2444): a
    /// compound collision shape and combined mass properties are set on an
    /// <see cref="HkRigidBodyCinfo"/>, then a fresh <see cref="HkRigidBody"/>
    /// is constructed with the profile's velocity caps and damping.
    /// Positions are held as float transforms relative to the fixed scenario
    /// origin so bodies stay numerically close to it; world-visible
    /// positions are computed as <c>origin + local</c> (). Destruction is
    /// disabled (Havok break-off parts are never enabled).
    /// </summary>
    public sealed class SimRigidBody : IDisposable
    {
        /// <summary>Stable numeric id, unique per world, in spawn order.</summary>
        public long Id { get; }

        /// <summary>True for a dynamic (simulated) body; false for a fixed/static body.</summary>
        public bool IsDynamic { get; }

        /// <summary>The scenario origin this body's coordinates are relative to.</summary>
        public Vector3D ScenarioOrigin { get; }

        /// <summary>The rigid body's mass in kilograms (0 for static bodies after Havok fix-up).</summary>
        public float Mass => _rigidBody == null ? 0f : _rigidBody.Mass;

        /// <summary>True once <see cref="Dispose"/> has run (the wrapped native body may be gone).</summary>
        public bool IsDisposed => _disposed;

        /// <summary>Center of mass in grid-local meters (ground truth, ).</summary>
        public Vector3 CenterOfMassLocal => _rigidBody == null ? Vector3.Zero : _rigidBody.CenterOfMassLocal;

        /// <summary>World-visible position of the grid origin.</summary>
        public Vector3D PositionWorld => ScenarioOrigin + new Vector3D(_localPosition);

        /// <summary>World orientation of the grid origin.</summary>
        public QuaternionD Orientation => ToQuaternionD(_orientation);

        /// <summary>Linear velocity in the world frame (m/s).</summary>
        public Vector3D LinearVelocity => new Vector3D(_linearVelocity);

        /// <summary>Angular velocity in the world frame (rad/s).</summary>
        public Vector3 AngularVelocity => _angularVelocity;

        /// <summary>
        /// The configured grid angular-rate cap (rad/s): the larger of the
        /// template's <c>MaxAngularVelocity</c> and the profile's
        /// <c>MaxAngularVelocity</c> set on the Havok body at creation
        /// ( step 9). Havok already enforces this cap; the gyro
        /// override setter clamps commanded rates to it (the game's
        /// <c>MaxAngularRadiansPerSecond</c>), and gyro response never
        /// retunes it.
        /// </summary>
        public float MaxAngularVelocity => _maxAngularVelocity;

        /// <summary>
        /// The body's inverse inertia diagonal (1/Ixx, 1/Iyy, 1/Izz) in
        /// the grid-local frame (axes aligned with the grid's principal
        /// axes — the frame the gyro override loop sizes its torque in,
        /// matching the game's <c>InverseInertiaTensor</c> diagonal read,
        /// MyGridGyroSystem.cs:293).
        /// </summary>
        public Vector3 InverseInertiaDiagonal => _inverseInertiaDiagonal;

        /// <summary>World transform of the grid origin, derived from the body pose.</summary>
        public MatrixD WorldMatrix
        {
            get
            {
                MatrixD matrix = MatrixD.CreateFromQuaternion(Orientation);
                matrix.Translation = PositionWorld;
                return matrix;
            }
        }

        /// <summary>The wrapped Havok rigid body (internal; the public surface is the state and pose).</summary>
        internal HkRigidBody Body => _rigidBody;

        private readonly Vector3D _origin;
        private HkRigidBody _rigidBody;

        // One owned reference on the compound shape. Following the game's
        // shape lifetime (MyGridShape keeps m_root alive and disposes it via
        // Base.RemoveReference after the rigid body is created).
        private HkShape _shape;

        // The static compound shape does NOT add references to its child
        // instances (verified against HavokWrapper: AddInstance leaves the
        // child's reference count unchanged), so each block's box shape is
        // kept alive for the body's lifetime and released in Dispose —
        // mirroring the game keeping block shapes alive while the grid shape
        // is alive (MyGridShape owns the block shapes and releases them on
        // shape rebuild).
        private readonly List<HkShape> _childShapes = new List<HkShape>();

        // Cached state, refreshed from Havok during phase 8 (SynchronizeState).
        private Vector3 _localPosition;
        private Quaternion _orientation;
        private Vector3 _linearVelocity;
        private Vector3 _angularVelocity;

        private float _maxAngularVelocity;
        private Vector3 _inverseInertiaDiagonal;

        private bool _disposed;

        private SimRigidBody(long id, Vector3D scenarioOrigin, bool isDynamic)
        {
            Id = id;
            _origin = scenarioOrigin;
            IsDynamic = isDynamic;
        }

        /// <summary>
        /// Builds the compound shape and mass properties from
        /// <paramref name="template"/> and constructs the rigid body from
        /// <paramref name="initialState"/>. Called once per spawn; each
        /// spawn gets fresh Havok objects ().
        /// </summary>
        public static SimRigidBody Create(
            long id,
            ShipTemplate template,
            Vector3D scenarioOrigin,
            in SimRigidBodyState initialState,
            SePhysicsProfile profile)
        {
            if (template == null)
            {
                throw new ArgumentNullException(nameof(template));
            }
            if (profile == null)
            {
                profile = SePhysicsProfile.Default;
            }

            bool isDynamic = !template.IsStatic;
            var body = new SimRigidBody(id, scenarioOrigin, isDynamic);

            // Per-grid velocity caps from the template take precedence; the
            // profile supplies the default when the template leaves them unset
            // (: caps are profile-configurable per grid).
            float maxLinear = template.MaxLinearVelocity > 0f ? template.MaxLinearVelocity : profile.MaxLinearVelocity;
            float maxAngular = template.MaxAngularVelocity > 0f ? template.MaxAngularVelocity : profile.MaxAngularVelocity;

            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                body._shape = body.BuildCompoundShape(template);

                MassProperties mass = MassProperties.FromShipTemplate(template);

                var info = new HkRigidBodyCinfo();
                info.Shape = body._shape;
                info.Position = (Vector3)(initialState.PositionWorld - scenarioOrigin);
                info.Rotation = ToQuaternion(initialState.Orientation);
                info.LinearVelocity = (Vector3)initialState.LinearVelocity;
                info.AngularVelocity = initialState.AngularVelocity;
                info.LinearDamping = profile.LinearDamping;
                info.AngularDamping = profile.AngularDamping;
                info.MaxLinearVelocity = maxLinear;
                info.MaxAngularVelocity = maxAngular;
                info.MotionType = isDynamic ? HkMotionType.Dynamic : HkMotionType.Fixed;
                info.QualityType = isDynamic ? HkCollidableQualityType.Moving : HkCollidableQualityType.Fixed;

                HkMassProperties mp = default(HkMassProperties);
                mp.Mass = mass.TotalMass;
                mp.CenterOfMass = mass.CenterOfMass;
                mp.InertiaTensor = mass.InertiaTensor;
                mp.Volume = 1f;
                info.SetMassProperties(mp);

                body._rigidBody = new HkRigidBody(info);

                // Collision layer: 15 is the game's default grid layer; 19 is
                // the no-collision-response layer (MyPhysicsBody.cs:1414-1418).
                body._rigidBody.Layer = profile.CollisionResponse ? 15 : 19;

                body._localPosition = (Vector3)(initialState.PositionWorld - scenarioOrigin);
                body._orientation = ToQuaternion(initialState.Orientation);
                body._linearVelocity = (Vector3)initialState.LinearVelocity;
                body._angularVelocity = initialState.AngularVelocity;

                // Retain the configured angular-rate cap (also set on the
                // Havok body above) and the grid-local inverse inertia
                // diagonal for the gyro override loop ( step 3).
                body._maxAngularVelocity = maxAngular;
                body._inverseInertiaDiagonal = new Vector3(
                    mass.InverseInertiaTensor.M11,
                    mass.InverseInertiaTensor.M22,
                    mass.InverseInertiaTensor.M33);
            }

            return body;
        }

        /// <summary>
        /// Builds the grid's compound collision shape: one axis-aligned box
        /// per block at the block's grid-local transform, baked into an
        /// <see cref="HkStaticCompoundShape"/> — the general-purpose
        /// container this Havok build offers for a compound of convex
        /// children (the game uses <c>HkGridShape</c> for ship grids and
        /// <c>HkSmartListShape</c>/<c>HkTransformShape</c> for welded
        /// children; <see cref="HkStaticCompoundShape"/>.AddInstance is this
        /// build's equivalent of "one child shape per block at its local
        /// transform"). Each child box is kept alive by <see cref="_childShapes"/>.
        /// </summary>
        private HkShape BuildCompoundShape(ShipTemplate template)
        {
            var compound = new HkStaticCompoundShape(HkReferencePolicy.TakeOwnership);
            foreach (TemplateBlock block in template.Blocks)
            {
                var box = new HkBoxShape(block.HalfExtents);
                compound.AddInstance(box, Matrix.CreateTranslation(block.CenterLocal));
                // AddInstance takes no child reference (verified against
                // HavokWrapper); retain each box for the body's lifetime and
                // release it in Dispose.
                _childShapes.Add(box.Base);
            }
            compound.Bake();
            // The compound's own reference is the wrapper's owned reference;
            // released exactly once in Dispose.
            return compound.Base;
        }

        /// <summary>
        /// Writes the body back from a world state (scenario setup and
        /// ground-truth injection, ).
        /// </summary>
        public void SetState(in SimRigidBodyState state)
        {
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(SimRigidBody));
            }

            Vector3 local = (Vector3)(state.PositionWorld - ScenarioOrigin);
            Quaternion orientation = ToQuaternion(state.Orientation);

            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.Position = local;
                _rigidBody.Rotation = orientation;
                _rigidBody.LinearVelocity = (Vector3)state.LinearVelocity;
                _rigidBody.AngularVelocity = state.AngularVelocity;
                if (!_rigidBody.IsActive)
                {
                    _rigidBody.IsActive = true;
                }
            }

            _localPosition = local;
            _orientation = orientation;
            _linearVelocity = (Vector3)state.LinearVelocity;
            _angularVelocity = state.AngularVelocity;
        }

        /// <summary>Reads the body's current world-visible state.</summary>
        public SimRigidBodyState GetState()
        {
            return new SimRigidBodyState(
                Id,
                PositionWorld,
                Orientation,
                LinearVelocity,
                AngularVelocity);
        }

        /// <summary>Applies a world-frame linear impulse (N·s) at the body's center of mass.</summary>
        public void ApplyLinearImpulse(Vector3 impulse)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.ApplyLinearImpulse(impulse);
            }
        }

        /// <summary>Applies a world-frame angular impulse (N·m·s) about the body's center of mass.</summary>
        public void ApplyAngularImpulse(Vector3 impulse)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.ApplyAngularImpulse(impulse);
            }
        }

        /// <summary>
        /// Applies a world-frame impulse at <paramref name="worldPoint"/>
        /// (off the center of mass), producing both a linear velocity
        /// change and an angular velocity change <c>I⁻¹ (r × impulse)</c>
        /// — the game's <c>rigidBody.ApplyPointImpulse(impulse, position)</c>
        /// (MyGridPhysics.cs:1339, MyPhysicsBody.cs:1246). The point is
        /// translated to origin-relative (body-local) space, matching how
        /// the body stores its position.
        /// </summary>
        public void ApplyPointImpulse(Vector3 impulse, Vector3D worldPoint)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.ApplyPointImpulse(impulse, (Vector3)(worldPoint - ScenarioOrigin));
            }
        }

        /// <summary>Inserts the rigid body into the world (spawn).</summary>
        internal void AddToWorld(HkWorld world)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                world.AddRigidBody(_rigidBody);
            }
        }

        /// <summary>Removes the rigid body from the world (teardown).</summary>
        internal void RemoveFromWorld(HkWorld world)
        {
            if (_rigidBody == null || _disposed)
            {
                return;
            }
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                if (_rigidBody.InWorld)
                {
                    world.RemoveRigidBody(_rigidBody);
                }
            }
        }

        /// <summary>
        /// Reads position, orientation, and velocities back from Havok after
        /// a physics step. Called by SimWorld in phase 8 (SynchronizeState)
        /// so observers always see the post-step state.
        /// </summary>
        internal void SynchronizeFromHavok()
        {
            _localPosition = _rigidBody.Position;
            _orientation = _rigidBody.Rotation;
            _linearVelocity = _rigidBody.LinearVelocity;
            _angularVelocity = _rigidBody.AngularVelocity;
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                if (_rigidBody != null && !_rigidBody.IsDisposed)
                {
                    _rigidBody.Dispose();
                }
                if (_shape.IsValid)
                {
                    _shape.Base.RemoveReference();
                }
                for (int i = 0; i < _childShapes.Count; i++)
                {
                    if (_childShapes[i].IsValid)
                    {
                        _childShapes[i].Base.RemoveReference();
                    }
                }
                _childShapes.Clear();
            }

            _rigidBody = null;
            _shape = default(HkShape);
            _disposed = true;
        }

        private static Quaternion ToQuaternion(QuaternionD value)
        {
            return new Quaternion((float)value.X, (float)value.Y, (float)value.Z, (float)value.W);
        }

        private static QuaternionD ToQuaternionD(Quaternion value)
        {
            return new QuaternionD(value.X, value.Y, value.Z, value.W);
        }
    }
}
