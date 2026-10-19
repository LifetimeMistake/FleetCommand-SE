using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Templates;
using Havok;
using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// One rigid body backing one dynamic grid. Its <see cref="HkRigidBody"/> is created
    /// mirroring MyGridPhysics.CreateBody (MyGridPhysics.cs:2426-2444): a compound
    /// collision shape and combined mass properties on an <see cref="HkRigidBodyCinfo"/>,
    /// with the profile's velocity caps and damping. Positions are float transforms
    /// relative to a fixed scenario origin; destruction (Havok break-off) is disabled.
    /// </summary>
    public sealed class SimRigidBody : IDisposable
    {
        /// <summary>The grid-wide inertia-dampener enable state — the sim's analog of the game's
        /// <c>MyEntityThrustComponent.DampenersEnabled</c>, a single shared per-grid flag that every
        /// ship controller reads and writes (MyShipController.cs:770-786, MyEntityThrustComponent.cs:299).
        /// It is shared per grid because the game's dampeners are grid-global, not per-controller:
        /// toggling any controller's <c>DampenersOverride</c> switches the whole grid's dampeners
        /// (<c>CubeGrid.EnableDampingInternal(value)</c>, :784). Stored on the shared grid body,
        /// the per-grid object every controller reaches via <c>Grid.Body</c>. Defaults on, matching
        /// the game (MyEntityThrustComponent.cs:463).</summary>
        public bool DampenersEnabled { get; set; } = true;

        /// <summary>Stable numeric id, unique per world, in spawn order.</summary>
        public long Id { get; }

        /// <summary>True for a dynamic (simulated) body; false for a fixed/static body.</summary>
        public bool IsDynamic { get; }

        /// <summary>The scenario origin this body's coordinates are relative to.</summary>
        public Vector3D ScenarioOrigin { get; }

        /// <summary>The rigid body's mass in kilograms (0 for static bodies after Havok
        /// fix-up).</summary>
        public float Mass => _rigidBody == null ? 0f : _rigidBody.Mass;

        /// <summary>True once <see cref="Dispose"/> has run (the wrapped native body may be
        /// gone).</summary>
        public bool IsDisposed => _disposed;

        /// <summary>Center of mass in grid-local meters (ground truth).</summary>
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
        /// Configured grid angular-rate cap (rad/s), set on the Havok body at creation. Havok
        /// enforces it; the gyro override setter clamps commanded rates to it (the game's
        /// <c>MaxAngularRadiansPerSecond</c>).
        /// </summary>
        public float MaxAngularVelocity => _maxAngularVelocity;

        /// <summary>
        /// Configured grid linear-velocity cap (m/s). The sim applies it per frame after physics,
        /// mirroring the game's per-frame grid max-speed clamp (MyGridPhysics / sector env def).
        /// </summary>
        public float MaxLinearVelocity => _maxLinearVelocity;

        /// <summary>Configured angular damping, set on the Havok body at creation; the game
        /// applies MyPerGameSettings.DefaultAngularDamping (MyGridPhysics.cs:436).</summary>
        public float AngularDamping => _angularDamping;

        /// <summary>Actual native inverse-inertia diagonal in the grid-local frame,
        /// matching MyGridGyroSystem's read. Havok may diagonalize the supplied tensor,
        /// so the analytical full-matrix inverse is not an interchangeable value.</summary>
        public Vector3 InverseInertiaDiagonal
        {
            get
            {
                if (_rigidBody == null || _disposed) return Vector3.Zero;
                Matrix inverse = _rigidBody.InverseInertiaTensor;
                return new Vector3(inverse.M11, inverse.M22, inverse.M33);
            }
        }

        /// <summary>World transform of the grid origin, derived from the body pose.</summary>
        public MatrixD WorldMatrix
        {
            get
            {
                MatrixD matrix = _bodyMatrix;
                matrix.Translation = PositionWorld;
                return matrix;
            }
        }

        /// <summary>The wrapped Havok rigid body (internal; the public surface is the state and
        /// pose).</summary>
        internal HkRigidBody Body => _rigidBody;

        private readonly Vector3D _origin;
        private HkRigidBody _rigidBody;

        // One owned reference on the compound shape; the static compound adds no
        // references to its children (HavokWrapper), so each block's box shape is
        // kept alive here and released in Dispose (mirrors MyGridShape).
        private HkShape _shape;

        private readonly List<HkShape> _childShapes = new List<HkShape>();

        // Cached state, refreshed from Havok during phase 8 (SynchronizeState).
        private Vector3 _localPosition;
        private Quaternion _orientation;
        private MatrixD _bodyMatrix;
        private Vector3 _linearVelocity;
        private Vector3 _angularVelocity;

        private float _maxAngularVelocity;
        private float _maxLinearVelocity;
        private float _angularDamping;

        private bool _disposed;

        private SimRigidBody(long id, Vector3D scenarioOrigin, bool isDynamic)
        {
            Id = id;
            _origin = scenarioOrigin;
            IsDynamic = isDynamic;
        }

        /// <summary>Builds the compound shape and mass properties from <paramref name="template"/>
        /// and constructs the body from <paramref name="initialState"/>.</summary>
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

            // Per-grid velocity caps take precedence; the profile supplies the default when
            // unset. The angular default is the game's per-grid-size cap: large grids get
            // LargeShipMaxAngularVelocityLimit, small grids SmallShipMaxAngularVelocityLimit
            // (MyGridPhysics.GetShipMaxAngularVelocity, MyGridPhysics.cs:382-394). Cube size
            // is the grid's cell size: large 2.5 m, small 0.5 m.
            float maxLinear = template.MaxLinearVelocity > 0f ? template.MaxLinearVelocity : profile.MaxLinearVelocity;
            float maxAngular = template.MaxAngularVelocity > 0f
                ? template.MaxAngularVelocity
                : (IsLargeGrid(template.GridSize) ? profile.MaxAngularVelocityLarge : profile.MaxAngularVelocitySmall);

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
                // MyPhysicsBody defaults to Low; MyGridPhysics.CreateBody disables
                // deactivation only for dynamic large grids.
                info.SolverDeactivation = isDynamic && IsLargeGrid(template.GridSize)
                    ? HkSolverDeactivation.Off : HkSolverDeactivation.Low;
                info.QualityType = isDynamic ? HkCollidableQualityType.Moving : HkCollidableQualityType.Fixed;

                HkMassProperties mp = default(HkMassProperties);
                mp.Mass = mass.TotalMass;
                mp.CenterOfMass = mass.CenterOfMass;
                mp.InertiaTensor = mass.InertiaTensor;
                mp.Volume = 1f;
                info.SetMassProperties(mp);

                body._rigidBody = new HkRigidBody(info);

                // Collision layer: 15 is the default grid layer, 19 the no-collision layer
                // (MyPhysicsBody.cs:1414-1418).
                body._rigidBody.Layer = profile.CollisionResponse ? 15 : 19;

                body._localPosition = (Vector3)(initialState.PositionWorld - scenarioOrigin);
                body._orientation = ToQuaternion(initialState.Orientation);
                body._bodyMatrix = body._rigidBody.GetRigidBodyMatrix();
                body._linearVelocity = (Vector3)initialState.LinearVelocity;
                body._angularVelocity = initialState.AngularVelocity;

                // Retain the configured velocity caps and angular damping. Gyros read
                // the native inverse inertia tensor directly.
                body._maxAngularVelocity = maxAngular;
                body._maxLinearVelocity = maxLinear;
                body._angularDamping = profile.AngularDamping;
            }

            return body;
        }

        /// <summary>
        /// Builds the grid's compound collision shape: one axis-aligned box per block at its
        /// grid-local transform in an <see cref="HkStaticCompoundShape"/> (this build's
        /// equivalent of "one child shape per block"; the game uses HkGridShape for ships).
        /// </summary>
        private HkShape BuildCompoundShape(ShipTemplate template)
        {
            var compound = new HkStaticCompoundShape(HkReferencePolicy.TakeOwnership);
            foreach (TemplateBlock block in template.Blocks)
            {
                var box = new HkBoxShape(block.HalfExtents);
                compound.AddInstance(box, Matrix.CreateTranslation(block.CenterLocal));
                // AddInstance takes no child reference (HavokWrapper); retain it for release in
                // Dispose.
                _childShapes.Add(box.Base);
            }
            compound.Bake();
            // The compound's own reference is released exactly once in Dispose.
            return compound.Base;
        }

        /// <summary>Writes the body back from a world state (scenario setup and ground-truth
        /// injection).</summary>
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
                _bodyMatrix = _rigidBody.GetRigidBodyMatrix();
            }

            _localPosition = local;
            _orientation = orientation;
            _linearVelocity = (Vector3)state.LinearVelocity;
            _angularVelocity = state.AngularVelocity;
            StateChanged?.Invoke();
        }

        /// <summary>Scenario pose/velocity replacement; attached auxiliary bodies follow
        /// teleports without being overwritten during ordinary physics integration.</summary>
        internal event Action StateChanged;

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

        /// <summary>
        /// Replaces the body's mass properties from newly combined <paramref name="mass"/>
        /// (block added/removed or inventory changed) via the game's runtime path
        /// — <c>RigidBody.SetMassProperties</c> (MyPhysicsBody.cs:2867).
        /// </summary>
        public void UpdateMassProperties(MassProperties mass)
        {
            if (_disposed || _rigidBody == null)
            {
                return;
            }
            if (mass == null)
            {
                throw new ArgumentNullException(nameof(mass));
            }
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                var mp = default(HkMassProperties);
                mp.Mass = mass.TotalMass;
                mp.CenterOfMass = mass.CenterOfMass;
                mp.InertiaTensor = mass.InertiaTensor;
                mp.Volume = 1f;
                _rigidBody.SetMassProperties(ref mp);
            }
        }

        /// <summary>
        /// Clamps world linear velocity to <paramref name="maxSpeed"/>, mirroring the game's
        /// per-frame grid clamp (<c>MovementBlockMaxSpeed</c>, MyGridPhysics.cs:538-555).
        /// No-op at or below the cap.
        /// </summary>
        public void ClampLinearSpeed(float maxSpeed)
        {
            if (_disposed || _rigidBody == null || maxSpeed <= 0f)
            {
                return;
            }
            Vector3 v = _linearVelocity;
            float len = v.Length();
            if (len > maxSpeed && len > 0f)
            {
                Vector3 clamped = v * (maxSpeed / len);
                using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                {
                    _rigidBody.LinearVelocity = clamped;
                }
                _linearVelocity = clamped;
            }
        }

        /// <summary>Applies a world-frame linear impulse (N·s) at the body's center of
        /// mass.</summary>
        public void ApplyLinearImpulse(Vector3 impulse)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.ApplyLinearImpulse(impulse);
            }
        }

        /// <summary>Game thrust supplies a COM position to AddForceTorqueBody, selecting
        /// Havok ApplyForce rather than its positionless impulse path. activeOnly means
        /// ongoing thrust cannot wake a body after it has fallen asleep.</summary>
        internal void ApplyThrustForce(Vector3 worldForce, float dtSeconds)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                if (!_rigidBody.IsActive) return;
                Vector3 com = Vector3.Transform(_rigidBody.CenterOfMassLocal, _rigidBody.GetRigidBodyMatrix());
                _rigidBody.ApplyForce(dtSeconds, worldForce, com);
            }
        }

        internal void Activate()
        {
            if (_disposed || !IsDynamic) return;
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                _rigidBody.Activate();
        }

        internal void SetAutomaticDeactivation(bool enabled)
        {
            if (_disposed || !IsDynamic) return;
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                _rigidBody.EnableDeactivation = enabled;
        }

        internal bool IsActive
        {
            get
            {
                if (_disposed) return false;
                using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                    return _rigidBody.IsActive;
            }
        }

        /// <summary>Applies a world-frame angular impulse (N·m·s) about the body's center of
        /// mass.</summary>
        public void ApplyAngularImpulse(Vector3 impulse)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _rigidBody.ApplyAngularImpulse(impulse);
            }
        }

        /// <summary>
        /// Applies a world-frame impulse at <paramref name="worldPoint"/> (off the COM), producing
        /// linear plus angular change <c>I⁻¹ (r × impulse)</c> — the game's
        /// RigidBody.ApplyPointImpulse (MyGridPhysics.cs:1339, MyPhysicsBody.cs:1246).
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

        /// <summary>Reads position, orientation, and velocities back from Havok after a physics
        /// step.</summary>
        internal void SynchronizeFromHavok()
        {
            // MyPhysicsBody.GetWorldMatrix reads this matrix directly. Going
            // through the native quaternion getter and rebuilding a matrix adds
            // a lossy conversion to every controller/thruster world transform.
            _bodyMatrix = _rigidBody.GetRigidBodyMatrix();
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

        private static bool IsLargeGrid(float gridSize)
        {
            // Large grid cells are 2.5 m, small grid cells are 0.5 m; the game selects the
            // angular cap on this cube size (MyGridPhysics.GetShipMaxAngularVelocity).
            return gridSize > 1f;
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
