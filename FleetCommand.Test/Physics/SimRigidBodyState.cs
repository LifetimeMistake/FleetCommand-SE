using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// One rigid body's publicly visible instantaneous state: world-visible position,
    /// orientation, and world-frame velocities (recomputed as <c>origin + local</c>).
    /// </summary>
    public struct SimRigidBodyState
    {
        /// <summary>The owning body's stable numeric id.</summary>
        public long BodyId { get; internal set; }

        /// <summary>World-visible position of the grid origin (computed from origin +
        /// local).</summary>
        public Vector3D PositionWorld { get; set; }

        /// <summary>World orientation of the grid origin.</summary>
        public QuaternionD Orientation { get; set; }

        /// <summary>Linear velocity in the world frame (m/s).</summary>
        public Vector3D LinearVelocity { get; set; }

        /// <summary>Angular velocity in the world frame (rad/s).</summary>
        public Vector3 AngularVelocity { get; set; }

        internal SimRigidBodyState(
            long bodyId,
            Vector3D positionWorld,
            QuaternionD orientation,
            Vector3D linearVelocity,
            Vector3 angularVelocity)
        {
            BodyId = bodyId;
            PositionWorld = positionWorld;
            Orientation = orientation;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
        }

        /// <summary>Creates a state at <paramref name="positionWorld"/>, identity orientation, at
        /// rest.</summary>
        public static SimRigidBodyState At(Vector3D positionWorld)
        {
            return new SimRigidBodyState(0, positionWorld, QuaternionD.Identity, Vector3D.Zero, Vector3.Zero);
        }

        /// <summary>Creates a state at <paramref name="positionWorld"/> with the given orientation,
        /// at rest.</summary>
        public static SimRigidBodyState At(Vector3D positionWorld, QuaternionD orientation)
        {
            return new SimRigidBodyState(0, positionWorld, orientation, Vector3D.Zero, Vector3.Zero);
        }

        /// <summary>Returns a copy with the given world linear velocity.</summary>
        public SimRigidBodyState WithLinearVelocity(Vector3D linearVelocity)
        {
            return new SimRigidBodyState(BodyId, PositionWorld, Orientation, linearVelocity, AngularVelocity);
        }

        /// <summary>Returns a copy with the given world angular velocity.</summary>
        public SimRigidBodyState WithAngularVelocity(Vector3 angularVelocity)
        {
            return new SimRigidBodyState(BodyId, PositionWorld, Orientation, LinearVelocity, angularVelocity);
        }
    }
}
