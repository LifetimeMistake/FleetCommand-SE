using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// Physical spawn state for a ship (): initial world-visible
    /// position, orientation, and velocities. The scenario origin
    /// configured for the world is folded into body-local coordinates at
    /// spawn time by <see cref="Framework.FleetTestContext.Spawn"/>.
    /// </summary>
    public struct SpawnState
    {
        /// <summary>Initial world-visible position of the grid origin.</summary>
        public Vector3D Position { get; }

        /// <summary>Initial world orientation of the grid origin.</summary>
        public QuaternionD Orientation { get; }

        /// <summary>Initial world-frame linear velocity (m/s).</summary>
        public Vector3D LinearVelocity { get; }

        /// <summary>Initial world-frame angular velocity (rad/s).</summary>
        public Vector3 AngularVelocity { get; }

        private SpawnState(Vector3D position, QuaternionD orientation, Vector3D linearVelocity, Vector3 angularVelocity)
        {
            Position = position;
            Orientation = orientation;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
        }

        /// <summary>A spawn state at <paramref name="position"/> with the given orientation, at rest.</summary>
        public static SpawnState At(Vector3D position, MatrixD orientation)
        {
            QuaternionD quat;
            QuaternionD.CreateFromRotationMatrix(ref orientation, out quat);
            return new SpawnState(position, quat, Vector3D.Zero, Vector3.Zero);
        }

        /// <summary>A spawn state at <paramref name="position"/> with the given orientation, at rest.</summary>
        public static SpawnState At(Vector3D position, QuaternionD orientation)
        {
            return new SpawnState(position, orientation, Vector3D.Zero, Vector3.Zero);
        }

        /// <summary>A spawn state at <paramref name="position"/> with identity orientation, at rest.</summary>
        public static SpawnState At(Vector3D position)
        {
            return new SpawnState(position, QuaternionD.Identity, Vector3D.Zero, Vector3.Zero);
        }

        /// <summary>Returns a copy with the given world linear velocity.</summary>
        public SpawnState WithVelocity(Vector3D linearVelocity)
        {
            return new SpawnState(Position, Orientation, linearVelocity, AngularVelocity);
        }

        /// <summary>Returns a copy with the given world angular velocity.</summary>
        public SpawnState WithAngularVelocity(Vector3 angularVelocity)
        {
            return new SpawnState(Position, Orientation, LinearVelocity, angularVelocity);
        }

        /// <summary>Converts this spawn state to a rigid-body state (body id assigned by the caller).</summary>
        public SimRigidBodyState ToBodyState(long bodyId)
        {
            return new SimRigidBodyState(bodyId, Position, Orientation, LinearVelocity, AngularVelocity);
        }
    }
}
