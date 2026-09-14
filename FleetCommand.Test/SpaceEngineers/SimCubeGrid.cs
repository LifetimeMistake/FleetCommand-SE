using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The rigid grid's physical shell (, ): owns one
    /// <see cref="SimRigidBody"/> per dynamic grid and derives its
    /// world-visible pose from the body. The full entity hierarchy,
    /// <see cref="Sandbox.ModAPI.Ingame.IMyCubeGrid"/> surface, block
    /// membership, and terminal discovery arrive in  — this stage
    /// provides only the grid that owns the body.
    /// </summary>
    public sealed class SimCubeGrid
    {
        /// <summary>Stable entity id, unique per world, in spawn order.</summary>
        public long EntityId { get; }

        /// <summary>The grid's display name.</summary>
        public string DisplayName { get; }

        /// <summary>Grid cell size in meters.</summary>
        public float GridSize { get; }

        /// <summary>The rigid body this grid owns (one per dynamic grid).</summary>
        public SimRigidBody Body { get; }

        /// <summary>True once <see cref="Dispose"/> has run.</summary>
        public bool IsDisposed { get; private set; }

        /// <summary>World-visible position of the grid origin (from the body).</summary>
        public Vector3D Position => Body.PositionWorld;

        /// <summary>World transform of the grid origin (from the body).</summary>
        public MatrixD WorldMatrix => Body.WorldMatrix;

        public SimCubeGrid(long entityId, string displayName, float gridSize, SimRigidBody body)
        {
            EntityId = entityId;
            DisplayName = displayName;
            GridSize = gridSize;
            Body = body;
        }

        /// <summary>
        /// Disposes the grid's owned body. Idempotent; the grid itself is a
        /// lightweight handle around the body.
        /// </summary>
        public void Dispose()
        {
            if (IsDisposed)
            {
                return;
            }
            if (Body != null)
            {
                Body.Dispose();
            }
            IsDisposed = true;
        }
    }
}
