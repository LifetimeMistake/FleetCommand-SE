using System;
using VRage.Game.Components.Interfaces;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>Root of the simulator entity hierarchy, implementing <see cref="IMyEntity"/>:
    /// provides the identity, naming, and world-matrix surface shared by grids and blocks.
    /// <see cref="WorldMatrix"/> is abstract (a grid's transform is owned by its rigid body, a
    /// block's is derived from the grid body plus its local pose). Members outside the implemented
    /// contract throw a naming <see cref="NotSupportedException"/> rather than returning a
    /// default.</summary>
    public abstract class SimEntity : IMyEntity
    {
        /// <inheritdoc />
        public long EntityId { get; }

        /// <inheritdoc />
        public string Name { get; }

        /// <inheritdoc />
        public virtual string DisplayName => Name;

        /// <inheritdoc />
        public abstract MatrixD WorldMatrix { get; }

        /// <summary>True once this entity has been closed/disposed.</summary>
        public bool IsDisposed { get; protected set; }

        /// <inheritdoc />
        public bool Closed => IsDisposed;

        protected SimEntity(long entityId, string name)
        {
            if (entityId == 0)
            {
                throw new ArgumentException("Entity ids must be non-zero.", nameof(entityId));
            }
            EntityId = entityId;
            Name = name;
        }

        /// <inheritdoc />
        public Vector3D GetPosition()
        {
            return WorldMatrix.Translation;
        }

        // Members outside the implemented contract: component containers,
        // inventories, and bounding volumes are not modeled.

        /// <inheritdoc />
        public virtual IMyEntityComponentContainer Components => ThrowNotSupported<IMyEntityComponentContainer>("IMyEntity.Components");

        /// <inheritdoc />
        public virtual bool HasInventory => ThrowNotSupported<bool>("IMyEntity.HasInventory");

        /// <inheritdoc />
        public virtual int InventoryCount => ThrowNotSupported<int>("IMyEntity.InventoryCount");

        /// <inheritdoc />
        public virtual BoundingBoxD WorldAABB => ThrowNotSupported<BoundingBoxD>("IMyEntity.WorldAABB");

        /// <inheritdoc />
        public virtual BoundingBoxD WorldAABBHr => ThrowNotSupported<BoundingBoxD>("IMyEntity.WorldAABBHr");

        /// <inheritdoc />
        public virtual BoundingSphereD WorldVolume => ThrowNotSupported<BoundingSphereD>("IMyEntity.WorldVolume");

        /// <inheritdoc />
        public virtual BoundingSphereD WorldVolumeHr => ThrowNotSupported<BoundingSphereD>("IMyEntity.WorldVolumeHr");

        /// <inheritdoc />
        public virtual IMyInventory GetInventory() => ThrowNotSupported<IMyInventory>("IMyEntity.GetInventory()");

        /// <inheritdoc />
        public virtual IMyInventory GetInventory(int index) => ThrowNotSupported<IMyInventory>("IMyEntity.GetInventory(int)");

        /// <summary>Builds the canonical not-implemented exception naming the member.</summary>
        protected static NotSupportedException NotSupported(string member)
        {
            return new NotSupportedException(
                "SimEntity does not implement '" + member + "' (outside the implemented contract).");
        }

        /// <summary>Throws <see cref="NotSupportedException"/> naming
        /// <paramref name="member"/>.</summary>
        protected static T ThrowNotSupported<T>(string member)
        {
            throw NotSupported(member);
        }
    }
}
