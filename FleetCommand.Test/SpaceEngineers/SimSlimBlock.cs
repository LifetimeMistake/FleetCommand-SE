using System;
using System.Collections.Generic;
using VRage.Game.ModAPI.Ingame;
using VRage.ObjectBuilders;
using VRage.Utils;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>Thin slim-block view returned by <see cref="SimCubeGrid.GetCubeBlock"/> for the grid's block-membership surface: wraps the fat terminal block and reports position, grid, fat block, and mass. Integrity, damage, deformation, stockpile, color, and skin data are not modeled (no damage model) and those members throw a naming <see cref="NotSupportedException"/>.</summary>
    public sealed class SimSlimBlock : IMySlimBlock
    {
        private readonly SimTerminalBlock _fat;

        internal SimSlimBlock(SimTerminalBlock fatBlock)
        {
            _fat = fatBlock ?? throw new ArgumentNullException(nameof(fatBlock));
        }

        /// <inheritdoc />
        public Vector3I Position => _fat.Position;

        /// <inheritdoc />
        public IMyCubeGrid CubeGrid => _fat.CubeGrid;

        /// <inheritdoc />
        public IMyCubeBlock FatBlock => _fat;

        /// <inheritdoc />
        public float Mass => _fat.Mass;

        /// <inheritdoc />
        public SerializableDefinitionId BlockDefinition => ThrowNotSupported<SerializableDefinitionId>("IMySlimBlock.BlockDefinition");

        /// <inheritdoc />
        public float AccumulatedDamage => ThrowNotSupported<float>("IMySlimBlock.AccumulatedDamage");

        /// <inheritdoc />
        public float BuildIntegrity => ThrowNotSupported<float>("IMySlimBlock.BuildIntegrity");

        /// <inheritdoc />
        public float BuildLevelRatio => ThrowNotSupported<float>("IMySlimBlock.BuildLevelRatio");

        /// <inheritdoc />
        public float CurrentDamage => ThrowNotSupported<float>("IMySlimBlock.CurrentDamage");

        /// <inheritdoc />
        public float DamageRatio => ThrowNotSupported<float>("IMySlimBlock.DamageRatio");

        /// <inheritdoc />
        public bool HasDeformation => ThrowNotSupported<bool>("IMySlimBlock.HasDeformation");

        /// <inheritdoc />
        public bool IsDestroyed => ThrowNotSupported<bool>("IMySlimBlock.IsDestroyed");

        /// <inheritdoc />
        public bool IsFullIntegrity => ThrowNotSupported<bool>("IMySlimBlock.IsFullIntegrity");

        /// <inheritdoc />
        public bool IsFullyDismounted => ThrowNotSupported<bool>("IMySlimBlock.IsFullyDismounted");

        /// <inheritdoc />
        public float MaxDeformation => ThrowNotSupported<float>("IMySlimBlock.MaxDeformation");

        /// <inheritdoc />
        public float MaxIntegrity => ThrowNotSupported<float>("IMySlimBlock.MaxIntegrity");

        /// <inheritdoc />
        public long OwnerId => ThrowNotSupported<long>("IMySlimBlock.OwnerId");

        /// <inheritdoc />
        public bool ShowParts => ThrowNotSupported<bool>("IMySlimBlock.ShowParts");

        /// <inheritdoc />
        public bool StockpileAllocated => ThrowNotSupported<bool>("IMySlimBlock.StockpileAllocated");

        /// <inheritdoc />
        public bool StockpileEmpty => ThrowNotSupported<bool>("IMySlimBlock.StockpileEmpty");

        /// <inheritdoc />
        public Vector3 ColorMaskHSV => ThrowNotSupported<Vector3>("IMySlimBlock.ColorMaskHSV");

        /// <inheritdoc />
        public MyStringHash SkinSubtypeId => ThrowNotSupported<MyStringHash>("IMySlimBlock.SkinSubtypeId");

        /// <inheritdoc />
        public void GetMissingComponents(Dictionary<string, int> addToDictionary)
        {
            throw NotSupported("IMySlimBlock.GetMissingComponents()");
        }

        private static NotSupportedException NotSupported(string member)
        {
            return new NotSupportedException(
                "SimSlimBlock does not implement '" + member + "' (outside the implemented contract).");
        }

        private static T ThrowNotSupported<T>(string member)
        {
            throw NotSupported(member);
        }
    }
}
