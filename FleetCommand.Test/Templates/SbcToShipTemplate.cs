using System;
using System.Collections.Generic;
using VRageMath;
using FleetCommand.Test.Sbc;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// Converts an already-valid <see cref="SbcFile"/> (from <see cref="SbcFileReader"/>)
    /// into a <see cref="ShipTemplate"/>. The format layer has already guaranteed the
    /// geometry is legal; this layer only maps semantics — resolving each block to the
    /// sim block that realizes its kind via <see cref="SbcAdapterRegistry"/> (reflection
    /// over the implemented blocks), and rejecting any block the simulator does not
    /// implement, by throwing with the block named, since we cannot construct such a
    /// template.
    /// </summary>
    public static class SbcToShipTemplate
    {
        /// <summary>Converts a single-grid blueprint to a <see cref="ShipTemplate"/>.
        /// Throws <see cref="ArgumentException"/> if a block's subtype is not
        /// implemented in the simulator, or if the file has != 1 grid.</summary>
        public static ShipTemplate Convert(SbcFile file, string name)
        {
            if (file == null)
            {
                throw new ArgumentNullException(nameof(file));
            }
            if (file.Grids.Count != 1)
            {
                throw new ArgumentException(
                    $"Blueprint '{file.Name}' has {file.Grids.Count} grids; a ShipTemplate models exactly one rigid grid.");
            }
            return Convert(file.Grids[0], name);
        }

        /// <summary>Converts one grid of an assembly, preserving its static flag.
        /// World placement and inter-grid links are the assembly caller's responsibility.</summary>
        public static ShipTemplate Convert(SbcGrid grid, string name)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));
            float gridSize = grid.GridSize == SbcGridSize.Small ? 0.5f : 2.5f;

            var blocks = new List<TemplateBlock>();
            string designated = null;
            int n = 0;
            foreach (SbcBlock b in grid.Blocks)
            {
                n++;
                if (!SbcAdapterRegistry.TryGet(b.SubtypeId, out ISbcBlockAdapter adapter))
                {
                    throw new ArgumentException(
                        $"Block '{b.SubtypeId}' (xsi:type {b.TypeId}) is not implemented in the simulator — " +
                        "this blueprint cannot be built as a ShipTemplate.");
                }
                TemplateBlock block = adapter.Build(b, gridSize, (string)b.Data.Element("CustomName") ?? "B" + n);
                blocks.Add(block);
                if (designated == null && block.IsControllerKind)
                {
                    designated = block.Name;
                }
            }
            return new ShipTemplate(name, gridSize, blocks, isStatic: grid.IsStatic,
                designatedControllerName: designated);
        }
    }
}
