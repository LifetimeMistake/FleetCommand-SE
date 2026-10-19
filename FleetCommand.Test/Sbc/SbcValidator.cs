using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Sbc
{
    /// <summary>Raised when an SBC grid is structurally invalid (an overlap, a block
    /// with no legal mount, or a grid broken into disconnected pieces). Carries a
    /// human-readable reason per offending block.</summary>
    public sealed class SbcValidationException : Exception
    {
        /// <summary>The grid-local block index of the offending block when the problem
        /// is block-specific (else -1).</summary>
        public int BlockIndex { get; }

        public SbcValidationException(string message, int blockIndex = -1)
            : base(message)
        {
            BlockIndex = blockIndex;
        }
    }

    /// <summary>
    /// Structural validation for an <see cref="SbcGrid"/> — the format layer's one
    /// job. Rejects, with a message naming the offender: overlapping blocks (bad
    /// placement), blocks with no legal mount onto a neighbor (illegal mount points),
    /// and grids that do not form a single connected piece (floating blocks). Runs on
    /// the real occupied cells and mount faces via <see cref="BlockGeometry"/> /
    /// <see cref="CubeFacing"/> (same math as the hand-grown layout generator). This is
    /// intentionally separate from <see cref="Templates.SbcToShipTemplate"/>, which
    /// maps semantics and rejects sim-unknown blocks.
    /// </summary>
    public static class SbcValidator
    {
        /// <summary>Validates <paramref name="grid"/>; throws
        /// <see cref="SbcValidationException"/> on the first class of structural
        /// error (overlap first, then mounts, then connectivity).</summary>
        public static void Validate(SbcGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            var blocks = grid.Blocks;
            int count = blocks.Count;
            if (count == 0)
            {
                throw new SbcValidationException("A grid has no blocks; a blueprint needs at least one block.");
            }

            // Geometry per block (best-effort default for subtypes the catalog doesn't know —
            // rejecting sim-unknown blocks is the converter's job, not the format layer's).
            var geo = new BlockGeometry[count];
            for (int i = 0; i < count; i++)
            {
                if (!BlockGeometryCatalog.TryGet(blocks[i].SubtypeId, out geo[i]))
                {
                    geo[i] = new BlockGeometry(Vector3I.One, CubeFacing.AllFaces);
                }
            }

            // — occupancy (bad placement): every block's real cells must be distinct —
            var occ = new Dictionary<Vector3I, int>(Vector3IComparer.Instance);
            for (int i = 0; i < count; i++)
            {
                foreach (Vector3I c in geo[i].Cells(blocks[i]))
                {
                    if (occ.TryGetValue(c, out int other))
                    {
                        throw new SbcValidationException(
                            $"Block '{blocks[i].SubtypeId}' at {blocks[i].Min} overlaps block " +
                            $"'{blocks[other].SubtypeId}' at cell {c}; a blueprint must not occupy a cell twice.",
                            i);
                    }
                    occ[c] = i;
                }
            }

            // — mounts + connectivity (union-find) —
            var parent = new int[count];
            for (int i = 0; i < count; i++)
            {
                parent[i] = i;
            }

            for (int i = 0; i < count; i++)
            {
                SbcBlock b1 = blocks[i];
                ISet<string> wm1 = geo[i].WorldMountFaces(b1);
                bool anyNeighbor = false; // shares a face with another block at all
                bool gotMount = false;     // shares a face both blocks can attach through
                foreach (Vector3I c in geo[i].Cells(b1))
                {
                    foreach (string face in CubeFacing.AllFaces)
                    {
                        Vector3I step = CubeFacing.DirOf(face);
                        if (occ.TryGetValue(Add(c, step), out int j))
                        {
                            anyNeighbor = true;
                            string opp = CubeFacing.FaceOfDir(Neg(step));
                            if (wm1.Contains(face) && geo[j].WorldMountFaces(blocks[j]).Contains(opp))
                            {
                                gotMount = true;
                                Union(parent, i, j);
                            }
                        }
                    }
                }
                // A block with no face-neighbor is legal (a lone block is its own grid;
                // if it lacks neighbors in a multi-block grid, connectivity rejects it).
                // A block with face-neighbors must attach through at least one legal pair.
                if (!gotMount && anyNeighbor)
                {
                    throw new SbcValidationException(
                        $"Block '{b1.SubtypeId}' at {b1.Min} shares a face with another block but has no legal " +
                        $"mount: no neighbor across a face both blocks can attach through (illegal mount point).", i);
                }
            }

            // — connectivity (floating blocks): exactly one component —
            int roots = 0;
            for (int i = 0; i < count; i++)
            {
                if (Find(parent, i) == i)
                {
                    roots++;
                }
            }
            if (roots != 1)
            {
                throw new SbcValidationException(
                    $"Grid is broken into {roots} disconnected pieces; every block must connect into one grid (floating blocks).");
            }
        }

        private static int Find(int[] parent, int i)
        {
            while (parent[i] != i)
            {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        }

        private static void Union(int[] parent, int a, int b)
        {
            int ra = Find(parent, a);
            int rb = Find(parent, b);
            if (ra != rb)
            {
                parent[ra] = rb;
            }
        }

        private static Vector3I Add(Vector3I a, Vector3I b) => new Vector3I(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        private static Vector3I Neg(Vector3I v) => new Vector3I(-v.X, -v.Y, -v.Z);
    }
}
