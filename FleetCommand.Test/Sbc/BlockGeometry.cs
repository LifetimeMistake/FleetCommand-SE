using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Sbc
{
    /// <summary>
    /// The geometric facts an SBC blueprint needs to be validated: a block subtype's
    /// cell size and the local faces it can mount through. Sourced from the game's
    /// <c>CubeBlocks</c> SBCs (an absent <c>&lt;MountPoints&gt;</c> list = all faces;
    /// thruster "Size" is 1x1x2 but a thruster only occupies two cells along its
    /// forward axis, not a ± extent). This is format-layer block data — deliberately
    /// separate from the sim's semantic catalog in <see cref="Templates.SbcToShipTemplate"/>.
    /// </summary>
    public sealed class BlockGeometry
    {
        /// <summary>The block's size in whole cells (each axis ≥ 1).</summary>
        public Vector3I Size { get; }

        /// <summary>True for a thruster: occupies two cells along its forward axis
        /// (the mount is on the Back face; the nozzle extends forward).</summary>
        public bool IsThruster { get; }

        /// <summary>The local faces (Front/Back/Top/Bottom/Left/Right) this block can
        /// mount through; empty means all faces.</summary>
        public IReadOnlyList<string> MountFaces { get; }

        public BlockGeometry(Vector3I size, IReadOnlyList<string> mountFaces = null, bool isThruster = false)
        {
            if (size.X < 1 || size.Y < 1 || size.Z < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(size), "A block's size must be at least 1 cell per axis.");
            }
            Size = size;
            IsThruster = isThruster;
            MountFaces = mountFaces ?? Array.Empty<string>();
        }

        /// <summary>The grid cells this block occupies (given its frame from the SBC
        /// orientation). A thruster occupies two cells along the positive unit of the
        /// axis its forward lies on (the block's Min is the mounting end and grows
        /// toward the grid). Other blocks take an oriented box of <see cref="Size"/>.
        /// Reproduces the game's placement so overlap and mount checks run on real
        /// occupancy.</summary>
        public ISet<Vector3I> Cells(SbcBlock block)
        {
            var frame = block.Frame;
            if (IsThruster)
            {
                var fw = frame.Forward;
                int axis = Math.Abs(fw.X) > 0 ? 0 : Math.Abs(fw.Y) > 0 ? 1 : 2;
                var e = new Vector3I(axis == 0 ? 1 : 0, axis == 1 ? 1 : 0, axis == 2 ? 1 : 0);
                return new HashSet<Vector3I>(Vector3IComparer.Instance)
                {
                    block.Min,
                    Add(block.Min, e),
                };
            }
            var cells = new HashSet<Vector3I>(Vector3IComparer.Instance);
            var right = frame.Right;
            var up = frame.Up;
            var back = frame.Back;
            for (int i = 0; i < Size.X; i++)
            {
                for (int j = 0; j < Size.Y; j++)
                {
                    for (int k = 0; k < Size.Z; k++)
                    {
                        Vector3I offset = Add(Add(Scaled(right, i), Scaled(up, j)), Scaled(back, k));
                        cells.Add(Add(block.Min, offset));
                    }
                }
            }
            return cells;
        }

        /// <summary>The world face names (internal) this block can mount through, given
        /// its frame — its local <see cref="MountFaces"/> mapped to world directions.
        /// An empty <see cref="MountFaces"/> means all faces.</summary>
        public ISet<string> WorldMountFaces(SbcBlock block)
        {
            var frame = block.Frame;
            var set = new HashSet<string>(StringComparer.Ordinal);
            var faces = MountFaces.Count == 0 ? CubeFacing.AllFaces : MountFaces;
            foreach (string face in faces)
            {
                set.Add(CubeFacing.FaceOfDir(frame.FaceWorldDir(face)));
            }
            return set;
        }

        private static Vector3I Add(Vector3I a, Vector3I b) => new Vector3I(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        private static Vector3I Scaled(Vector3I v, int s) => new Vector3I(v.X * s, v.Y * s, v.Z * s);
    }

    /// <summary>
    /// The geometry catalog: subtype id → <see cref="BlockGeometry"/>, covering the
    /// blocks the equivalence vessel uses. Values verified against the decompiled
    /// <c>CubeBlocks</c> SBCs (mounts from the real <c>&lt;MountPoints&gt;</c>;
    /// armor has no <c>xsi:type</c> and full-face default). A subtype absent here is
    /// passed through structural checks with a best-effort 1x1x1 all-face default —
    /// rejecting sim-unknown blocks is the converter's job (<see cref="Templates.SbcToShipTemplate"/>),
    /// not the format layer's.
    /// </summary>
    public static class BlockGeometryCatalog
    {
        private static readonly Dictionary<string, BlockGeometry> Catalog = Build();

        /// <summary>Looks up geometry for a subtype; true and the geometry when known,
        /// false with an all-face 1x1x1 default otherwise.</summary>
        public static bool TryGet(string subtypeId, out BlockGeometry geometry)
        {
            return Catalog.TryGetValue(subtypeId, out geometry);
        }

        /// <summary>True when the subtype has a real geometry entry.</summary>
        public static bool Contains(string subtypeId) => Catalog.ContainsKey(subtypeId);

        private static Dictionary<string, BlockGeometry> Build()
        {
            var d = new Dictionary<string, BlockGeometry>(StringComparer.Ordinal);
            var one = new Vector3I(1, 1, 1);
            var all = new[] { CubeFacing.FaceFront, CubeFacing.FaceBack, CubeFacing.FaceTop,
                              CubeFacing.FaceBottom, CubeFacing.FaceLeft, CubeFacing.FaceRight };
            var noFront = new[] { CubeFacing.FaceBack, CubeFacing.FaceTop, CubeFacing.FaceBottom,
                                  CubeFacing.FaceLeft, CubeFacing.FaceRight };

            // — small grid —
            // Armor: no <MountPoints> in the SBC => all faces (CubeBlocks_Armor.sbc).
            d["SmallBlockArmorBlock"] = new BlockGeometry(one, all);
            d["SmallBlockArmorSlope"] = new BlockGeometry(one,
                new[] { CubeFacing.FaceBottom, CubeFacing.FaceFront,
                        CubeFacing.FaceLeft, CubeFacing.FaceRight });
            d["SmallHeavyBlockArmorBlock"] = new BlockGeometry(one, all);
            // Remote control: full-face (CubeBlocks_Communications.sbc).
            d["SmallBlockRemoteControl"] = new BlockGeometry(one, all);
            // Programmable block: Front only (CubeBlocks_Automation.sbc).
            d["SmallProgrammableBlock"] = new BlockGeometry(one, new[] { CubeFacing.FaceFront });
            // Gyro: Bottom only (CubeBlocks_Control.sbc).
            d["SmallBlockGyro"] = new BlockGeometry(one, new[] { CubeFacing.FaceBottom });
            // Cockpit: 3x3x3, all faces but Top (CubeBlocks_Characters.sbc).
            d["SmallBlockCockpit"] = new BlockGeometry(new Vector3I(3, 3, 3),
                new[] { CubeFacing.FaceBottom, CubeFacing.FaceFront, CubeFacing.FaceBack,
                        CubeFacing.FaceLeft, CubeFacing.FaceRight });
            // LCD: 3x3x1, no Back face (CubeBlocks_LCDPanels.sbc).
            d["SmallLCDPanel"] = new BlockGeometry(new Vector3I(3, 3, 1), noFront);
            // Battery / reactor: full-face (CubeBlocks_Energy.sbc).
            d["SmallBlockSmallBatteryBlock"] = new BlockGeometry(one, all);
            d["SmallBlockSmallGenerator"] = new BlockGeometry(one, all);
            // Ion thruster: 1x1x2, mounts Back + the four sides, NOT Front
            // (CubeBlocks_Thrusters.sbc); occupies two cells along its forward axis.
            d["SmallBlockSmallThrust"] = new BlockGeometry(new Vector3I(1, 1, 2), noFront, isThruster: true);
            // Large (small-grid) ion thruster: 3x2x4 box, mounts Back + sides, NOT Front.
            d["SmallBlockLargeThrust"] = new BlockGeometry(new Vector3I(3, 2, 4), noFront);
            // Connectors (CubeBlocks_Conveyor.sbc): ConnectorSmall and ConnectorMedium are
            // small grid, no Front mount; ConnectorMedium is 3x3x2; inset connectors full-face.
            d["ConnectorSmall"] = new BlockGeometry(one, noFront);
            d["ConnectorMedium"] = new BlockGeometry(new Vector3I(3, 3, 2), noFront);
            d["SmallBlockInsetConnector"] = new BlockGeometry(one, all);

            // — large grid (the user's TEST GRID is a large grid) —
            d["LargeBlockArmorBlock"] = new BlockGeometry(one, all);
            // Armor slope: mounts the four vertical-ish faces and bottom, not Top or Back
            // (CubeBlocks_Armor.sbc).
            d["LargeBlockArmorSlope"] = new BlockGeometry(one,
                new[] { CubeFacing.FaceBottom, CubeFacing.FaceFront,
                        CubeFacing.FaceLeft, CubeFacing.FaceRight });
            d["LargeHeavyBlockArmorBlock"] = new BlockGeometry(one, all);
            d["LargeBlockRemoteControl"] = new BlockGeometry(one, all);
            // Large programmable block mounts through Bottom only (CubeBlocks_Automation.sbc).
            d["LargeProgrammableBlock"] = new BlockGeometry(one, new[] { CubeFacing.FaceBottom });
            d["LargeBlockGyro"] = new BlockGeometry(one, new[] { CubeFacing.FaceBottom });
            // Large cockpit mounts through Bottom only (CubeBlocks_Characters.sbc).
            d["LargeBlockCockpit"] = new BlockGeometry(one, new[] { CubeFacing.FaceBottom });
            d["LargeLCDPanel"] = new BlockGeometry(one, noFront);
            d["LargeBlockBatteryBlock"] = new BlockGeometry(one, all);
            d["LargeBlockSmallGenerator"] = new BlockGeometry(one, all);
            // Large reactor: 3x3x3, full face (CubeBlocks_Energy.sbc).
            d["LargeBlockLargeGenerator"] = new BlockGeometry(new Vector3I(3, 3, 3), all);
            // Large grid "small" ion thruster: 1x1x2, same thruster rule as the small one.
            d["LargeBlockSmallThrust"] = new BlockGeometry(new Vector3I(1, 1, 2), noFront, isThruster: true);
            // Large grid large ion thruster: 3x2x4 box, no Front mount.
            d["LargeBlockLargeThrust"] = new BlockGeometry(new Vector3I(3, 2, 4), noFront);
            // Large-grid connectors: standard connector mounts only Back/Bottom/Top;
            // large inset connector is full-face.
            d["Connector"] = new BlockGeometry(one,
                new[] { CubeFacing.FaceBack, CubeFacing.FaceBottom, CubeFacing.FaceTop });
            d["LargeBlockInsetConnector"] = new BlockGeometry(one, all);

            return d;
        }
    }
}
