using System;
using System.Collections.Generic;
using VRageMath;
using System.Xml.Linq;

namespace FleetCommand.Test.Sbc
{
    /// <summary>The grid size of an SBC cube grid (small 0.5 m / large 2.5 m cells).</summary>
    public enum SbcGridSize { Small, Large }

    /// <summary>One cube block as it appears in an SBC blueprint: its type and subtype
    /// ids, minimum grid cell, and <c>BlockOrientation</c> (the game's base-six
    /// forward/up direction names; identity = Forward/Up). Immutable, format-only —
    /// no geometry here.</summary>
    public sealed class SbcBlock
    {
        // Preserve serialized operating state without coupling structural validation to it.
        public XElement Data { get; }
        /// <summary>The block's type id (the <c>xsi:type</c> value, e.g.
        /// "CubeBlock", "RemoteControl", "Thrust").</summary>
        public string TypeId { get; }

        /// <summary>The block's subtype id (e.g. "SmallBlockArmorBlock").</summary>
        public string SubtypeId { get; }

        /// <summary>The block's minimum grid cell (grid-local integer coordinates).</summary>
        public Vector3I Min { get; }

        /// <summary>The block's <c>BlockOrientation</c> Forward direction name (a
        /// base-six name; default "Forward").</summary>
        public string ForwardName { get; }

        /// <summary>The block's <c>BlockOrientation</c> Up direction name (a base-six
        /// name; default "Up").</summary>
        public string UpName { get; }

        public SbcBlock(string typeId, string subtypeId, Vector3I min,
            string forwardName = CubeFacing.OrientForward,
            string upName = CubeFacing.OrientUp, XElement data = null)
        {
            if (string.IsNullOrEmpty(subtypeId))
            {
                throw new ArgumentException("A block needs a non-empty subtype id.", nameof(subtypeId));
            }
            TypeId = typeId ?? string.Empty;
            SubtypeId = subtypeId;
            Min = min;
            ForwardName = forwardName;
            UpName = upName;
            Data = data == null ? new XElement("MyObjectBuilder_CubeBlock") : new XElement(data);
        }

        /// <summary>This block's resolved world frame from its orientation.</summary>
        public BlockFrame Frame
        {
            get
            {
                return CubeFacing.MakeFrame(
                    CubeFacing.OrientToFaceName(ForwardName),
                    CubeFacing.OrientToFaceName(UpName));
            }
        }
    }

    /// <summary>One cube grid within an SBC file (grid size plus its ordered block
    /// list).</summary>
    public sealed class SbcGrid
    {
        /// <summary>The grid's cell size.</summary>
        public SbcGridSize GridSize { get; }

        /// <summary>The grid's blocks, in file order.</summary>
        public IReadOnlyList<SbcBlock> Blocks { get; }

        public SbcGrid(SbcGridSize gridSize, IEnumerable<SbcBlock> blocks)
        {
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }
            GridSize = gridSize;
            Blocks = new List<SbcBlock>(blocks);
        }
    }

    /// <summary>A parsed SBC blueprint file (one or more cube grids).</summary>
    public sealed class SbcFile
    {
        /// <summary>The blueprint's subtype (name) as written in the file.</summary>
        public string Name { get; }

        /// <summary>The file's grids, in file order.</summary>
        public IReadOnlyList<SbcGrid> Grids { get; }

        public SbcFile(string name, IEnumerable<SbcGrid> grids)
        {
            if (grids == null)
            {
                throw new ArgumentNullException(nameof(grids));
            }
            Name = name ?? string.Empty;
            Grids = new List<SbcGrid>(grids);
        }
    }

    /// <summary>Structural equality for <see cref="Vector3I"/> grid-cell keys.</summary>
    internal sealed class Vector3IComparer : IEqualityComparer<Vector3I>
    {
        public static readonly Vector3IComparer Instance = new Vector3IComparer();

        public bool Equals(Vector3I x, Vector3I y)
        {
            return x.X == y.X && x.Y == y.Y && x.Z == y.Z;
        }

        public int GetHashCode(Vector3I v)
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + v.X;
                hash = hash * 31 + v.Y;
                hash = hash * 31 + v.Z;
                return hash;
            }
        }
    }
}
