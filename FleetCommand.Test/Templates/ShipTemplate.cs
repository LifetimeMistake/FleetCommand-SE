using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// The block kind for physical construction in this stage. Only the
    /// kind's effect on mass, shape, and (for fixed thrusters) the fact
    /// that they contribute mass without applying torque is represented here;
    /// thruster behaviour itself arrives in .
    /// </summary>
    public enum BlockKind
    {
        /// <summary>An ordinary block; contributes an axis-aligned box to shape and mass.</summary>
        Standard,

        /// <summary>
        /// A fixed thruster: contributes an axis-aligned box to shape and
        /// mass (fixed thrusters still affect mass and inertia even though
        /// they apply no torque — ).
        /// </summary>
        FixedThruster,

        /// <summary>An ordinary ship controller (IMyShipController). .</summary>
        ShipController,

        /// <summary>A remote control (IMyRemoteControl). .</summary>
        RemoteControl,

        /// <summary>A cockpit (IMyCockpit). .</summary>
        Cockpit,
    }

    /// <summary>
    /// One block's physical description inside a <see cref="ShipTemplate"/>:
    /// its grid-local placement (min cell plus cell size), dry mass,
    /// optional inventory mass, orientation, and kind. Blocks are rigid,
    /// axis-aligned boxes sized by <c>GridSize</c>; their center and half
    /// extents (grid-local meters) drive both the compound shape and the
    /// mass elements.
    /// </summary>
    public sealed class TemplateBlock
    {
        /// <summary>The block's name (unique per grid; used by later stages).</summary>
        public string Name { get; }

        /// <summary>The block's minimum grid cell (grid-local integer coordinates).</summary>
        public Vector3I Min { get; }

        /// <summary>The block's extent in whole cells (each &gt;= 1).</summary>
        public Vector3I Size { get; }

        /// <summary>Dry (structural) mass in kilograms.</summary>
        public float DryMass { get; }

        /// <summary>Optional inventory cargo mass in kilograms (0 = none).</summary>
        public float InventoryMass { get; }

        /// <summary>Grid-local orientation (no effect on box mass; carried for later stages).</summary>
        public Quaternion Orientation { get; }

        /// <summary>The block kind. Only <see cref="BlockKind.Standard"/> and <see cref="BlockKind.FixedThruster"/> exist this stage.</summary>
        public BlockKind Kind { get; }

        /// <summary>
        /// Grid-local center in meters: ((Min + Max) · 0.5) · GridSize,
        /// matching the game's mass-element placement
        /// (MyCubeBlockCollector.AddMass, MyGridShape.CollectBlockInventories).
        /// </summary>
        public Vector3 CenterLocal { get; }

        /// <summary>Grid-local half extents in meters: (Size · GridSize) · 0.5.</summary>
        public Vector3 HalfExtents { get; }

        /// <summary>
        /// The block's grid-local world transform (rotation from
        /// <see cref="Orientation"/>, translation <see cref="CenterLocal"/>).
        /// </summary>
        public Matrix LocalTransform
        {
            get
            {
                Matrix matrix = Matrix.CreateFromQuaternion(Orientation);
                matrix.Translation = CenterLocal;
                return matrix;
            }
        }

        public TemplateBlock(string name, Vector3I min, float dryMass, float gridSize,
            Vector3I? size = null, float inventoryMass = 0f,
            Quaternion? orientation = null, BlockKind kind = BlockKind.Standard)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("Block name must not be empty.", nameof(name));
            }
            if (dryMass <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(dryMass), "A block must have positive dry mass.");
            }
            if (gridSize <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(gridSize));
            }

            Name = name;
            Min = min;
            Size = size ?? Vector3I.One;
            DryMass = dryMass;
            InventoryMass = inventoryMass;
            Orientation = orientation ?? Quaternion.Identity;
            Kind = kind;

            Vector3I max = Min + Size - Vector3I.One;
            Vector3 cellSum = (BlockCenter(min) + BlockCenter(max)) * (0.5f * gridSize);
            CenterLocal = cellSum;
            HalfExtents = new Vector3(Size) * gridSize * 0.5f;
        }

        private static Vector3 BlockCenter(Vector3I v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }
    }

    /// <summary>
    /// Immutable physical description of one rigid grid (): grid
    /// size, the block list, static/dynamic collision mode, and this
    /// stage's physical scope (mass elements and compound shape from block
    /// defs with local transforms). The full validating builder arrives in
    /// ; this stage provides only enough structure to build mass
    /// and compound shape from blocks.
    /// </summary>
    public sealed class ShipTemplate
    {
        /// <summary>The template's display name.</summary>
        public string Name { get; }

        /// <summary>Grid cell size in meters (small grid 0.5, large grid 2.5).</summary>
        public float GridSize { get; }

        /// <summary>Physical block definitions, in deterministic spawn/declaration order.</summary>
        public IReadOnlyList<TemplateBlock> Blocks { get; }

        /// <summary>True for a static (fixed, non-simulated) grid; false for a dynamic grid.</summary>
        public bool IsStatic { get; }

        /// <summary>Per-grid linear speed cap in m/s (profile-configurable; not a live sector lookup).</summary>
        public float MaxLinearVelocity { get; }

        /// <summary>Per-grid angular speed cap in rad/s.</summary>
        public float MaxAngularVelocity { get; }

        /// <summary>Total dry block mass in kilograms (without inventory).</summary>
        public float DryMass
        {
            get
            {
                float sum = 0f;
                for (int i = 0; i < Blocks.Count; i++)
                {
                    sum += Blocks[i].DryMass;
                }
                return sum;
            }
        }

        public ShipTemplate(
            string name,
            float gridSize,
            IEnumerable<TemplateBlock> blocks,
            bool isStatic = false,
            float maxLinearVelocity = 0f,
            float maxAngularVelocity = 0f)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("Template name must not be empty.", nameof(name));
            }
            if (gridSize <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(gridSize));
            }
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }

            Name = name;
            GridSize = gridSize;
            Blocks = new List<TemplateBlock>(blocks);
            IsStatic = isStatic;
            MaxLinearVelocity = maxLinearVelocity;
            MaxAngularVelocity = maxAngularVelocity;
        }
    }

    /// <summary>
    /// Placeholder for the validating <see cref="ShipTemplate"/> builder
    /// (); arrives in .
    /// </summary>
    public sealed class ShipTemplateBuilder
    {
    }
}
