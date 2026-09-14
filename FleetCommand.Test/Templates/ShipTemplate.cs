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

        /// <summary>
        /// A gyroscope: contributes an axis-aligned box to shape and
        /// mass, and () its local orientation gives the
        /// grid-space override direction (<see cref="SimGyro"/>) via the
        /// grid's <see cref="GyroRateSystem"/>.
        /// </summary>
        Gyro,

        /// <summary>An ordinary ship controller (IMyShipController). .</summary>
        ShipController,

        /// <summary>A remote control (IMyRemoteControl). .</summary>
        RemoteControl,

        /// <summary>A cockpit (IMyCockpit). .</summary>
        Cockpit,

        /// <summary>A programmable block (IMyProgrammableBlock). .</summary>
        ProgrammableBlock,

        /// <summary>A text panel / LCD (IMyTextPanel). .</summary>
        TextPanel,

        /// <summary>
        /// A battery (IMyBatteryBlock): the ship's finite power supply for
        /// its SimPowerBus (). Contributes an axis-aligned box to
        /// shape and mass like any block.
        /// </summary>
        Battery,
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

        /// <summary>The block kind. Standard and fixed thrusters are physical; the rest arrive with their stages.</summary>
        public BlockKind Kind { get; }

        /// <summary>
        /// Battery maximum output power in MW (used when <see cref="Kind"/>
        /// is <see cref="BlockKind.Battery"/>; default 0). .
        /// </summary>
        public float MaxOutput { get; }

        /// <summary>
        /// Battery maximum charging input power in MW (used when
        /// <see cref="Kind"/> is <see cref="BlockKind.Battery"/>;
        /// default 0). .
        /// </summary>
        public float MaxInput { get; }

        /// <summary>
        /// Battery capacity in MWh (used when <see cref="Kind"/> is
        /// <see cref="BlockKind.Battery"/>; default 0). .
        /// </summary>
        public float MaxStoredPower { get; }

        /// <summary>
        /// Battery initial charge as a fraction of
        /// <see cref="MaxStoredPower"/> (0..1, default 1 — a spawned
        /// battery starts full). Used when <see cref="Kind"/> is
        /// <see cref="BlockKind.Battery"/>. .
        /// </summary>
        public float InitialStoredPower { get; }

        /// <summary>
        /// Thruster: the block's grid-body FORWARD direction — the axis its
        /// exhaust points along (one of the six cardinal unit axes, matching
        /// the game's <c>MyThrust.ThrustForwardVector</c> /
        /// <c>Orientation.Forward</c>). The grid is pushed OPPOSITE to
        /// this direction (game: <c>ThrustForce = -ThrustForwardVector</c>,
        /// MyThrust.cs:188). Used when <see cref="Kind"/> is
        /// <see cref="BlockKind.FixedThruster"/>; null for non-thrusters.
        ///  (<see cref="SimThrust.GridThrustDirection"/>).
        /// </summary>
        public Vector3I? ThrustDirection { get; }

        /// <summary>
        /// Thruster: maximum thrust at full effectiveness, in Newtons.
        /// Used when <see cref="Kind"/> is <see cref="BlockKind.FixedThruster"/>;
        /// default 0 (a block with no thruster params). .
        /// </summary>
        public float MaxThrust { get; }

        /// <summary>
        /// Thruster: current effectiveness profile multiplier in [0, 1]
        /// (1 = full effectiveness in this environment, 0 = no effect —
        /// e.g. an atmospheric thruster in vacuum). <see cref="MaxEffectiveThrust"/>
        /// is <see cref="MaxThrust"/> x this value. Used when
        /// <see cref="Kind"/> is <see cref="BlockKind.FixedThruster"/>;
        /// defaults to 1 (a thruster in its intended environment), the
        /// game's full-effectiveness profile. .
        /// </summary>
        public float Effectiveness { get; }

        /// <summary>
        /// Thruster: power the block needs (MW) for full capability; the
        /// grid's <see cref="SpaceEngineers.SimPowerBus"/> allocates this in
        /// phase 4 and the delivered ratio scales the delivered thrust
        /// (single source of truth). Default 0 (ideal source). .
        /// </summary>
        public float ThrusterPowerDemand { get; }

        /// <summary>
        /// Gyro: maximum gyroscopic force at full power (the template
        /// value the game's <c>ForceMagnitude</c> would supply), in N·m.
        /// <see cref="SimGyro.MaxGyroForce"/> is this value x the gyro's
        /// <c>GyroPower</c> (in [0, 1]). Used when <see cref="Kind"/> is
        /// <see cref="BlockKind.Gyro"/>; default 0 (a block with no gyro
        /// params). .
        /// </summary>
        public float GyroForce { get; }

        /// <summary>
        /// Gyro: power the block needs (MW) at full power setting; the
        /// grid's <see cref="SpaceEngineers.SimPowerBus"/> allocates this
        /// in phase 4 (scaled by the gyro's <c>GyroPower</c>) and the
        /// delivered ratio scales the delivered torque (single source of
        /// truth). Default 0 (ideal source). .
        /// </summary>
        public float GyroPowerDemand { get; }

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
            Quaternion? orientation = null, BlockKind kind = BlockKind.Standard,
            float maxOutput = 0f, float maxInput = 0f,
            float maxStoredPower = 0f, float initialStoredPower = 1f,
            Vector3I? thrustDirection = null, float maxThrust = 0f,
            float effectiveness = 1f, float powerDemand = 0f,
            float gyroForce = 0f, float gyroPowerDemand = 0f)
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
            if (thrustDirection.HasValue && !IsCardinalAxis(thrustDirection.Value))
            {
                throw new ArgumentException(
                    "A fixed thruster's forward direction must be one of the six cardinal axes (+-X, +-Y, +-Z).",
                    nameof(thrustDirection));
            }
            if (effectiveness < 0f || effectiveness > 1f)
            {
                throw new ArgumentOutOfRangeException(nameof(effectiveness), "Effectiveness must be in [0, 1].");
            }

            Name = name;
            Min = min;
            Size = size ?? Vector3I.One;
            DryMass = dryMass;
            InventoryMass = inventoryMass;
            Orientation = orientation ?? Quaternion.Identity;
            Kind = kind;
            MaxOutput = maxOutput;
            MaxInput = maxInput;
            MaxStoredPower = maxStoredPower;
            InitialStoredPower = initialStoredPower;
            ThrustDirection = thrustDirection;
            MaxThrust = maxThrust;
            Effectiveness = effectiveness;
            ThrusterPowerDemand = powerDemand;
            GyroForce = gyroForce;
            GyroPowerDemand = gyroPowerDemand;

            Vector3I max = Min + Size - Vector3I.One;
            Vector3 cellSum = (BlockCenter(min) + BlockCenter(max)) * (0.5f * gridSize);
            CenterLocal = cellSum;
            HalfExtents = new Vector3(Size) * gridSize * 0.5f;
        }

        private static bool IsCardinalAxis(Vector3I v)
        {
            int sum = Math.Abs(v.X) + Math.Abs(v.Y) + Math.Abs(v.Z);
            return sum == 1;
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
