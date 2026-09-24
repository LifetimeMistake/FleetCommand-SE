using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Templates
{
    /// <summary>The block kind for physical construction; the kind's effect on mass and shape is
    /// represented here.</summary>
    public enum BlockKind
    {
        /// <summary>An ordinary block; contributes an axis-aligned box to shape and mass.</summary>
        Standard,

        /// <summary>A fixed thruster: contributes an axis-aligned box to shape and mass (applies no
        /// torque).</summary>
        FixedThruster,

        /// <summary>A gyroscope: contributes an axis-aligned box to shape and mass; its local
        /// orientation gives the grid-space override direction.</summary>
        Gyro,

        /// <summary>An ordinary ship controller (IMyShipController).</summary>
        ShipController,

        /// <summary>A remote control (IMyRemoteControl).</summary>
        RemoteControl,

        /// <summary>A cockpit (IMyCockpit).</summary>
        Cockpit,

        /// <summary>A programmable block (IMyProgrammableBlock).</summary>
        ProgrammableBlock,

        /// <summary>A text panel / LCD (IMyTextPanel).</summary>
        TextPanel,

        /// <summary>A battery (IMyBatteryBlock): the ship's finite power supply for its
        /// SimPowerBus.</summary>
        Battery,

        /// <summary>A reactor (IMyReactor): a second, finite power producer, giving a battery
        /// something to charge from.</summary>
        Reactor,

        /// <summary>A ship connector (IMyShipConnector): contributes mass, and its connection frame
        /// drives world pose and capture geometry.</summary>
        Connector,
    }

    /// <summary>
    /// One block's physical description inside a <see cref="ShipTemplate"/>: grid-local
    /// placement, dry mass, optional inventory mass, orientation, and kind. Blocks are
    /// rigid axis-aligned boxes sized by <c>GridSize</c>; their center and half extents
    /// drive both the compound shape and the mass elements.
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

        /// <summary>Grid-local orientation (no effect on box mass; carried for later
        /// stages).</summary>
        public Quaternion Orientation { get; }

        /// <summary>The block kind. Standard and fixed thrusters are physical; the rest arrive with
        /// their stages.</summary>
        public BlockKind Kind { get; }

        /// <summary>Battery / reactor maximum output power in MW (<see cref="BlockKind.Battery"/>
        /// or <see cref="BlockKind.Reactor"/>; default 0).</summary>
        public float MaxOutput { get; }

        /// <summary>Battery maximum charging input power in MW (<see cref="BlockKind.Battery"/>;
        /// default 0).</summary>
        public float MaxInput { get; }

        /// <summary>Battery capacity in MWh (<see cref="BlockKind.Battery"/>; default 0).</summary>
        public float MaxStoredPower { get; }

        /// <summary>Battery initial charge as a fraction of <see cref="MaxStoredPower"/> (0..1,
        /// default 1 — a spawned battery starts full).</summary>
        public float InitialStoredPower { get; }

        /// <summary>Reactor initial fuel in item-count units (the game's uranium inventory, counted
        /// in <c>ConsumptionPerSecond_Items</c>). Reactor output is binary — full
        /// <see cref="MaxOutput"/> while any fuel remains — so this sets only runtime, never
        /// partial output. Default large (effectively always fueled).</summary>
        public float InitialFuel { get; }

        /// <summary>Reactor fuel consumed per second at full output, in item-count units (the
        /// game's <c>ConsumptionPerSecond_Items</c>, MyReactor.cs:271); actual consumption
        /// scales with delivered output. Default 1.</summary>
        public float FuelConsumptionPerSecond { get; }

        /// <summary>
        /// Thruster: the grid-body FORWARD direction, the axis its exhaust points along
        /// (a cardinal axis; the game's <c>MyThrust.ThrustForwardVector</c>). The grid is
        /// pushed OPPOSITE to this (game: <c>ThrustForce = -ThrustForwardVector</c>,
        /// MyThrust.cs:188). Null for non-thrusters.
        /// </summary>
        public Vector3I? ThrustDirection { get; }

        /// <summary>Thruster: maximum thrust at full effectiveness, in Newtons (default 0 — no
        /// thruster params).</summary>
        public float MaxThrust { get; }

        /// <summary>
        /// Thruster: effectiveness profile multiplier in [0, 1] (1 = full in this
        /// environment, 0 = none — e.g. an atmospheric thruster in vacuum). Defaults to 1.
        /// </summary>
        public float Effectiveness { get; }

        /// <summary>Thruster: power needed (MW) for full capability; the delivered ratio scales
        /// delivered thrust. Default 0 (ideal source).</summary>
        public float ThrusterPowerDemand { get; }

        /// <summary>
        /// Thruster: the definition's <c>SlowdownFactor</c> — the multiplier on the
        /// delivered thrust envelope in the base (manual + dampened) path
        /// (MyThrusterBlockThrustComponent :225/:240/:250; default from
        /// <c>MyThrustDefinition</c>, :106). Defaults to 1 (see
        /// <see cref="SimThrust.SlowdownFactor"/>).
        /// </summary>
        public float SlowdownFactor { get; }

        /// <summary>Gyro: maximum gyroscopic force at full power, in N·m;
        /// <see cref="SimGyro.MaxGyroForce"/> is this x the gyro's <c>GyroPower</c>.</summary>
        public float GyroForce { get; }

        /// <summary>Gyro: power needed (MW) at full setting; the delivered ratio scales delivered
        /// torque. Default 0 (ideal source).</summary>
        public float GyroPowerDemand { get; }

        /// <summary>
        /// Connector: additional rotation of the connection frame relative to the block's own
        /// placement, reorienting the connect face without moving the block. Default
        /// <c>Quaternion.Identity</c> (follows the block's orientation).
        /// </summary>
        public Quaternion ConnectorOrientation { get; }

        /// <summary>Connector: this block's own capture reach in meters; 0 (default) means use the
        /// physics profile's default detection.</summary>
        public float ConnectorReach { get; }

        /// <summary>Connector: the pull strength in [0, 1] (the connector's <c>Strength</c>
        /// slider, MyShipConnector.cs:845-869); the pair's minimum drives the approach hinge's
        /// hold. Template-default 1.</summary>
        public float ConnectorStrength { get; }

        /// <summary>
        /// Connector: the block's per-definition attach-direction selector, a six-face
        /// direction in the block's OWN frame — the game's <c>ConnectDirection</c>
        /// (<c>MyShipConnector.cs:1489-1490</c>: a = ConnectDirection.X·Left +
        /// .Y·Up + .Z·Forward). Default <c>Vector3.Forward</c> (0,0,-1), the
        /// object-builder default (MyObjectBuilder_ShipConnectorDefinition.cs:1659); e.g. the
        /// structural platform connector sets (0,-1,0) — connect along -Up.
        /// </summary>
        public Vector3 ConnectDirection { get; }

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
            float gyroForce = 0f, float gyroPowerDemand = 0f,
            Quaternion? connectorOrientation = null, float connectorReach = 0f,
            float connectorStrength = 1f,
            float slowdownFactor = 1f,
            float initialFuel = 1000000f, float fuelConsumptionPerSecond = 1f,
            Vector3? connectDirection = null)
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
            if (connectorStrength < 0f || connectorStrength > 1f)
            {
                throw new ArgumentOutOfRangeException(nameof(connectorStrength), "Connector strength must be in [0, 1].");
            }
            Vector3 connectDir = connectDirection ?? Vector3.Forward;
            if (!IsCardinalAxis(connectDir))
            {
                throw new ArgumentException(
                    "A connector's connect direction must be one of the six cardinal axes (+-X, +-Y, +-Z).",
                    nameof(connectDirection));
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
            ConnectorOrientation = connectorOrientation ?? Quaternion.Identity;
            ConnectorReach = connectorReach;
            ConnectDirection = connectDir;
            ConnectorStrength = connectorStrength;
            SlowdownFactor = slowdownFactor;
            InitialFuel = initialFuel;
            FuelConsumptionPerSecond = fuelConsumptionPerSecond;

            Vector3I max = Min + Size - Vector3I.One;
            Vector3 cellSum = (BlockCenter(min) + BlockCenter(max)) * (0.5f * gridSize);
            CenterLocal = cellSum;
            HalfExtents = new Vector3(Size) * gridSize * 0.5f;
        }

        /// <summary>
        /// True when this block kind acts as a ship controller (provides the
        /// controller reference frame the template may designate): one of
        /// <see cref="ShipController"/>, <see cref="RemoteControl"/> or
        /// <see cref="Cockpit"/>.
        /// </summary>
        public bool IsControllerKind
        {
            get
            {
                return Kind == BlockKind.ShipController
                    || Kind == BlockKind.RemoteControl
                    || Kind == BlockKind.Cockpit;
            }
        }

        private static bool IsCardinalAxis(Vector3I v)
        {
            int sum = Math.Abs(v.X) + Math.Abs(v.Y) + Math.Abs(v.Z);
            return sum == 1;
        }

        private static bool IsCardinalAxis(Vector3 v)
        {
            int sum = (int)Math.Round(Math.Abs(v.X)) + (int)Math.Round(Math.Abs(v.Y)) + (int)Math.Round(Math.Abs(v.Z));
            return sum == 1;
        }

        private static Vector3 BlockCenter(Vector3I v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }
    }

    /// <summary>
    /// A named grid-local reference direction carried by a <see cref="ShipTemplate"/>
    /// (e.g. a controller or weapon reference frame). Purely template data — craft
    /// identity comes from data, never from a branch on craft type.
    /// </summary>
    public sealed class NamedReferenceAxis
    {
        /// <summary>The reference's name (must be unique and non-empty).</summary>
        public string Name { get; }

        /// <summary>The grid-local unit direction of the reference.</summary>
        public Vector3 Axis { get; }

        public NamedReferenceAxis(string name, Vector3 axis)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("A named reference axis needs a non-empty name.", nameof(name));
            }
            if (!IsFinite(axis) || axis.LengthSquared() < 1e-12f)
            {
                throw new ArgumentException(
                    $"Named reference axis '{name}' must be a finite, non-zero direction.", nameof(axis));
            }
            Name = name;
            Axis = axis;
        }

        internal static bool IsFinite(Vector3 v)
        {
            return IsFinite(v.X) && IsFinite(v.Y) && IsFinite(v.Z);
        }

        internal static bool IsFinite(float f)
        {
            return !float.IsNaN(f) && !float.IsInfinity(f);
        }
    }

    /// <summary>
    /// Immutable physical description of one rigid grid: grid size, block list,
    /// static/dynamic collision mode, designated controller, and named reference axes.
    /// Built by <see cref="ShipTemplateBuilder"/>; building once and spawning many times
    /// never mutates it (entity IDs are assigned per spawn).
    /// </summary>
    public sealed class ShipTemplate
    {
        /// <summary>The template's display name.</summary>
        public string Name { get; }

        /// <summary>Grid cell size in meters (small grid 0.5, large grid 2.5).</summary>
        public float GridSize { get; }

        /// <summary>Physical block definitions, in deterministic spawn/declaration order.</summary>
        public IReadOnlyList<TemplateBlock> Blocks { get; }

        /// <summary>True for a static (fixed, non-simulated) grid; false for a dynamic
        /// grid.</summary>
        public bool IsStatic { get; }

        /// <summary>Per-grid linear speed cap in m/s (profile-configurable; not a live sector
        /// lookup).</summary>
        public float MaxLinearVelocity { get; }

        /// <summary>Per-grid angular speed cap in rad/s.</summary>
        public float MaxAngularVelocity { get; }

        /// <summary>The grid's designated (primary) controller name, or null when none; when set it
        /// must resolve to a controller-kind block.</summary>
        public string DesignatedControllerName { get; }

        /// <summary>Named grid-local reference directions (e.g. a weapon reference axis); empty
        /// when none declared.</summary>
        public IReadOnlyList<NamedReferenceAxis> NamedAxes { get; }

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
            float maxAngularVelocity = 0f,
            string designatedControllerName = null,
            IEnumerable<NamedReferenceAxis> namedAxes = null)
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
            DesignatedControllerName = designatedControllerName;
            NamedAxes = new List<NamedReferenceAxis>(namedAxes ?? Array.Empty<NamedReferenceAxis>());
        }
    }
}
