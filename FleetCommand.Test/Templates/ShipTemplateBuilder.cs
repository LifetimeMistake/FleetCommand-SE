using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// The supported rigid-grid model bound: every block must lie within <c>[-MaxGridCells,
    /// MaxGridCells]</c> on each axis, so an out-of-range placement is a first-class
    /// validation error rather than an arithmetic surprise downstream.
    /// </summary>
    public static class ShipTemplateModel
    {
        /// <summary>The maximum supported grid-local cell coordinate magnitude (each
        /// axis).</summary>
        public const int MaxGridCells = 10000;
    }

    /// <summary>
    /// A validating, fluent builder for an immutable <see cref="ShipTemplate"/>.
    /// <see cref="Build"/> validates the whole template (duplicate names, invalid local
    /// transforms, non-positive mass, non-cardinal fixed-thruster directions, a missing
    /// designated controller, and blocks outside the model bound), throwing an
    /// <see cref="ArgumentException"/> that names the offender. The result is reusable:
    /// entity IDs are assigned at spawn, never baked in.
    /// </summary>
    public sealed class ShipTemplateBuilder
    {
        private readonly string _name;
        private readonly float _gridSize;
        private readonly List<BlockSpec> _blocks = new List<BlockSpec>();
        private readonly List<NamedReferenceAxis> _namedAxes = new List<NamedReferenceAxis>();
        private string _designatedControllerName;
        private bool _isStatic;
        private float _maxLinearVelocity;
        private float _maxAngularVelocity;
        private bool _frozen;

        private ShipTemplateBuilder(string name, float gridSize)
        {
            _name = name;
            _gridSize = gridSize;
        }

        /// <summary>Begins a small-grid template (0.5 m cells) with the given name.</summary>
        public static ShipTemplateBuilder SmallGrid(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("A template needs a non-empty name.", nameof(name));
            }
            return new ShipTemplateBuilder(name, 0.5f);
        }

        /// <summary>Begins a large-grid template (2.5 m cells) with the given name.</summary>
        public static ShipTemplateBuilder LargeGrid(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("A template needs a non-empty name.", nameof(name));
            }
            return new ShipTemplateBuilder(name, 2.5f);
        }

        /// <summary>
        /// Begins a new builder seeded from an existing template (for deriving variants).
        /// Everything is copied so <see cref="Build"/> produces an equivalent template;
        /// the caller then mutates the copy (e.g. <see cref="RemoveBlock"/>).
        /// </summary>
        public static ShipTemplateBuilder From(ShipTemplate template)
        {
            if (template == null)
            {
                throw new ArgumentNullException(nameof(template));
            }

            var builder = new ShipTemplateBuilder(template.Name, template.GridSize)
            {
                _designatedControllerName = template.DesignatedControllerName,
                _isStatic = template.IsStatic,
                _maxLinearVelocity = template.MaxLinearVelocity,
                _maxAngularVelocity = template.MaxAngularVelocity,
            };
            foreach (TemplateBlock block in template.Blocks)
            {
                builder._blocks.Add(new BlockSpec(block));
            }
            if (template.NamedAxes != null)
            {
                foreach (NamedReferenceAxis axis in template.NamedAxes)
                {
                    builder._namedAxes.Add(new NamedReferenceAxis(axis.Name, axis.Axis));
                }
            }
            return builder;
        }

        internal string Name => _name;

        // — fluent block entries —

        /// <summary>
        /// Adds an ordinary structural block (a Standard hull block): an
        /// axis-aligned box contributing to shape and mass.
        /// </summary>
        public ShipTemplateBuilder AddStandard(string blockName, Vector3I min, float dryMass,
            Vector3I? size = null, Quaternion? orientation = null, float inventoryMass = 0f)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.Standard,
                Min = min,
                Size = size,
                DryMass = dryMass,
                InventoryMass = inventoryMass,
                Orientation = orientation,
            });
            return this;
        }

        /// <summary>
        /// Adds a ship controller and, when none is yet designated, marks it as the
        /// template's designated (primary) controller. Use <see cref="WithController"/>
        /// to choose a different one.
        /// </summary>
        public ShipTemplateBuilder AddController(string blockName, Vector3I min, float dryMass = 400f,
            Quaternion? orientation = null)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.ShipController,
                Min = min,
                DryMass = dryMass,
                Orientation = orientation,
            });
            if (_designatedControllerName == null)
            {
                _designatedControllerName = blockName;
            }
            return this;
        }

        /// <summary>Adds a gyroscope of the given maximum gyro force (N·m); its grid-local
        /// orientation gives its override axes.</summary>
        public ShipTemplateBuilder AddGyro(string blockName, Vector3I min, float gyroForce,
            float dryMass = 100f, Quaternion? orientation = null, float gyroPowerDemand = 0f)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.Gyro,
                Min = min,
                DryMass = dryMass,
                Orientation = orientation,
                GyroForce = gyroForce,
                GyroPowerDemand = gyroPowerDemand,
            });
            return this;
        }

        /// <summary>
        /// Adds <paramref name="count"/> fixed thrusters whose exhaust
        /// points along <paramref name="forward"/> (the grid is pushed
        /// opposite; force = -forward), each of the given
        /// <paramref name="maxThrust"/>, beginning at
        /// <paramref name="startMin"/> and laid out along
        /// <paramref name="forward"/>. Blocks are named <paramref name="groupName"/>
        /// when <paramref name="count"/> is 1, otherwise
        /// <c>groupName + index</c>.
        /// </summary>
        public ShipTemplateBuilder AddThrusterGroup(string groupName, Vector3I forward, int count,
            float maxThrust, Vector3I startMin = default(Vector3I), float dryMass = 100f,
            float effectiveness = 1f, float slowdownFactor = 1f, float powerDemandPer = 0f)
        {
            CheckNotFrozen();
            if (count <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(count), "A thruster group needs at least one thruster.");
            }
            for (int i = 0; i < count; i++)
            {
                string blockName = count == 1 ? groupName : groupName + i.ToString();
                _blocks.Add(new BlockSpec
                {
                    Name = blockName,
                    Kind = BlockKind.FixedThruster,
                    Min = startMin + forward * i,
                    DryMass = dryMass,
                    ThrustDirection = forward,
                    MaxThrust = maxThrust,
                    Effectiveness = effectiveness,
                    PowerDemand = powerDemandPer,
                    SlowdownFactor = slowdownFactor,
                });
            }
            return this;
        }

        /// <summary>
        /// Adds a ship connector, whose connection frame is derived from the block's
        /// grid-local orientation plus an optional extra
        /// <paramref name="connectorOrientation"/> rotation.
        /// </summary>
        public ShipTemplateBuilder AddConnector(string blockName, Vector3I min, float dryMass = 200f,
            Quaternion? orientation = null, Quaternion? connectorOrientation = null,
            float connectorReach = 0f)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.Connector,
                Min = min,
                DryMass = dryMass,
                Orientation = orientation,
                ConnectorOrientation = connectorOrientation,
                ConnectorReach = connectorReach,
            });
            return this;
        }

        /// <summary>Adds a battery power producer with the given output/capacity.</summary>
        public ShipTemplateBuilder AddBattery(string blockName, Vector3I min, float maxOutput,
            float maxInput, float maxStoredPower, float dryMass = 100f, float initialStoredPower = 1f)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.Battery,
                Min = min,
                DryMass = dryMass,
                MaxOutput = maxOutput,
                MaxInput = maxInput,
                MaxStoredPower = maxStoredPower,
                InitialStoredPower = initialStoredPower,
            });
            return this;
        }

        /// <summary>Adds a reactor power producer with the given maximum output.</summary>
        public ShipTemplateBuilder AddReactor(string blockName, Vector3I min, float maxOutput,
            float dryMass = 100f)
        {
            CheckNotFrozen();
            _blocks.Add(new BlockSpec
            {
                Name = blockName,
                Kind = BlockKind.Reactor,
                Min = min,
                DryMass = dryMass,
                MaxOutput = maxOutput,
            });
            return this;
        }

        // — configuration —

        /// <summary>Designates <paramref name="blockName"/> as the primary controller (must be
        /// present at Build).</summary>
        public ShipTemplateBuilder WithController(string blockName)
        {
            CheckNotFrozen();
            _designatedControllerName = blockName;
            return this;
        }

        /// <summary>Adds a named grid-local reference axis (e.g. a weapon reference).</summary>
        public ShipTemplateBuilder WithNamedAxis(string axisName, Vector3 axis)
        {
            CheckNotFrozen();
            _namedAxes.Add(new NamedReferenceAxis(axisName, axis));
            return this;
        }

        /// <summary>Sets the per-grid linear/angular speed caps (m/s, rad/s).</summary>
        public ShipTemplateBuilder WithSpeedLimits(float maxLinearVelocity, float maxAngularVelocity)
        {
            CheckNotFrozen();
            _maxLinearVelocity = maxLinearVelocity;
            _maxAngularVelocity = maxAngularVelocity;
            return this;
        }

        /// <summary>Marks the grid static (fixed, non-simulated).</summary>
        public ShipTemplateBuilder AsStatic()
        {
            CheckNotFrozen();
            _isStatic = true;
            return this;
        }

        /// <summary>Removes the block with the given name (for variant derivation); throws if no
        /// such block exists.</summary>
        public ShipTemplateBuilder RemoveBlock(string blockName)
        {
            CheckNotFrozen();
            for (int i = 0; i < _blocks.Count; i++)
            {
                if (string.Equals(_blocks[i].Name, blockName, StringComparison.Ordinal))
                {
                    _blocks.RemoveAt(i);
                    if (string.Equals(_designatedControllerName, blockName, StringComparison.Ordinal))
                    {
                        _designatedControllerName = null;
                    }
                    return this;
                }
            }
            throw new ArgumentException($"No block named '{blockName}' to remove.", nameof(blockName));
        }

        /// <summary>
        /// Validates the accumulated template and returns the immutable
        /// <see cref="ShipTemplate"/>. Validation failures throw an
        /// <see cref="ArgumentException"/> naming the offending
        /// block/member. Subsequent calls return an equivalent independent
        /// template (the builder remains valid to keep deriving).
        /// </summary>
        public ShipTemplate Build()
        {
            _frozen = true;
            Validate();

            var blocks = new TemplateBlock[_blocks.Count];
            for (int i = 0; i < _blocks.Count; i++)
            {
                blocks[i] = _blocks[i].ToTemplateBlock(_gridSize);
            }

            var axes = new List<NamedReferenceAxis>(_namedAxes);
            return new ShipTemplate(
                _name, _gridSize, blocks, _isStatic, _maxLinearVelocity, _maxAngularVelocity,
                _designatedControllerName, axes);
        }

        private void Validate()
        {
            if (_blocks.Count == 0)
            {
                throw new ArgumentException(
                    $"Template '{_name}' has no blocks; a rigid grid needs at least one physical block.");
            }

            var names = new HashSet<string>(StringComparer.Ordinal);
            float totalDryMass = 0f;
            BlockSpec designated = null;

            foreach (BlockSpec spec in _blocks)
            {
                // — duplicate names —
                if (!names.Add(spec.Name))
                {
                    throw new ArgumentException(
                        $"Duplicate block name '{spec.Name}' in template '{_name}' — block names must be unique.");
                }

                VerifyBlock(spec);
                totalDryMass += spec.DryMass;

                if (string.Equals(_designatedControllerName, spec.Name, StringComparison.Ordinal))
                {
                    designated = spec;
                }
            }

            // — zero / negative physical mass —
            if (!(totalDryMass > 0f))
            {
                throw new ArgumentException(
                    $"Template '{_name}' has no positive physical mass (total dry mass {totalDryMass}).");
            }

            // — missing designated controller —
            if (_designatedControllerName != null)
            {
                if (designated == null)
                {
                    throw new ArgumentException(
                        $"Template '{_name}' designates controller '{_designatedControllerName}' but no such block is present.");
                }
                BlockKind k = designated.Kind;
                if (k != BlockKind.ShipController && k != BlockKind.RemoteControl && k != BlockKind.Cockpit)
                {
                    throw new ArgumentException(
                        $"Template '{_name}' designates controller '{_designatedControllerName}' but that block is not a controller kind (it is {k}).");
                }
            }
        }

        private void VerifyBlock(BlockSpec spec)
        {
            string when = $"Block '{spec.Name}' of template '{_name}'";

            // — invalid local transforms: non-finite, degenerate orientation —
            Quaternion q = spec.Orientation ?? Quaternion.Identity;
            if (!IsFinite(q) || q.LengthSquared() < 1e-6f)
            {
                throw new ArgumentException(
                    $"{when} has an invalid local orientation (non-finite or degenerate).",
                    "orientation");
            }

            // — blocks outside the supported rigid-grid model —
            Vector3I min = spec.Min;
            Vector3I size = spec.Size ?? Vector3I.One;
            int hiX = min.X + size.X - 1, hiY = min.Y + size.Y - 1, hiZ = min.Z + size.Z - 1;
            if (!InRange(min.X, hiX) || !InRange(min.Y, hiY) || !InRange(min.Z, hiZ) ||
                size.X < 1 || size.Y < 1 || size.Z < 1)
            {
                throw new ArgumentException(
                    $"{when} lies outside the supported rigid-grid model (cells out of the " +
                    $"[{-ShipTemplateModel.MaxGridCells}, {ShipTemplateModel.MaxGridCells}] bound).",
                    "min");
            }

            // — non-positive physical mass —
            if (spec.DryMass <= 0f)
            {
                throw new ArgumentException($"{when} must have positive physical mass (got {spec.DryMass}).", "dryMass");
            }
            if (spec.InventoryMass < 0f)
            {
                throw new ArgumentException($"{when} must have non-negative inventory mass (got {spec.InventoryMass}).", "inventoryMass");
            }

            // — non-cardinal fixed-thruster directions —
            if (spec.Kind == BlockKind.FixedThruster)
            {
                Vector3I dir = spec.ThrustDirection ?? Vector3I.Zero;
                if (Math.Abs(dir.X) + Math.Abs(dir.Y) + Math.Abs(dir.Z) != 1)
                {
                    throw new ArgumentException(
                        $"{when} is a fixed thruster but its forward direction is not one of the six cardinal axes.",
                        "thrustDirection");
                }
            }
        }

        private static bool IsFinite(Quaternion q)
        {
            return IsFinite(q.X) && IsFinite(q.Y) && IsFinite(q.Z) && IsFinite(q.W);
        }

        private static bool IsFinite(float f)
        {
            return !float.IsNaN(f) && !float.IsInfinity(f);
        }

        private static bool InRange(int v, int hi)
        {
            return v >= -ShipTemplateModel.MaxGridCells && v <= ShipTemplateModel.MaxGridCells
                && hi >= -ShipTemplateModel.MaxGridCells && hi <= ShipTemplateModel.MaxGridCells;
        }

        private void CheckNotFrozen()
        {
            if (_frozen)
            {
                throw new InvalidOperationException(
                    "A builder may be re-used to derive further variants, but not mutated after Build(); " +
                    "use ShipTemplateBuilder.From(template) to seed a fresh builder for a new variant.");
            }
        }

        /// <summary>
        /// Raw, deferred description of one block: the builder records
        /// these and turns them into <see cref="TemplateBlock"/> instances
        /// in <see cref="Build"/> so that every validation rule (including
        /// non-positive mass and the model bound) is reported by Build with
        /// an <see cref="ArgumentException"/> naming the block.
        /// </summary>
        private sealed class BlockSpec
        {
            public string Name;
            public BlockKind Kind;
            public Vector3I Min;
            public Vector3I? Size;
            public float DryMass;
            public float InventoryMass;
            public Quaternion? Orientation;
            public float MaxOutput, MaxInput, MaxStoredPower, InitialStoredPower;
            public Vector3I? ThrustDirection;
            public float MaxThrust, Effectiveness, PowerDemand;
            public float GyroForce, GyroPowerDemand;
            public Quaternion? ConnectorOrientation;
            public float ConnectorReach, SlowdownFactor;

            public BlockSpec()
            {
                Effectiveness = 1f;
                InitialStoredPower = 1f;
                SlowdownFactor = 1f;
            }

            public BlockSpec(TemplateBlock b)
            {
                Name = b.Name;
                Kind = b.Kind;
                Min = b.Min;
                Size = b.Size;
                DryMass = b.DryMass;
                InventoryMass = b.InventoryMass;
                Orientation = b.Orientation;
                MaxOutput = b.MaxOutput;
                MaxInput = b.MaxInput;
                MaxStoredPower = b.MaxStoredPower;
                InitialStoredPower = b.InitialStoredPower;
                ThrustDirection = b.ThrustDirection;
                MaxThrust = b.MaxThrust;
                Effectiveness = b.Effectiveness;
                PowerDemand = b.ThrusterPowerDemand;
                GyroForce = b.GyroForce;
                GyroPowerDemand = b.GyroPowerDemand;
                ConnectorOrientation = b.ConnectorOrientation;
                ConnectorReach = b.ConnectorReach;
                SlowdownFactor = b.SlowdownFactor;
            }

            public TemplateBlock ToTemplateBlock(float gridSize)
            {
                return new TemplateBlock(
                    Name, Min, DryMass, gridSize,
                    Size, InventoryMass, Orientation, Kind,
                    MaxOutput, MaxInput, MaxStoredPower, InitialStoredPower,
                    ThrustDirection, MaxThrust, Effectiveness, PowerDemand,
                    GyroForce, GyroPowerDemand, ConnectorOrientation, ConnectorReach, SlowdownFactor);
            }
        }
    }
}
