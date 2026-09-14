using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The rigid grid: owns its <see cref="SimRigidBody"/> and its blocks
    /// (, ), and implements the real <see cref="IMyCubeGrid"/>
    /// from VRage.Game plus its <see cref="IMyEntity"/> parent (,
    /// ). The grid's world transform is owned by the body; block
    /// membership, position queries, and same-grid / construct identity are
    /// backed by this grid's block list. This stage inverts the scene:
    /// <c>SimCubeGrid</c> is the grid that owns blocks.
    /// </summary>
    public sealed class SimCubeGrid : SimEntity, IMyCubeGrid
    {
        private readonly Dictionary<Vector3I, SimTerminalBlock> _blockByPosition = new Dictionary<Vector3I, SimTerminalBlock>();

        /// <summary>The rigid body this grid owns (one per dynamic grid).</summary>
        public SimRigidBody Body { get; }

        /// <summary>The world that owns this grid (gravity source, tick clock, …).  reads it for controller gravity.</summary>
        public SimWorld World { get; }

        /// <summary>Grid cell size in meters.</summary>
        public float GridSize { get; }

        /// <summary>The grid terminal system used to discover this grid's blocks.</summary>
        public SimGridTerminalSystem TerminalSystem { get; }

        /// <summary>Also the grid's user-visible terminal name (settable).</summary>
        public string CustomName { get; set; }

        /// <summary>World-visible position of the grid origin (from the body).</summary>
        public Vector3D Position => Body.PositionWorld;

        public SimCubeGrid(long entityId, string name, float gridSize, SimRigidBody body, SimWorld world)
            : base(entityId, name)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body));
            }
            if (world == null)
            {
                throw new ArgumentNullException(nameof(world));
            }
            Body = body;
            World = world;
            GridSize = gridSize;
            CustomName = name;
            TerminalSystem = new SimGridTerminalSystem(this);
        }

        /// <inheritdoc />
        public override MatrixD WorldMatrix => Body.WorldMatrix;

        /// <inheritdoc />
        public override string DisplayName => CustomName;

        // — IMyCubeGrid —

        /// <inheritdoc />
        public MyCubeSize GridSizeEnum => GridSize <= 1.5f ? MyCubeSize.Small : MyCubeSize.Large;

        /// <inheritdoc />
        public bool IsStatic => !Body.IsDynamic;

        /// <inheritdoc />
        public Vector3 LinearVelocity => (Vector3)Body.LinearVelocity;

        /// <inheritdoc />
        public float Speed => (float)Body.LinearVelocity.Length();

        /// <summary>Grid cell bounds: the minimum block cell across all blocks.</summary>
        public Vector3I Min
        {
            get
            {
                Vector3I? min = null;
                IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
                for (int i = 0; i < blocks.Count; i++)
                {
                    min = min.HasValue ? MinComponents(min.Value, blocks[i].Min) : blocks[i].Min;
                }
                return min ?? Vector3I.Zero;
            }
        }

        /// <summary>Grid cell bounds: the maximum block cell across all blocks.</summary>
        public Vector3I Max
        {
            get
            {
                Vector3I? max = null;
                IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
                for (int i = 0; i < blocks.Count; i++)
                {
                    max = max.HasValue ? MaxComponents(max.Value, blocks[i].Max) : blocks[i].Max;
                }
                return max ?? Vector3I.Zero;
            }
        }

        /// <inheritdoc />
        public bool CubeExists(Vector3I pos)
        {
            return _blockByPosition.ContainsKey(pos);
        }

        /// <inheritdoc />
        public IMySlimBlock GetCubeBlock(Vector3I pos)
        {
            SimTerminalBlock block;
            if (_blockByPosition.TryGetValue(pos, out block))
            {
                return block.Closed ? null : new SimSlimBlock(block);
            }
            return null;
        }

        /// <inheritdoc />
        public Vector3D GridIntegerToWorld(Vector3I gridCoords)
        {
            // Grid cell origin transformed by the grid's world matrix —
            // the game's grid-coordinate to world-space conversion.
            Vector3D local = new Vector3D(gridCoords.X, gridCoords.Y, gridCoords.Z) * GridSize;
            return Vector3D.Transform(local, WorldMatrix);
        }

        /// <inheritdoc />
        public Vector3I WorldToGridInteger(Vector3D coords)
        {
            MatrixD world = WorldMatrix;
            MatrixD inverse;
            MatrixD.Invert(ref world, out inverse);
            Vector3D local = Vector3D.Transform(coords, inverse) / (double)GridSize;
            return Vector3I.Round(local);
        }

        /// <inheritdoc />
        public bool IsSameConstructAs(IMyCubeGrid other)
        {
            // For a single rigid grid, "same construct" equals "same grid"
            // (); there are no rotors or pistons to follow yet.
            return other != null && other.EntityId == EntityId;
        }

        // — block membership (sim surface) —

        /// <summary>Registers a spawned block with this grid and its terminal system.</summary>
        internal void AddBlock(SimTerminalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            _blockByPosition.Add(block.Position, block);
            TerminalSystem.RegisterBlock(block);
        }

        /// <summary>
        /// Disposes the grid's owned body and closes all its blocks.
        /// Idempotent; subsequent enumeration over the terminal system and
        /// the world's tick continue to work (closed items are skipped).
        /// </summary>
        public void Dispose()
        {
            if (IsDisposed)
            {
                return;
            }
            IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
            for (int i = 0; i < blocks.Count; i++)
            {
                blocks[i].Dispose();
            }
            if (Body != null)
            {
                Body.Dispose();
            }
            IsDisposed = true;
        }

        private static Vector3I MinComponents(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                Math.Min(a.X, b.X),
                Math.Min(a.Y, b.Y),
                Math.Min(a.Z, b.Z));
        }

        private static Vector3I MaxComponents(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                Math.Max(a.X, b.X),
                Math.Max(a.Y, b.Y),
                Math.Max(a.Z, b.Z));
        }
    }
}
