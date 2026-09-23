using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The grid terminal system for one rigid grid, implementing
    /// <see cref="IMyGridTerminalSystem"/>: exact-name lookup, typed enumeration, predicate
    /// filtering, id lookup, access checks, and block groups. All enumeration works over the grid's
    /// blocks in deterministic spawn order, never returns blocks from other grids (same-grid
    /// filtering), and skips closed (disposed) blocks, so a grid whose blocks were removed
    /// mid-world stays safe to enumerate.</summary>
    public sealed class SimGridTerminalSystem : IMyGridTerminalSystem
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimTerminalBlock> _blocks = new List<SimTerminalBlock>();
        private readonly List<SimBlockGroup> _groups = new List<SimBlockGroup>();

        internal SimGridTerminalSystem(SimCubeGrid grid)
        {
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
        }

        /// <summary>The grid's blocks in deterministic spawn order (sim surface).</summary>
        public IReadOnlyList<SimTerminalBlock> Blocks => _blocks;

        /// <summary>The owning grid.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>Registers a spawned block with this terminal system.</summary>
        internal void RegisterBlock(SimTerminalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            _blocks.Add(block);
        }

        /// <summary>Removes a block from this terminal system (scenario block removal).</summary>
        internal void DeregisterBlock(SimTerminalBlock block)
        {
            if (block != null)
            {
                _blocks.Remove(block);
            }
        }

        // — block groups (fixtures) —

        /// <summary>Creates a named, empty block group and registers it (sim surface for
        /// fixtures).</summary>
        public SimBlockGroup CreateGroup(string name)
        {
            return CreateGroup(name, null);
        }

        /// <summary>Creates a named block group containing <paramref name="members"/> and registers
        /// it (sim surface for fixtures).</summary>
        public SimBlockGroup CreateGroup(string name, IEnumerable<SimTerminalBlock> members)
        {
            SimBlockGroup group = new SimBlockGroup(name);
            if (members != null)
            {
                foreach (SimTerminalBlock block in members)
                {
                    group.Add(block);
                }
            }
            _groups.Add(group);
            return group;
        }

        // — IMyGridTerminalSystem —

        /// <inheritdoc />
        public void GetBlocks(List<IMyTerminalBlock> blocks)
        {
            RequireList(blocks);
            blocks.Clear();
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                if (!block.Closed)
                {
                    blocks.Add(block);
                }
            }
        }

        /// <inheritdoc />
        public void GetBlockGroups(List<IMyBlockGroup> blockGroups, Func<IMyBlockGroup, bool> collect = null)
        {
            if (blockGroups == null)
            {
                throw new ArgumentNullException(nameof(blockGroups));
            }
            blockGroups.Clear();
            for (int i = 0; i < _groups.Count; i++)
            {
                SimBlockGroup group = _groups[i];
                if (collect == null || collect(group))
                {
                    blockGroups.Add(group);
                }
            }
        }

        /// <inheritdoc />
        public void GetBlocksOfType<T>(List<IMyTerminalBlock> blocks, Func<IMyTerminalBlock, bool> collect = null) where T : class
        {
            RequireList(blocks);
            blocks.Clear();
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                if (block.Closed || !(block is T))
                {
                    continue;
                }
                if (collect == null || collect(block))
                {
                    blocks.Add(block);
                }
            }
        }

        /// <inheritdoc />
        public void GetBlocksOfType<T>(List<T> blocks, Func<T, bool> collect = null) where T : class
        {
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }
            blocks.Clear();
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                T typed = block as T;
                if (block.Closed || typed == null)
                {
                    continue;
                }
                if (collect == null || collect(typed))
                {
                    blocks.Add(typed);
                }
            }
        }

        /// <inheritdoc />
        public void SearchBlocksOfName(string name, List<IMyTerminalBlock> blocks, Func<IMyTerminalBlock, bool> collect = null)
        {
            RequireList(blocks);
            blocks.Clear();
            if (name == null)
            {
                return;
            }
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                if (block.Closed || block.CustomName == null)
                {
                    continue;
                }
                if (block.CustomName.IndexOf(name, StringComparison.OrdinalIgnoreCase) >= 0
                    && (collect == null || collect(block)))
                {
                    blocks.Add(block);
                }
            }
        }

        /// <inheritdoc />
        public IMyTerminalBlock GetBlockWithName(string name)
        {
            if (name == null)
            {
                return null;
            }
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                if (!block.Closed
                    && string.Equals(block.CustomName, name, StringComparison.OrdinalIgnoreCase))
                {
                    return block;
                }
            }
            return null;
        }

        /// <inheritdoc />
        public IMyBlockGroup GetBlockGroupWithName(string name)
        {
            if (name == null)
            {
                return null;
            }
            for (int i = 0; i < _groups.Count; i++)
            {
                SimBlockGroup group = _groups[i];
                if (string.Equals(group.Name, name, StringComparison.OrdinalIgnoreCase))
                {
                    return group;
                }
            }
            return null;
        }

        /// <inheritdoc />
        public IMyTerminalBlock GetBlockWithId(long id)
        {
            for (int i = 0; i < _blocks.Count; i++)
            {
                SimTerminalBlock block = _blocks[i];
                if (!block.Closed && block.EntityId == id)
                {
                    return block;
                }
            }
            return null;
        }

        /// <inheritdoc />
        public bool CanAccess(IMyTerminalBlock block, MyTerminalAccessScope scope = MyTerminalAccessScope.All)
        {
            if (block == null || block is SimTerminalBlock == false)
            {
                return false;
            }
            SimTerminalBlock candidate = (SimTerminalBlock)block;
            // Single rigid grid: all scope values resolve to "same grid".
            return !candidate.Closed && candidate.Grid == _grid;
        }

        /// <inheritdoc />
        public bool CanAccess(IMyCubeGrid grid, MyTerminalAccessScope scope = MyTerminalAccessScope.All)
        {
            if (grid == null || grid is SimCubeGrid == false)
            {
                return false;
            }
            SimCubeGrid candidate = (SimCubeGrid)grid;
            // Single rigid grid: all scope values resolve to "same grid".
            return !candidate.Closed && candidate == _grid;
        }

        private static void RequireList(List<IMyTerminalBlock> blocks)
        {
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }
        }
    }
}
