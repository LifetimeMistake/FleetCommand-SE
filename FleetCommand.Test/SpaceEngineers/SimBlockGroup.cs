using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A named group of terminal blocks, implementing <see cref="IMyBlockGroup"/>; members are enumerated live, skipping closed blocks.</summary>
    public sealed class SimBlockGroup : IMyBlockGroup
    {
        private readonly List<SimTerminalBlock> _members = new List<SimTerminalBlock>();

        /// <inheritdoc />
        public string Name { get; }

        internal SimBlockGroup(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("A block group must have a name.", nameof(name));
            }
            Name = name;
        }

        /// <summary>Adds a block to the group (scenario setup); idempotent.</summary>
        internal void Add(SimTerminalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            if (!_members.Contains(block))
            {
                _members.Add(block);
            }
        }

        /// <inheritdoc />
        public void GetBlocks(List<IMyTerminalBlock> blocks, Func<IMyTerminalBlock, bool> collect = null)
        {
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }
            blocks.Clear();
            for (int i = 0; i < _members.Count; i++)
            {
                SimTerminalBlock block = _members[i];
                if (!block.Closed && (collect == null || collect(block)))
                {
                    blocks.Add(block);
                }
            }
        }

        /// <inheritdoc />
        public void GetBlocksOfType<T>(List<IMyTerminalBlock> blocks, Func<IMyTerminalBlock, bool> collect = null) where T : class
        {
            if (blocks == null)
            {
                throw new ArgumentNullException(nameof(blocks));
            }
            blocks.Clear();
            for (int i = 0; i < _members.Count; i++)
            {
                SimTerminalBlock block = _members[i];
                if (!block.Closed && block is T && (collect == null || collect(block)))
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
            for (int i = 0; i < _members.Count; i++)
            {
                SimTerminalBlock block = _members[i];
                T typed = block as T;
                if (!block.Closed && typed != null && (collect == null || collect(typed)))
                {
                    blocks.Add(typed);
                }
            }
        }
    }
}
