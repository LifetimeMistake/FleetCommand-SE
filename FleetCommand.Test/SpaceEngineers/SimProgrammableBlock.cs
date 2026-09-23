using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A programmable block implementing <see cref="IMyProgrammableBlock"/>: provides the
    /// identity a Program-composition root needs plus explicit execution hooks
    /// (<see cref="RunHandler"/> via <see cref="TryRun"/>) and <see cref="Storage"/> as the
    /// storage-facing context; no modeled text surface.</summary>
    public sealed class SimProgrammableBlock : SimFunctionalBlock, IMyProgrammableBlock
    {
        private bool _isRunning;

        public SimProgrammableBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            Runtime = new SimRuntimeInfo(grid.World);
        }

        /// <summary>The program's runtime information.</summary>
        public SimRuntimeInfo Runtime { get; }

        /// <summary>The program's storage string (MyGridProgram exposes <c>string Storage</c>),
        /// provided as the storage-facing context for the composition root.</summary>
        public string Storage { get; set; } = string.Empty;

        /// <summary>The default terminal run argument.</summary>
        public string TerminalRunArgument { get; set; } = string.Empty;

        /// <summary>True while the program is executing (between TryRun enter and leave).</summary>
        public bool IsRunning => _isRunning;

        /// <summary>The explicit execution hook: the program's composition-root entry, invoked
        /// synchronously by <see cref="TryRun"/>.</summary>
        public Action<string> RunHandler { get; set; }

        /// <summary>Runs the program synchronously with <paramref name="argument"/> (no live game
        /// loop); forwards <see cref="TerminalRunArgument"/> when the argument is null/empty (the
        /// game's <c>OnRunDefaultApplied</c>, MyProgrammableBlock.cs:552-555) and refuses to re-run
        /// while already running. Records the run on <see cref="Runtime"/>.</summary>
        public bool TryRun(string argument)
        {
            if (_isRunning)
            {
                return false;
            }
            _isRunning = true;
            try
            {
                Action<string> handler = RunHandler;
                if (handler != null)
                {
                    string effective = string.IsNullOrEmpty(argument) ? TerminalRunArgument : argument;
                    handler(effective ?? string.Empty);
                }
            }
            finally
            {
                _isRunning = false;
                Runtime.MarkRun();
            }
            return true;
        }

        // — IMyTextSurfaceProvider (no modeled screen) —

        /// <inheritdoc />
        public bool UseGenericLcd => false;

        /// <summary>No modeled text surface on a programmable block.</summary>
        public int SurfaceCount => 0;

        /// <inheritdoc />
        public IMyTextSurface GetSurface(int index)
        {
            return null;
        }
    }
}
