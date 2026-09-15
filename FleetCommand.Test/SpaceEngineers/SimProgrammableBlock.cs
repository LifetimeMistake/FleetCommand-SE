using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Programmable block, implementing the real <see cref="IMyProgrammableBlock"/>
    /// (from Sandbox.ModAPI.Ingame; , ). It provides the
    /// identity a Program-composition root needs (all the
    /// <see cref="SimFunctionalBlock"/> identity) plus the explicit
    /// execution hooks the design's  asks for: tests attach the
    /// program's entry to <see cref="RunHandler"/> and invoke
    /// <see cref="TryRun"/> directly instead of a live game loop, and use
    /// <see cref="Storage"/> as the storage-facing context passed to the
    /// root. <see cref="Runtime"/> exposes the program's
    /// <see cref="SimRuntimeInfo"/>. No modeled text surface (0), and
    /// terminals helper methods are the terminal surface itself.
    /// </summary>
    public sealed class SimProgrammableBlock : SimFunctionalBlock, IMyProgrammableBlock
    {
        private bool _isRunning;

        public SimProgrammableBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            Runtime = new SimRuntimeInfo(grid.World);
        }

        /// <summary>The program's runtime information ().</summary>
        public SimRuntimeInfo Runtime { get; }

        /// <summary>
        /// The program's storage string ('s "storage" service;
        /// the game's MyGridProgram exposes <c>string Storage</c>). Not an
        /// IMyProgrammableBlock member — provided as the storage-facing
        /// context for the composition root.
        /// </summary>
        public string Storage { get; set; } = string.Empty;

        /// <summary>The default terminal run argument.</summary>
        public string TerminalRunArgument { get; set; } = string.Empty;

        /// <summary>True while the program is executing (between TryRun enter and leave).</summary>
        public bool IsRunning => _isRunning;

        /// <summary>
        /// The explicit execution hook (): the program's
        /// composition-root entry, invoked synchronously by
        /// <see cref="TryRun"/>.
        /// </summary>
        public Action<string> RunHandler { get; set; }

        /// <summary>
        /// Runs the program synchronously with <paramref name="argument"/>
        /// (the explicit-execution model,  — no live game loop). When
        /// the explicit argument is null or empty the default terminal run
        /// argument (<see cref="TerminalRunArgument"/>) is forwarded
        /// instead, matching the game's Run-with-default front ends
        /// (<c>MyProgrammableBlock.cs:552-555</c>, <c>OnRunDefaultApplied</c>).
        /// An already-running programmable block cannot be run again,
        /// matching the interface contract. Records the run on
        /// <see cref="Runtime"/> so <see cref="SimRuntimeInfo.TimeSinceLastRun"/>
        /// reflects the elapsed interval from the fixed clock.
        /// </summary>
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

        // — IMyTextSurfaceProvider (no modeled screen this stage) —

        /// <inheritdoc />
        public bool UseGenericLcd => false;

        /// <summary>No modeled text surface on a programmable block this stage.</summary>
        public int SurfaceCount => 0;

        /// <inheritdoc />
        public IMyTextSurface GetSurface(int index)
        {
            return null;
        }
    }
}
