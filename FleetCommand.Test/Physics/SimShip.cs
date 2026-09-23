using FleetCommand.Test.SpaceEngineers;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// The ship handle returned by <see cref="Framework.FleetTestContext.Spawn"/>: the
    /// physics-side surface over <see cref="Grid"/>, <see cref="Body"/>,
    /// <see cref="TerminalSystem"/>, <see cref="Controller"/>, <see cref="Power"/>,
    /// and <see cref="SetState"/> for scenario setup.
    /// </summary>
    public sealed class SimShip
    {
        /// <summary>Stable numeric id, unique per world, in spawn order.</summary>
        public long Id { get; }

        /// <summary>The owning grid and its rigid body.</summary>
        public SimCubeGrid Grid { get; }

        /// <summary>Discovery surface over this grid's blocks.</summary>
        public IMyGridTerminalSystem TerminalSystem => Grid.TerminalSystem;

        /// <summary>
        /// The ship's deterministic power bus: blocks register demand + priority, batteries
        /// register as the finite supply (or the template provides an ideal source), and
        /// the per-block power ratio is delivered ahead of actuation.
        /// </summary>
        public SimPowerBus Power => Grid.PowerBus;

        /// <summary>The live rigid-body state of the ship's grid.</summary>
        public SimRigidBodyState Body => Grid.Body.GetState();

        /// <summary>The template-designated controller (first controller-kind block), or null when
        /// none.</summary>
        public IMyShipController Controller { get; }

        internal SimShip(long id, SimCubeGrid grid, IMyShipController controller)
        {
            Id = id;
            Grid = grid;
            Controller = controller;
        }

        /// <summary>Writes the ship's rigid body back from a world state (scenario setup and
        /// observation injection).</summary>
        public void SetState(in SimRigidBodyState state)
        {
            Grid.Body.SetState(state);
        }
    }
}
