using FleetCommand.Test.SpaceEngineers;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// The ship handle returned by
    /// <see cref="Framework.FleetTestContext.Spawn"/> (). This
    /// provides the physics-side surface: a stable <see cref="Id"/>
    /// (numeric, ascending in spawn order), the owning <see cref="Grid"/>,
    /// the live rigid-body <see cref="Body"/> state,
    /// <see cref="TerminalSystem"/> (discovery over the grid's blocks,
    /// ), the template-designated <see cref="Controller"/>
    /// (), the deterministic <see cref="Power"/> bus (),
    /// and <see cref="SetState"/> for scenario setup and ground-truth
    /// injection.
    /// </summary>
    public sealed class SimShip
    {
        /// <summary>Stable numeric id, unique per world, in spawn order.</summary>
        public long Id { get; }

        /// <summary>The owning grid and its rigid body.</summary>
        public SimCubeGrid Grid { get; }

        /// <summary>Discovery surface over this grid's blocks ().</summary>
        public IMyGridTerminalSystem TerminalSystem => Grid.TerminalSystem;

        /// <summary>
        /// The ship's deterministic power bus (): functional
        /// blocks register demand + priority, batteries register as the
        /// finite supply (or the template provides an ideal source), and
        /// phase 4 delivers the per-block power ratio ahead of actuation
        /// (, ).
        /// </summary>
        public SimPowerBus Power => Grid.PowerBus;

        /// <summary>The live rigid-body state of the ship's grid.</summary>
        public SimRigidBodyState Body => Grid.Body.GetState();

        /// <summary>
        /// The controller the template designates (the first
        /// controller-kind block), or null when the template has none
        /// (; ).
        /// </summary>
        public IMyShipController Controller { get; }

        internal SimShip(long id, SimCubeGrid grid, IMyShipController controller)
        {
            Id = id;
            Grid = grid;
            Controller = controller;
        }

        /// <summary>
        /// Writes the ship's rigid body back from a world state (scenario
        /// setup and observation injection, ).
        /// </summary>
        public void SetState(in SimRigidBodyState state)
        {
            Grid.Body.SetState(state);
        }
    }
}
