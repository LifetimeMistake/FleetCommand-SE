using FleetCommand.Test.SpaceEngineers;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// The ship handle returned by
    /// <see cref="Framework.FleetTestContext.Spawn"/> (). This
    /// stage provides the physics-side surface: a stable <see cref="Id"/>
    /// (numeric, ascending in spawn order), the owning <see cref="Grid"/>,
    /// the live rigid-body <see cref="Body"/> state, and
    /// <see cref="SetState"/> for scenario setup and ground-truth
    /// injection. The terminal-system / controller / power members
    /// (<c>TerminalSystem</c>, <c>Controller</c>, <c>Power</c>) belong to
    /// Stages 4–6 and are not present yet.
    /// </summary>
    public sealed class SimShip
    {
        /// <summary>Stable numeric id, unique per world, in spawn order.</summary>
        public long Id { get; }

        /// <summary>The owning grid and its rigid body.</summary>
        public SimCubeGrid Grid { get; }

        /// <summary>The live rigid-body state of the ship's grid.</summary>
        public SimRigidBodyState Body => Grid.Body.GetState();

        internal SimShip(long id, SimCubeGrid grid)
        {
            Id = id;
            Grid = grid;
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
