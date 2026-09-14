using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Cockpit block, implementing the real <see cref="IMyCockpit"/> (from
    /// Sandbox.ModAPI.Ingame; , ). It extends
    /// <see cref="SimShipController"/> (so it provides the full
    /// ship-controller behavior and identity needed by
    /// discovery/configuration) and reports no modeled text surfaces.
    /// Life-support (oxygen) is not modeled: those members throw a naming
    /// <see cref="NotSupportedException"/>.
    /// </summary>
    public sealed class SimCockpit : SimShipController, IMyCockpit
    {
        public SimCockpit(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        // — IMyCockpit members outside the  contract (no oxygen model) —

        /// <inheritdoc />
        public float OxygenCapacity => ThrowNotSupported<float>("IMyCockpit.OxygenCapacity");

        /// <inheritdoc />
        public float OxygenFilledRatio => ThrowNotSupported<float>("IMyCockpit.OxygenFilledRatio");

        // — IMyTextSurfaceProvider —

        /// <inheritdoc />
        public bool UseGenericLcd => false;

        /// <summary>No modeled text surfaces on a cockpit this stage.</summary>
        public int SurfaceCount => 0;

        /// <inheritdoc />
        public IMyTextSurface GetSurface(int index)
        {
            return null;
        }
    }
}
