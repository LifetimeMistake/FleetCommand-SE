using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A cockpit implementing <see cref="IMyCockpit"/>: extends <see cref="SimShipController"/> and has no modeled text surfaces; oxygen members throw <see cref="NotSupportedException"/>.</summary>
    public sealed class SimCockpit : SimShipController, IMyCockpit
    {
        public SimCockpit(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        // — IMyCockpit members (no oxygen model) —

        /// <inheritdoc />
        public float OxygenCapacity => ThrowNotSupported<float>("IMyCockpit.OxygenCapacity");

        /// <inheritdoc />
        public float OxygenFilledRatio => ThrowNotSupported<float>("IMyCockpit.OxygenFilledRatio");

        // — IMyTextSurfaceProvider —

        /// <inheritdoc />
        public bool UseGenericLcd => false;

        /// <summary>No modeled text surfaces on a cockpit.</summary>
        public int SurfaceCount => 0;

        /// <inheritdoc />
        public IMyTextSurface GetSurface(int index)
        {
            return null;
        }
    }
}
