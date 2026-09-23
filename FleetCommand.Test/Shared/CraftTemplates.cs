using FleetCommand.Test.Templates;
using VRageMath;

namespace FleetCommand.Test.Shared
{
    /// <summary>
    /// Shared ship-template helpers used by multiple conformance fixtures.
    /// This is a test-only fixture location (not FleetCommand.Common).
    /// </summary>
    public static class CraftTemplates
    {
        /// <summary>
        /// A battery + heavy hull + one driver block, exposing a
        /// controllable power-bus shortfall via <paramref name="batteryMaxOutput"/>.
        /// The <paramref name="driver"/> block (thruster, gyro, …) sits at
        /// the standard cell (4,0,0). Used by the thruster and gyro
        /// power-shortfall tests to build a half-power and a full-power
        /// twin from the same driver block.
        /// </summary>
        public static ShipTemplate PoweredDriverCraft(string name, float batteryMaxOutput, float batteryStored, TemplateBlock driver)
        {
            return new ShipTemplate(name, 2.5f, new[]
            {
                new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 2.5f,
                    kind: BlockKind.Battery, maxOutput: batteryMaxOutput, maxStoredPower: batteryStored),
                new TemplateBlock("Hull", new Vector3I(2, 0, 0), 2000f, 2.5f),
                driver,
            });
        }
    }
}
