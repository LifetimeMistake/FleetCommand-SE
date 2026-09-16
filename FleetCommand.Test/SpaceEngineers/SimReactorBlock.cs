using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Reactor block, implementing the real <see cref="IMyReactor"/> (which
    /// extends <see cref="IMyPowerProducer"/>; , ) as a
    /// second, finite power producer beside the
    /// <see cref="SimBatteryBlock"/>. This is the peer peer-producer that
    /// gives the battery something to charge from (.7): the bus
    /// sums the reactor's output into grid supply, so in <c>Recharge</c>
    /// or <c>Auto</c> a battery absorbs the reactor's surplus as charge.
    /// The reactor produces up to <see cref="MaxOutput"/> while enabled
    /// and functional (modelled always fueled — a fresh loaded core), and
    /// only as much as the grid actually draws, matching the game's
    /// producer behaviour (<c>MyReactor</c> output comes from
    /// <c>SourceComp.CurrentOutput</c>, gated on working + fuel). It has
    /// no capacity to drain, so <see cref="Draw"/> records output only.
    /// </summary>
    public sealed class SimReactorBlock : SimFunctionalBlock, IMyReactor, SimPowerSource
    {
        private float _currentOutput;

        public SimReactorBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            MaxOutput = template.MaxOutput;
            UseConveyorSystem = true; // the game's UseConveyorSystem defaults true
        }

        // — IMyReactor —

        /// <inheritdoc />
        public bool UseConveyorSystem { get; set; }

        // — IMyPowerProducer —

        /// <summary>Power delivered this tick in MW (set by the bus in phase 4).</summary>
        public float CurrentOutput => _currentOutput;

        /// <summary>Maximum output power in MW (from the template).</summary>
        public float MaxOutput { get; }

        /// <summary>
        /// Scenario-side fuel availability in [0, 1] ( "change
        /// available fuel ratio"): the multiplier on the reactor's
        /// available output (a depleted fuel rod produces proportionally
        /// less; 0 when out of fuel). Default 1.
        /// </summary>
        public float FuelRatio { get; set; } = 1f;

        /// <summary>Ratio of current output to maximum output (0 when <see cref="MaxOutput"/> is 0).</summary>
        public float CurrentOutputRatio => MaxOutput > 0f ? CurrentOutput / MaxOutput : 0f;

        // — supply behaviour (bus surface) —

        /// <summary>
        /// The maximum power this reactor can serve on one fixed step:
        /// its <see cref="MaxOutput"/> times the scenario
        /// <see cref="FuelRatio"/> while enabled and functional.
        /// </summary>
        public float AvailableOutput(float dtSeconds)
        {
            if (!Enabled || !IsFunctional)
            {
                return 0f;
            }
            float ratio = FuelRatio < 0f ? 0f : FuelRatio > 1f ? 1f : FuelRatio;
            return MaxOutput * ratio;
        }

        /// <summary>
        /// Supplies up to <paramref name="power"/> MW for this step:
        /// records the actually delivered output and returns it
        /// (bounded by <see cref="AvailableOutput"/>). The reactor has no
        /// capacity, so nothing is depleted.
        /// </summary>
        public float Draw(float power, float dtSeconds)
        {
            float cap = AvailableOutput(dtSeconds);
            float actual = power < 0f ? 0f : Math.Min(power, cap);
            _currentOutput = actual;
            return actual;
        }
    }
}
