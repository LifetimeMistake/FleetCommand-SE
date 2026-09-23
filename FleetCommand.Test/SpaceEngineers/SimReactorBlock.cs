using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A reactor implementing <see cref="IMyReactor"/> (extends
    /// <see cref="IMyPowerProducer"/>): produces AT FULL <see cref="MaxOutput"/> while enabled,
    /// functional, and any fuel remains, only as much as the grid actually draws (per
    /// <c>SourceComp.CurrentOutput</c>), and consumes fuel at a rate proportional to that actual
    /// output. Output is BINARY, not proportional to remaining fuel — exactly as the game's
    /// <c>MyFueledPowerProducer</c> (a reactor always produces full output while supplied;
    /// <c>CheckIsWorking</c> requires <c>IsSupplied</c> and <c>ComputeMaxProduction</c> returns
    /// the full <c>MaxOutput</c>, MyFueledPowerProducer.cs:143-151, :215-232), and fuel stops the
    /// reactor when the store empties (<c>RefreshRemainingCapacity</c> → <c>Capacity</c> →
    /// <c>CheckIsWorking</c>, MyReactor.cs:286-306).</summary>
    public sealed class SimReactorBlock : SimFunctionalBlock, IMyReactor, SimPowerSource
    {
        private float _currentOutput;
        private float _fuelRemaining;

        public SimReactorBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            MaxOutput = template.MaxOutput;
            UseConveyorSystem = true; // the game's UseConveyorSystem defaults true
            FuelConsumptionPerSecond = template.FuelConsumptionPerSecond;
            _fuelRemaining = template.InitialFuel;
        }

        // — IMyReactor —

        /// <inheritdoc />
        public bool UseConveyorSystem { get; set; }

        // — IMyPowerProducer —

        /// <summary>Power delivered this tick in MW (set by the bus in phase 4).</summary>
        public float CurrentOutput => _currentOutput;

        /// <summary>Maximum output power in MW (from the template).</summary>
        public float MaxOutput { get; }

        /// <summary>Ratio of current output to maximum output (0 when <see cref="MaxOutput"/> is
        /// 0).</summary>
        public float CurrentOutputRatio => MaxOutput > 0f ? CurrentOutput / MaxOutput : 0f;

        // — fuel store (the game's reactor inventory, MyReactor.RefreshRemainingCapacity :286-306) —

        /// <summary>Remaining fuel in item-count units (the game's uranium inventory, counted in
        /// <c>ConsumptionPerSecond_Items</c>). Any fuel above zero keeps the reactor supplied —
        /// <c>IsSupplied = Capacity &gt; 0</c> (MyFueledPowerProducer.cs:92) — and so producing
        /// at full output; fuel sets only runtime, never partial output. Clamped to [0, ∞): a
        /// negative setter means an empty store.</summary>
        public float FuelRemaining
        {
            get { return _fuelRemaining; }
            set { _fuelRemaining = value < 0f ? 0f : value; }
        }

        /// <summary>Fuel item-count units consumed per second at full output — the game's
        /// <c>ConsumptionPerSecond_Items</c> (MyReactor.cs:271). Actual consumption scales
        /// linearly with delivered output.</summary>
        public float FuelConsumptionPerSecond { get; }

        // — supply behaviour (bus surface) —

        /// <summary>The maximum power this reactor can serve on one fixed step: the full
        /// <see cref="MaxOutput"/> while enabled, functional, and fuel remains; zero when out of
        /// fuel. This is binary — a partially-full rod still delivers full output, matching
        /// <c>ComputeMaxProduction</c> (MyFueledPowerProducer.cs:225-232), never a partial
        /// fraction of <see cref="MaxOutput"/>.</summary>
        public float AvailableOutput(float dtSeconds)
        {
            if (!Enabled || !IsFunctional || _fuelRemaining <= 0f)
            {
                return 0f;
            }
            return MaxOutput;
        }

        /// <summary>Supplies up to <paramref name="power"/> MW for this step, bounded by
        /// <see cref="AvailableOutput"/>; records and returns the actually delivered output, and
        /// consumes fuel proportional to that delivered output.</summary>
        public float Draw(float power, float dtSeconds)
        {
            float cap = AvailableOutput(dtSeconds);
            float actual = power < 0f ? 0f : Math.Min(power, cap);
            _currentOutput = actual;
            ConsumeFuel(actual, dtSeconds);
            return actual;
        }

        /// <summary>Consumes fuel at a rate proportional to the actual delivered output — the game's
        /// <c>ConsumeFuel</c> (MyReactor.cs:254-284): per second <c>currentOutput /
        /// MaxPowerOutput * ConsumptionPerSecond_Items</c>, and nothing while output is zero
        /// (:260-264). When the store empties the reactor stops producing (<c>RefreshRemainingCapacity</c>
        /// → <c>Capacity</c> → <c>CheckIsWorking</c> → zero output, MyReactor.cs:286-306).</summary>
        private void ConsumeFuel(float delivered, float dtSeconds)
        {
            if (delivered <= 0f || MaxOutput <= 0f || FuelConsumptionPerSecond <= 0f)
            {
                return;
            }
            float consumed = delivered / MaxOutput * FuelConsumptionPerSecond * dtSeconds;
            if (consumed <= 0f)
            {
                return;
            }
            if (_fuelRemaining - consumed <= 0f)
            {
                _fuelRemaining = 0f; // exhausted → the reactor stops producing
            }
            else
            {
                _fuelRemaining -= consumed;
            }
        }

        /// <summary>An out-of-fuel reactor is not working, matching the game's
        /// <c>CheckIsWorking</c> requirement on <c>IsSupplied</c> (MyFueledPowerProducer.cs:143-151);
        /// phase 4 recomputes <c>IsWorking</c> after the bus draws (and consumes) fuel, so working
        /// state follows the store freshly.</summary>
        protected override bool WorkingRequirementSatisfied => _fuelRemaining > 0f;
    }
}
