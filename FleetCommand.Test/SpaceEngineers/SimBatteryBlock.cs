using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Battery block, implementing the real <see cref="IMyBatteryBlock"/>
    /// (which extends <see cref="IMyPowerProducer"/>; , )
    /// as the ship's finite power supply. The battery registers with the
    /// grid's <see cref="SimPowerBus"/>; in tick phase 4 the bus draws
    /// the delivered power from it and <see cref="CurrentStoredPower"/>
    /// (MWh) depletes by the drawn energy — <c>MW · h</c>, the same
    /// 3600-second capacity conversion the game's battery uses. The
    /// model is deterministic, not the game's live sector distributer:
    /// finite supply with depletion on draw (). Charging is not
    /// modeled this stage (no other producer exists to charge from), so
    /// <see cref="CurrentInput"/> is exactly 0 and
    /// <see cref="IsCharging"/> is false — real values, not silent
    /// defaults. <see cref="ChargeMode"/> (and the obsolete
    /// <c>OnlyRecharge</c>/<c>OnlyDischarge</c>/<c>SemiautoEnabled</c>
    /// aliases) follow the game: Recharge stops production, Discharge and
    /// Auto produce while capacity remains. An empty battery is not
    /// working (the game's <c>CheckIsWorking</c> requires capacity) and
    /// supplies nothing.
    /// </summary>
    public sealed class SimBatteryBlock : SimFunctionalBlock, IMyBatteryBlock
    {
        private const float SecondsPerHour = 3600f; // capacity conversion: MWh = MW · h

        private float _currentStoredPower;
        private float _currentOutput;
        private ChargeMode _chargeMode = ChargeMode.Auto;

        public SimBatteryBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            MaxStoredPower = template.MaxStoredPower;
            MaxOutput = template.MaxOutput;
            MaxInput = template.MaxInput;
            _currentStoredPower = ClampStored(template.MaxStoredPower * template.InitialStoredPower);
        }

        // — IMyBatteryBlock —

        /// <inheritdoc />
        public bool HasCapacityRemaining => CurrentStoredPower > 0f;

        /// <summary>
        /// Remaining stored energy in MWh. The setter clamps to
        /// <c>[0, MaxStoredPower]</c>, matching the game's
        /// <c>CurrentStoredPower</c> setter contract.
        /// </summary>
        public float CurrentStoredPower
        {
            get { return _currentStoredPower; }
            set { _currentStoredPower = ClampStored(value); }
        }

        /// <summary>Maximum stored energy in MWh (from the template).</summary>
        public float MaxStoredPower { get; }

        /// <summary>
        /// Current power input in MW. Charging is not modeled this stage —
        /// there is no other producer to charge from — so this is exactly 0.
        /// </summary>
        public float CurrentInput => 0f;

        /// <summary>Maximum charging power input in MW (from the template).</summary>
        public float MaxInput { get; }

        /// <inheritdoc />
        public bool IsCharging => CurrentInput > CurrentOutput && CurrentInput > 0f;

        /// <inheritdoc />
        public ChargeMode ChargeMode
        {
            get { return _chargeMode; }
            set { _chargeMode = value; }
        }

        /// <inheritdoc />
        [Obsolete("Use ChargeMode instead")]
        public bool OnlyRecharge
        {
            get { return ChargeMode == ChargeMode.Recharge; }
            set { ChargeMode = value ? ChargeMode.Recharge : ChargeMode.Auto; }
        }

        /// <inheritdoc />
        [Obsolete("Use ChargeMode instead")]
        public bool OnlyDischarge
        {
            get { return ChargeMode == ChargeMode.Discharge; }
            set { ChargeMode = value ? ChargeMode.Discharge : ChargeMode.Auto; }
        }

        /// <inheritdoc />
        [Obsolete("Semi-auto is no longer a valid mode, if you want to check for Auto instead, use ChargeMode")]
        public bool SemiautoEnabled
        {
            get { return ChargeMode == ChargeMode.Auto; }
            set { if (value) { ChargeMode = ChargeMode.Auto; } }
        }

        // — IMyPowerProducer —

        /// <summary>Power delivered this tick in MW (set by the bus in phase 4).</summary>
        public float CurrentOutput => _currentOutput;

        /// <summary>Maximum output power in MW (from the template).</summary>
        public float MaxOutput { get; }

        /// <summary>Ratio of current output to maximum output (0 when <see cref="MaxOutput"/> is 0).</summary>
        public float CurrentOutputRatio => MaxOutput > 0f ? CurrentOutput / MaxOutput : 0f;

        // — supply behaviour (bus surface) —

        /// <summary>
        /// The maximum power this battery can serve on one fixed step:
        /// its <see cref="MaxOutput"/> while enabled, functional, not in
        /// Recharge, and holding capacity, and never more than its
        /// remaining energy can supply over the step.
        /// </summary>
        internal float AvailableOutput(float dtSeconds)
        {
            if (!Enabled || !IsFunctional)
            {
                return 0f;
            }
            if (ChargeMode == ChargeMode.Recharge)
            {
                return 0f; // absorbing, not producing
            }
            if (!HasCapacityRemaining)
            {
                return 0f;
            }
            // Max power drawable this step without going negative:
            // stored (MWh) ÷ step (h) = MW.
            float byCapacity = _currentStoredPower * SecondsPerHour / dtSeconds;
            return Math.Min(MaxOutput, byCapacity);
        }

        /// <summary>
        /// Supplies up to <paramref name="power"/> MW for this step:
        /// records the actually delivered output and depletes stored
        /// energy by the drawn energy (<c>MW · h</c>). Returns the
        /// power actually supplied (bounded by <see cref="AvailableOutput"/>).
        /// </summary>
        internal float Draw(float power, float dtSeconds)
        {
            float cap = AvailableOutput(dtSeconds);
            float actual = power < 0f ? 0f : Math.Min(power, cap);
            _currentOutput = actual;
            float energy = actual * dtSeconds / SecondsPerHour;
            _currentStoredPower = Math.Max(0f, _currentStoredPower - energy);
            return actual;
        }

        /// <summary>
        /// An empty battery is not working, matching the game's
        /// <c>MyBatteryBlock.CheckIsWorking</c> (it requires
        /// <c>HasCapacityRemaining</c>).
        /// </summary>
        protected override bool WorkingRequirementSatisfied => HasCapacityRemaining;

        private float ClampStored(float value)
        {
            return value < 0f ? 0f : value > MaxStoredPower ? MaxStoredPower : value;
        }
    }
}
