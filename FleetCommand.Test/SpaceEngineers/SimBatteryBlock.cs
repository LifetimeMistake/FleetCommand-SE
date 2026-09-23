using System;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>Battery block implementing <see cref="IMyBatteryBlock"/> (which extends
    /// <see cref="IMyPowerProducer"/>) as the ship's finite and reversible power supply: registers
    /// with the grid's <see cref="SimPowerBus"/> both as a supplier (phase 4 draws its delivered
    /// output) and, in <c>Recharge</c> / <c>Auto</c>, as a charge sink that absorbs a peer
    /// producer's surplus. Reproduces the game's charge model (<c>MyBatteryBlock</c>):
    /// <c>Recharge</c> disables production and stores grid input; <c>Auto</c> <c>TransferPower</c>s
    /// (store the surplus or consume the deficit); <c>Discharge</c> only consumes its own output.
    /// <see cref="CurrentInput"/> is what the bus delivered and <see cref="IsCharging"/> follows
    /// the game's exact test <c>CurrentInput &gt; CurrentOutput &amp;&amp; CurrentInput &gt; 0</c>
    /// (MyBatteryBlock.cs :235-245). Stored energy changes only via the per-tick net charge path
    /// (<c>StorePower</c> / <c>ConsumePower</c>, :756-816) with the same <c>MW · h</c> conversion
    /// (<c>ProductionToCapacityMultiplier = 3600</c>) the game uses. An empty battery is not
    /// working (the game's <c>CheckIsWorking</c> requires capacity) but can still absorb charge,
    /// restoring working state as it fills.</summary>
    public sealed class SimBatteryBlock : SimFunctionalBlock, IMyBatteryBlock, SimPowerSource
    {
        private const float SecondsPerHour = 3600f; // capacity conversion: MWh = MW · h (ProductionToCapacityMultiplier)
        private const float ChargeSinkAggression = 0.6f; // MyBatteryBlock.cs:551 (60f / 100f)

        private float _currentStoredPower;
        private float _currentOutput;
        private float _currentInput;
        private ChargeMode _chargeMode = ChargeMode.Auto;

        public SimBatteryBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            MaxStoredPower = template.MaxStoredPower;
            MaxOutput = template.MaxOutput;
            MaxInput = template.MaxInput;
            _currentStoredPower = ClampStored(template.MaxStoredPower * template.InitialStoredPower);
        }

        /// <summary>Battery charge efficiency multiplier (the game's
        /// <c>BlockDefinition.RechargeMultiplier</c>, applied in <c>StorePower</c>); default 1 (the
        /// game default).</summary>
        public float RechargeMultiplier { get; set; } = 1f;

        // — IMyBatteryBlock —

        /// <inheritdoc />
        public bool HasCapacityRemaining => CurrentStoredPower > 0f;

        /// <summary>Remaining stored energy in MWh. The setter clamps to [0, MaxStoredPower],
        /// matching the game's <c>CurrentStoredPower</c> setter contract.</summary>
        public float CurrentStoredPower
        {
            get { return _currentStoredPower; }
            set { _currentStoredPower = ClampStored(value); }
        }

        /// <summary>Maximum stored energy in MWh (from the template).</summary>
        public float MaxStoredPower { get; }

        /// <summary>Current power input in MW: the power the grid actually delivered to this
        /// battery's charge sink this tick (set by the bus's charge path in phase 4); 0 when the
        /// battery is not drawing charge (<c>Discharge</c>, full, or no surplus).</summary>
        public float CurrentInput => _currentInput;

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

        /// <summary>Ratio of current output to maximum output (0 when <see cref="MaxOutput"/> is
        /// 0).</summary>
        public float CurrentOutputRatio => MaxOutput > 0f ? CurrentOutput / MaxOutput : 0f;

        // — supply behaviour (bus surface) —

        /// <summary>The maximum power this battery can serve on one fixed step: its
        /// <see cref="MaxOutput"/> while enabled, functional, not in Recharge, and holding
        /// capacity, and never more than its remaining energy can supply over the step.</summary>
        public float AvailableOutput(float dtSeconds)
        {
            if (!Enabled || !IsFunctional)
            {
                return 0f;
            }
            if (ChargeMode == ChargeMode.Recharge)
            {
                return 0f; // absorbing, not producing (:621 StorePower only)
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

        /// <summary>Supplies up to <paramref name="power"/> MW for this step: records the actually
        /// delivered output and returns the power actually supplied (bounded by
        /// <see cref="AvailableOutput"/>). Output here is a report; the stored-power change it
        /// represents is applied by the per-tick net charge path
        /// (<see cref="ApplyCharge(float, float)"/>, the game's
        /// <c>TransferPower</c>/<c>ConsumePower</c>), so the battery's net energy always equals
        /// <c>input − output</c> — never a double count.</summary>
        public float Draw(float power, float dtSeconds)
        {
            float cap = AvailableOutput(dtSeconds);
            float actual = power < 0f ? 0f : Math.Min(power, cap);
            _currentOutput = actual;
            return actual;
        }

        // — charge behaviour (bus surface) —

        /// <summary>The charge power this battery wants to draw this tick — the game's
        /// <c>Sink_ComputeRequiredPower</c> (MyBatteryBlock.cs :547-560): the smaller of
        /// <c>(MaxStored − Current) · 0.6 · 3600 + CurrentOutput</c> (an aggressive taper
        /// approaching the incoming output as it fills) and <see cref="MaxInput"/>, active only
        /// while enabled, functional, not full, and not in
        /// <see cref="ChargeMode.Discharge"/>.</summary>
        internal float ComputeChargeDemand()
        {
            if (!Enabled || !IsFunctional)
            {
                return 0f;
            }
            if (ChargeMode == ChargeMode.Discharge)
            {
                return 0f; // Discharge never absorbs input (:624 Cleanup)
            }
            if (_currentStoredPower >= MaxStoredPower)
            {
                return 0f; // full (m_isFull): no more input wanted
            }
            float gapDemand = (MaxStoredPower - _currentStoredPower) * ChargeSinkAggression * SecondsPerHour
                + _currentOutput;
            float result = Math.Min(gapDemand, MaxInput);
            return result < 0f ? 0f : result;
        }

        /// <summary>Applies the delivered charge input for this tick and nets it against the
        /// battery's output exactly as the game does (<c>UpdateInternal</c> :613-626). Sets
        /// <see cref="CurrentInput"/> to the delivered power, then: <c>Recharge</c> →
        /// <c>StorePower(input)</c>; <c>Auto</c> → <c>TransferPower(input, output)</c> (store input
        /// − output if surplus, consume output − input if deficit); <c>Discharge</c> →
        /// <c>ConsumePower(output)</c>. Stored energy is changed only here, so bus delivery and
        /// stored power never diverge.</summary>
        internal void ApplyCharge(float input, float dtSeconds)
        {
            _currentInput = input;
            switch (ChargeMode)
            {
                case ChargeMode.Recharge:
                    StorePower(input, dtSeconds); // output is 0 in Recharge
                    break;
                case ChargeMode.Auto:
                    float net = input - _currentOutput;
                    if (net < 0f)
                    {
                        ConsumePower(-net, dtSeconds);
                    }
                    else if (net > 0f)
                    {
                        StorePower(net, dtSeconds);
                    }
                    break;
                default: // Discharge: consume own output, never charge
                    ConsumePower(_currentOutput, dtSeconds);
                    break;
            }
        }

        /// <summary>Charges <c>input</c> MW into stored power this step, toward
        /// <see cref="MaxStoredPower"/> (capped there), reproducing
        /// <c>MyBatteryBlock.StorePower</c> :756-775 with the <c>RechargeMultiplier</c>
        /// applied.</summary>
        private void StorePower(float input, float dtSeconds)
        {
            if (input <= 0f)
            {
                return;
            }
            float charge = input * dtSeconds / SecondsPerHour * RechargeMultiplier;
            if (charge <= 0f)
            {
                return;
            }
            if (_currentStoredPower + charge < MaxStoredPower)
            {
                _currentStoredPower += charge;
            }
            else
            {
                _currentStoredPower = MaxStoredPower;
            }
        }

        /// <summary>Drains <c>output</c> MW from stored power this step, to zero, reproducing
        /// <c>MyBatteryBlock.ConsumePower</c> :784-816.</summary>
        private void ConsumePower(float output, float dtSeconds)
        {
            if (output <= 0f)
            {
                return;
            }
            float energy = output * dtSeconds / SecondsPerHour;
            if (energy <= 0f)
            {
                return;
            }
            if (_currentStoredPower - energy <= 0f)
            {
                _currentStoredPower = 0f;
            }
            else
            {
                _currentStoredPower -= energy;
            }
        }

        /// <summary>An empty battery is not working, matching the game's
        /// <c>MyBatteryBlock.CheckIsWorking</c> (it requires
        /// <c>HasCapacityRemaining</c>).</summary>
        protected override bool WorkingRequirementSatisfied => HasCapacityRemaining;

        private float ClampStored(float value)
        {
            return value < 0f ? 0f : value > MaxStoredPower ? MaxStoredPower : value;
        }
    }
}
