using System;
using System.Collections.Generic;
using VRageMath;
using FleetCommand.Test.Sbc;
using System.Linq;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// The parsed-SBC → sim-block contract. A block kind that the simulator implements
    /// provides one
    /// <see cref="ISbcBlockAdapter"/> so the SBC→ShipTemplate converter can build it;
    /// the converter discovers every adapter by reflection (see
    /// <see cref="SbcAdapterRegistry"/>), so adding a block to the sim is the single
    /// step that extends the converter.
    /// </summary>
    /// <remarks>
    /// Converters live as dedicated top-level types under <c>Templates</c> named after
    /// their Sim class (e.g. <see cref="ThrustBlockAdapter"/> ↔ <c>SimThrust</c>),
    /// instead of being nested inside the Sim classes — the block's numeric facts are
    /// SBC-layer data, not runtime Sim behavior, and co-locating them in one file
    /// keeps the two concerns separate while the name still ties them together.
    /// </remarks>
    public interface ISbcBlockAdapter
    {
        /// <summary>The block subtypes this adapter converts (a single adapter may cover
        /// multiple size variants, e.g. small and large).</summary>
        IReadOnlyList<string> SubtypeIds { get; }

        /// <summary>Builds the sim block from the parsed SBC block. <paramref name="name"/>
        /// is the grid-unique block name the template assigns (adapter-naming keeps block
        /// name allocation in the converter).</summary>
        TemplateBlock Build(SbcBlock block, float gridSize, string name);
    }

    /// <summary>The semantic facts a converter needs to build a <see cref="TemplateBlock"/>:
    /// the kind plus the kind's real power/thrust parameters. Masses are in component
    /// counts × real component masses (matching the game's <c>MyCubeBlockDefinition</c>);
    /// power/thrust from the block SBCs.</summary>
    public sealed class SimBlockSpec
    {
        public BlockKind Kind;
        public float DryMass;
        public float MaxOutput;          // battery / reactor (MW)
        public float MaxInput;           // battery (MW)
        public float MaxStoredPower;     // battery (MWh)
        public float GyroForce;          // N·m
        public float GyroPowerDemand;    // MW
        public float MaxThrust;          // N
        public float ThrustPowerDemand;  // MW
    }

    /// <summary>
    /// Base for block adapters: computes the shared <see cref="TemplateBlock"/> fields
    /// (orientation from the block frame, cell size from the format-layer geometry
    /// catalog, thrust direction for thrusters) from a <see cref="SimBlockSpec"/>, so a
    /// concrete adapter only supplies its subtypes and their spec.
    /// </summary>
    public abstract class SimBlockAdapter : ISbcBlockAdapter
    {
        public abstract IReadOnlyList<string> SubtypeIds { get; }
        protected abstract SimBlockSpec SpecFor(SbcBlock block);

        public TemplateBlock Build(SbcBlock block, float gridSize, string name)
        {
            SimBlockSpec spec = SpecFor(block);
            Vector3I? size = BlockGeometryCatalog.TryGet(block.SubtypeId, out BlockGeometry geo)
                ? geo.Size
                : (Vector3I?)null;
            float stored = (float?)block.Data.Element("CurrentStoredPower") ?? spec.MaxStoredPower;
            if (stored < 0 || stored > spec.MaxStoredPower || float.IsNaN(stored) || float.IsInfinity(stored))
                throw new ArgumentException("Invalid stored battery power: " + name);
            float fuel = 0;
            foreach (var item in block.Data.Descendants("MyObjectBuilder_InventoryItem"))
                if ((string)item.Element("PhysicalContent")?.Element("SubtypeName") == "Uranium")
                    fuel += (float?)item.Element("Amount") ?? 0;
            var result = new TemplateBlock(
                name, block.Min, spec.DryMass, gridSize,
                size: size,
                orientation: Orientation(block),
                kind: spec.Kind,
                maxOutput: spec.MaxOutput,
                maxInput: spec.MaxInput,
                maxStoredPower: spec.MaxStoredPower,
                initialStoredPower: spec.MaxStoredPower > 0 ? stored / spec.MaxStoredPower : 1,
                initialFuel: spec.Kind == BlockKind.Reactor ? fuel : 0,
                // MyReactorDefinition.FuelInfo: MaxPowerOutput / 3600 / uranium mass (1 kg).
                fuelConsumptionPerSecond: spec.Kind == BlockKind.Reactor ? spec.MaxOutput / 3600f : 0,
                inventoryMass: spec.Kind == BlockKind.Reactor ? fuel : 0,
                thrustDirection: spec.Kind == BlockKind.FixedThruster ? block.Frame.Forward : (Vector3I?)null,
                maxThrust: spec.MaxThrust,
                powerDemand: spec.ThrustPowerDemand,
                gyroForce: spec.GyroForce,
                gyroPowerDemand: spec.GyroPowerDemand);
            // Definition values and runtime constants, MW; synthetic templates opt in separately.
            result.UseDefinitionPowerGroups = true;
            switch (spec.Kind)
            {
                case BlockKind.Connector:
                    result.ConnectorDetectorDummy = ConnectorDetectorGeometry.ForSubtype(block.SubtypeId);
                    result.ConnectorIsSmall = block.SubtypeId == "ConnectorSmall" || block.SubtypeId == "SmallBlockInsetConnector";
                    break;
                case BlockKind.Battery:
                    result.BatteryRechargeMultiplier = 0.8f;
                    result.BatteryUpdateIntervalTicks = 100;
                    break;
                case BlockKind.FixedThruster: result.ThrusterIdlePowerDemand = 0.000002f; break;
                case BlockKind.ProgrammableBlock: result.RequiredPowerInput = 0.0005f; break;
                case BlockKind.Cockpit: result.RequiredPowerInput = 0.003f; break;
                case BlockKind.RemoteControl: result.RequiredPowerInput = 0.01f; break;
                case BlockKind.TextPanel: result.RequiredPowerInput = 0.00004f; break;
            }
            result.InitialEnabled = (bool?)block.Data.Element("Enabled") ?? true;
            result.InitialDampeners = (bool?)block.Data.Element("DampenersEnabled") ?? true;
            string mode = (string)block.Data.Element("ChargeMode") ?? "Auto";
            result.InitialChargeMode = (Sandbox.ModAPI.Ingame.ChargeMode)Enum.Parse(typeof(Sandbox.ModAPI.Ingame.ChargeMode), mode);
            return result;
        }

        protected static Quaternion Orientation(SbcBlock b)
        {
            // Build a world rotation from the orientation's forward/up and derive the
            // quaternion, matching the game's BlockOrientation.
            Vector3 fw = b.Frame.Forward;
            Vector3 up = b.Frame.Up;
            Matrix m = Matrix.CreateWorld(Vector3.Zero, fw, up);
            return Quaternion.CreateFromRotationMatrix(m);
        }

        protected static SimBlockSpec S(BlockKind kind, float dryMass)
        {
            return new SimBlockSpec { Kind = kind, DryMass = dryMass };
        }
    }

    // — concrete adapters, one per implemented block kind —

    /// <summary>Common light/heavy armor (block + slope), small and large — all Standard.</summary>
    public sealed class StandardBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[]
        {
            "SmallBlockArmorBlock", "LargeBlockArmorBlock",
            "SmallBlockArmorSlope", "LargeBlockArmorSlope",
            "SmallHeavyBlockArmorBlock", "LargeHeavyBlockArmorBlock",
        };

        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            switch (block.SubtypeId)
            {
                case "SmallBlockArmorBlock": return S(BlockKind.Standard, 20f);
                case "LargeBlockArmorBlock": return S(BlockKind.Standard, 500f);
                case "SmallBlockArmorSlope": return S(BlockKind.Standard, 20f);
                case "LargeBlockArmorSlope": return S(BlockKind.Standard, 260f);
                case "SmallHeavyBlockArmorBlock": return S(BlockKind.Standard, 112f);
                default: return S(BlockKind.Standard, 3300f); // LargeHeavyBlockArmorBlock
            }
        }
    }

    public sealed class RemoteControlBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallBlockRemoteControl", "LargeBlockRemoteControl" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
            => block.SubtypeId == "LargeBlockRemoteControl" ? S(BlockKind.RemoteControl, 137f) : S(BlockKind.RemoteControl, 38.2f);
    }

    public sealed class ProgrammableBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallProgrammableBlock", "LargeProgrammableBlock" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
            => block.SubtypeId == "LargeProgrammableBlock" ? S(BlockKind.ProgrammableBlock, 534.4f) : S(BlockKind.ProgrammableBlock, 138.4f);
    }

    public sealed class BatteryBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallBlockSmallBatteryBlock", "LargeBlockBatteryBlock" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            if (block.SubtypeId == "LargeBlockBatteryBlock")
            {
                return new SimBlockSpec { Kind = BlockKind.Battery, DryMass = 3845f, MaxOutput = 12f, MaxInput = 12f, MaxStoredPower = 3f };
            }
            return new SimBlockSpec { Kind = BlockKind.Battery, DryMass = 146.4f, MaxOutput = 0.2f, MaxInput = 0.2f, MaxStoredPower = 0.05f };
        }
    }

    public sealed class ReactorBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallBlockSmallGenerator", "LargeBlockSmallGenerator", "LargeBlockLargeGenerator" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            switch (block.SubtypeId)
            {
                case "LargeBlockLargeGenerator": return new SimBlockSpec { Kind = BlockKind.Reactor, DryMass = 73795f, MaxOutput = 300f };
                case "LargeBlockSmallGenerator": return new SimBlockSpec { Kind = BlockKind.Reactor, DryMass = 4793f, MaxOutput = 15f };
                default: return new SimBlockSpec { Kind = BlockKind.Reactor, DryMass = 278f, MaxOutput = 0.5f };
            }
        }
    }

    public sealed class GyroBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallBlockGyro", "LargeBlockGyro" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            if (block.SubtypeId == "LargeBlockGyro")
            {
                return new SimBlockSpec { Kind = BlockKind.Gyro, DryMass = 12817f, GyroForce = 3.36e7f, GyroPowerDemand = 0.01f };
            }
            return new SimBlockSpec { Kind = BlockKind.Gyro, DryMass = 613.6f, GyroForce = 4.48e5f, GyroPowerDemand = 0.002f };
        }
    }

    public sealed class CockpitBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallBlockCockpit", "LargeBlockCockpit" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
            => block.SubtypeId == "LargeBlockCockpit" ? S(BlockKind.Cockpit, 368f) : S(BlockKind.Cockpit, 797f);
    }

    public sealed class TextPanelBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[] { "SmallLCDPanel", "LargeLCDPanel" };
        protected override SimBlockSpec SpecFor(SbcBlock block)
            => block.SubtypeId == "LargeLCDPanel" ? S(BlockKind.TextPanel, 222.2f) : S(BlockKind.TextPanel, 89.8f);
    }

    public sealed class ConnectorBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[]
        {
            "Connector", "ConnectorSmall", "ConnectorMedium",
            "SmallBlockInsetConnector", "LargeBlockInsetConnector",
        };
        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            switch (block.SubtypeId)
            {
                case "Connector":
                case "LargeBlockInsetConnector": return S(BlockKind.Connector, 3564f);
                case "ConnectorSmall":
                case "SmallBlockInsetConnector": return S(BlockKind.Connector, 204.8f);
                default: return S(BlockKind.Connector, 685.2f); // ConnectorMedium
            }
        }
    }

    public sealed class ThrustBlockAdapter : SimBlockAdapter
    {
        public override IReadOnlyList<string> SubtypeIds => new[]
        {
            "SmallBlockSmallThrust", "LargeBlockSmallThrust",
            "SmallBlockLargeThrust", "LargeBlockLargeThrust",
        };
        protected override SimBlockSpec SpecFor(SbcBlock block)
        {
            switch (block.SubtypeId)
            {
                case "LargeBlockSmallThrust": return new SimBlockSpec { Kind = BlockKind.FixedThruster, DryMass = 4380f, MaxThrust = 3.456e5f, ThrustPowerDemand = 3.36f };
                case "SmallBlockLargeThrust": return new SimBlockSpec { Kind = BlockKind.FixedThruster, DryMass = 721f, MaxThrust = 1.728e5f, ThrustPowerDemand = 2.4f };
                case "LargeBlockLargeThrust": return new SimBlockSpec { Kind = BlockKind.FixedThruster, DryMass = 43200f, MaxThrust = 4.32e6f, ThrustPowerDemand = 33.6f };
                default: return new SimBlockSpec { Kind = BlockKind.FixedThruster, DryMass = 121f, MaxThrust = 1.44e4f, ThrustPowerDemand = 0.2f };
            }
        }
    }
}
