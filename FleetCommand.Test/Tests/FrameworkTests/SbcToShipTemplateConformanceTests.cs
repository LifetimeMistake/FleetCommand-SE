using System;
using System.IO;
using NUnit.Framework;
using VRageMath;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.Templates;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Exercises Layer 2 (SbcToShipTemplate): mapping an already-valid SBC into
    /// a ShipTemplate, and rejecting blocks the simulator does not implement. Pairs with
    /// <see cref="SbcReaderWriterConformanceTests"/> (Layer 1, format + structural
    /// legality).</summary>
    [TestFixture]
    public class SbcToShipTemplateConformanceTests
    {
        private static readonly SbcFileReader Reader = new SbcFileReader();

        // — small legal craft: armor(0,0,0), armor(1,0,0), gyro(0,1,0), thruster(2,0,0) —
        private static string LegalCraft()
        {
            return "<?xml version=\"1.0\"?>\n"
                 + "<Definitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n"
                 + "  <ShipBlueprints>\n"
                 + "    <ShipBlueprint xsi:type=\"MyObjectBuilder_ShipBlueprintDefinition\">\n"
                 + "      <Id Type=\"MyObjectBuilder_ShipBlueprintDefinition\" Subtype=\"legal\" />\n"
                 + "      <CubeGrids><CubeGrid>\n"
                 + "        <GridSizeEnum>Small</GridSizeEnum>\n"
                 + "        <CubeBlocks>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_CubeBlock\"><SubtypeName>SmallBlockArmorBlock</SubtypeName><Min x=\"0\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_CubeBlock\"><SubtypeName>SmallBlockArmorBlock</SubtypeName><Min x=\"1\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_Gyro\"><SubtypeName>SmallBlockGyro</SubtypeName><Min x=\"0\" y=\"1\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_Thrust\"><SubtypeName>SmallBlockSmallThrust</SubtypeName><Min x=\"2\" y=\"0\" z=\"0\" /><BlockOrientation Forward=\"Right\" Up=\"Up\" /></MyObjectBuilder_CubeBlock>\n"
                 + "        </CubeBlocks>\n"
                 + "      </CubeGrid></CubeGrids>\n"
                 + "    </ShipBlueprint>\n"
                 + "  </ShipBlueprints>\n"
                 + "</Definitions>\n";
        }

        // A format-legal grid whose second block is a subtype the sim doesn't implement
        // (falls back to an all-face 1x1x1 geometry so Layer 1 accepts it; Layer 2 must
        // reject it).
        private static string UnknownBlockCraft()
        {
            return "<?xml version=\"1.0\"?>\n"
                 + "<Definitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n"
                 + "  <ShipBlueprints>\n"
                 + "    <ShipBlueprint xsi:type=\"MyObjectBuilder_ShipBlueprintDefinition\">\n"
                 + "      <Id Type=\"MyObjectBuilder_ShipBlueprintDefinition\" Subtype=\"unknown\" />\n"
                 + "      <CubeGrids><CubeGrid>\n"
                 + "        <GridSizeEnum>Small</GridSizeEnum>\n"
                 + "        <CubeBlocks>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_CubeBlock\"><SubtypeName>SmallBlockArmorBlock</SubtypeName><Min x=\"0\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_Drill\"><SubtypeName>SmallBlockDrill</SubtypeName><Min x=\"1\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "        </CubeBlocks>\n"
                 + "      </CubeGrid></CubeGrids>\n"
                 + "    </ShipBlueprint>\n"
                 + "  </ShipBlueprints>\n"
                 + "</Definitions>\n";
        }

        [Test]
        public void LegalCraft_ConvertsToShipTemplate()
        {
            SbcFile file = Reader.Read(LegalCraft());
            ShipTemplate template = SbcToShipTemplate.Convert(file, "legal");

            Assert.That(template.Blocks.Count, Is.EqualTo(4));
            Assert.That(template.Blocks[0].Kind, Is.EqualTo(BlockKind.Standard));
            Assert.That(template.Blocks[2].Kind, Is.EqualTo(BlockKind.Gyro));

            TemplateBlock thruster = template.Blocks[3];
            Assert.That(thruster.Kind, Is.EqualTo(BlockKind.FixedThruster));
            Assert.That(thruster.ThrustDirection, Is.EqualTo(new Vector3I(1, 0, 0))); // Forward=Right
            Assert.That(thruster.MaxThrust, Is.EqualTo(14400f).Within(1f));           // real small thrust
            Assert.That(thruster.DryMass, Is.EqualTo(121f));                          // real small mass
        }

        [Test]
        public void UnknownBlock_IsRejectedOnConvert()
        {
            SbcFile file = Reader.Read(UnknownBlockCraft());
            var ex = Assert.Throws<ArgumentException>((Action)(() => SbcToShipTemplate.Convert(file, "unknown")));
            StringAssertContains(ex, "not implemented");
        }

        // — reflection discovery: each implemented kind resolves via SbcAdapterRegistry —

        // The block under test placed at (1,0,0), with an armor block at (0,0,0) so its
        // Back face has a mount partner — a lone block would (correctly) be rejected as
        // having no legal mount.
        private static string KindCraft(string xsiType, string subtype)
        {
            return "<?xml version=\"1.0\"?>\n"
                 + "<Definitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n"
                 + "  <ShipBlueprints>\n"
                 + "    <ShipBlueprint xsi:type=\"MyObjectBuilder_ShipBlueprintDefinition\">\n"
                 + "      <Id Type=\"MyObjectBuilder_ShipBlueprintDefinition\" Subtype=\"kind\" />\n"
                 + "      <CubeGrids><CubeGrid>\n"
                 + "        <GridSizeEnum>Small</GridSizeEnum>\n"
                 + "        <CubeBlocks>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_CubeBlock\"><SubtypeName>SmallBlockArmorBlock</SubtypeName><Min x=\"0\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "          <MyObjectBuilder_CubeBlock xsi:type=\"" + xsiType + "\"><SubtypeName>" + subtype
                 + "</SubtypeName><Min x=\"1\" y=\"0\" z=\"0\" /></MyObjectBuilder_CubeBlock>\n"
                 + "        </CubeBlocks>\n"
                 + "      </CubeGrid></CubeGrids>\n"
                 + "    </ShipBlueprint>\n"
                 + "  </ShipBlueprints>\n"
                 + "</Definitions>\n";
        }

        private static TemplateBlock SingleKind(string xsiType, string subtype)
        {
            SbcFile file = Reader.Read(KindCraft(xsiType, subtype));
            ShipTemplate template = SbcToShipTemplate.Convert(file, "kind");
            TemplateBlock underTest = template.Blocks[template.Blocks.Count - 1];
            return underTest;
        }

        [Test]
        public void ConnectorSubtype_ResolvesToConnectorKind()
        {
            TemplateBlock b = SingleKind("MyObjectBuilder_ShipConnector", "ConnectorSmall");
            Assert.That(b.Kind, Is.EqualTo(BlockKind.Connector));
            Assert.That(b.DryMass, Is.EqualTo(204f)); // real small connector mass
        }

        [Test]
        public void CockpitSubtype_ResolvesToCockpitKind()
        {
            TemplateBlock b = SingleKind("MyObjectBuilder_Cockpit", "SmallBlockCockpit");
            Assert.That(b.Kind, Is.EqualTo(BlockKind.Cockpit));
            Assert.That(b.Size, Is.EqualTo(new Vector3I(3, 3, 3)));
        }

        [Test]
        public void LargeReactorSubtype_ResolvesToReactorKind()
        {
            TemplateBlock b = SingleKind("MyObjectBuilder_Reactor", "LargeBlockLargeGenerator");
            Assert.That(b.Kind, Is.EqualTo(BlockKind.Reactor));
            Assert.That(b.MaxOutput, Is.EqualTo(300f).Within(1f));
        }

        [Test]
        public void LargeBatterySubtype_ResolvesToBatteryKind()
        {
            TemplateBlock b = SingleKind("MyObjectBuilder_BatteryBlock", "LargeBlockBatteryBlock");
            Assert.That(b.Kind, Is.EqualTo(BlockKind.Battery));
            Assert.That(b.MaxStoredPower, Is.EqualTo(3f).Within(0.1f));
        }

        [Test]
        public void LargeIonThruster_ResolvesToFixedThrusterKind()
        {
            TemplateBlock b = SingleKind("MyObjectBuilder_Thrust", "LargeBlockLargeThrust");
            Assert.That(b.Kind, Is.EqualTo(BlockKind.FixedThruster));
            Assert.That(b.MaxThrust, Is.EqualTo(4.32e6f).Within(1e3f));
        }

        [Test]
        public void AdapterRegistry_DiscoveredAllSupportedKinds()
        {
            // The registry must have found every adapter by reflection — every kind the
            // allocator implements must be reachable from an SBC subtype.
            Assert.That(SbcAdapterRegistry.TryGet("SmallBlockArmorBlock", out ISbcBlockAdapter std), Is.True);
            Assert.That(std, Is.InstanceOf<StandardBlockAdapter>());
            Assert.That(SbcAdapterRegistry.TryGet("SmallBlockGyro", out _), Is.True);
            Assert.That(SbcAdapterRegistry.TryGet("SmallBlockSmallThrust", out _), Is.True);
            // No block in the registry outside a converter.
            Assert.That(SbcAdapterRegistry.Count, Is.GreaterThanOrEqualTo(24));
        }

        private static void StringAssertContains(Exception ex, string fragment)
        {
            Assert.That(ex.Message, Does.Contain(fragment));
        }

        // — integration: the real blueprints —

        private static string Fixture(string folder)
        {
            return Path.Combine(TestContext.CurrentContext.TestDirectory, "blueprints", folder, "bp.sbc");
        }

        [Test]
        public void RealEquivalenceBlueprint_ConvertsToShipTemplate()
        {
            SbcFile file = Reader.ReadFile(Fixture("FleetCommand_Validation_v1"));
            ShipTemplate template = SbcToShipTemplate.Convert(file, file.Name);
            Assert.That(template.Blocks.Count, Is.EqualTo(183));
            // six thrusters with real direction each, one RC, one gyro, one battery, one reactor, one PB, one LCD
            Assert.That(Count(template, BlockKind.FixedThruster), Is.EqualTo(6));
            Assert.That(Count(template, BlockKind.Battery), Is.EqualTo(1));
            Assert.That(Count(template, BlockKind.Reactor), Is.EqualTo(3));
            Assert.That(Count(template, BlockKind.Gyro), Is.EqualTo(2));
            Assert.That(Count(template, BlockKind.ProgrammableBlock), Is.EqualTo(1));
            Assert.That(Count(template, BlockKind.TextPanel), Is.EqualTo(1));
        }

        [Test]
        public void RealTestGridBlueprint_ConvertsToShipTemplate()
        {
            SbcFile file = Reader.ReadFile(Fixture("FleetCommand_Validation_Dock_v1"));
            ShipTemplate template = SbcToShipTemplate.Convert(file, file.Name);
            Assert.That(template.Blocks.Count, Is.EqualTo(file.Grids[0].Blocks.Count));
            Assert.That(template.GridSize, Is.EqualTo(0.5f));
            Assert.That(Count(template, BlockKind.Connector), Is.EqualTo(1));
            Assert.That(Count(template, BlockKind.Battery), Is.EqualTo(1));
        }

        private static int Count(ShipTemplate template, BlockKind kind)
        {
            int n = 0;
            foreach (TemplateBlock b in template.Blocks)
            {
                if (b.Kind == kind)
                {
                    n++;
                }
            }
            return n;
        }
    }
}
