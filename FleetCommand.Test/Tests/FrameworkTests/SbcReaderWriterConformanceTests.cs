using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using NUnit.Framework;
using VRageMath;
using FleetCommand.Test.Sbc;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Exercises the format layer: the generic SBC reader/writer and its
    /// structural validation (overlap, illegal mounts, floating blocks) on both read
    /// and write. Hermetic cases are embedded; the real equivalence and TEST GRID
    /// blueprints are exercised as an integration gate.</summary>
    [TestFixture]
    public class SbcReaderWriterConformanceTests
    {
        private static readonly SbcFileReader Reader = new SbcFileReader();
        private static readonly SbcFileWriter Writer = new SbcFileWriter();

        // — hermetic SBC builders —

        private static string Block(string type, string subtype, Vector3I min,
            string forward = null, string up = null)
        {
            var sb = new StringBuilder();
            sb.Append("        <MyObjectBuilder_CubeBlock xsi:type=\"MyObjectBuilder_")
              .Append(type).Append("\">\n");
            sb.Append("          <SubtypeId>").Append(subtype).Append("</SubtypeId>\n");
            sb.Append("          <Min x=\"").Append(min.X).Append("\" y=\"").Append(min.Y)
              .Append("\" z=\"").Append(min.Z).Append("\" />\n");
            if (forward != null && up != null)
            {
                sb.Append("          <BlockOrientation Forward=\"").Append(forward)
                  .Append("\" Up=\"").Append(up).Append("\" />\n");
            }
            sb.Append("        </MyObjectBuilder_CubeBlock>\n");
            return sb.ToString();
        }

        private static string Blueprint(string name, params string[] blocks)
        {
            var sb = new StringBuilder();
            sb.Append("<?xml version=\"1.0\"?>\n");
            sb.Append("<Definitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" ");
            sb.Append("xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n");
            sb.Append("  <ShipBlueprints>\n");
            sb.Append("    <ShipBlueprint xsi:type=\"MyObjectBuilder_ShipBlueprintDefinition\">\n");
            sb.Append("      <Id Type=\"MyObjectBuilder_ShipBlueprintDefinition\" Subtype=\"").Append(name).Append("\" />\n");
            sb.Append("      <CubeGrids>\n");
            sb.Append("        <CubeGrid>\n");
            sb.Append("          <GridSizeEnum>Small</GridSizeEnum>\n");
            sb.Append("          <CubeBlocks>\n");
            foreach (string b in blocks) sb.Append(b);
            sb.Append("          </CubeBlocks>\n");
            sb.Append("        </CubeGrid>\n");
            sb.Append("      </CubeGrids>\n");
            sb.Append("    </ShipBlueprint>\n");
            sb.Append("  </ShipBlueprints>\n");
            sb.Append("</Definitions>\n");
            return sb.ToString();
        }

        // A minimal but fully-legal small craft: armor chain, a gyro leaf on armor,
        // and a thruster mounted on the second armor's back (all mounts valid).
        private static string LegalCraft()
        {
            return Blueprint("legal",
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(0, 0, 0)),
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(1, 0, 0)),
                Block("Gyro", "SmallBlockGyro", new Vector3I(0, 1, 0)),
                Block("Thrust", "SmallBlockSmallThrust", new Vector3I(2, 0, 0), "Right", "Up"));
        }

        // A thruster whose only neighbor is a gyro through the gyro's front (the gyro
        // mounts only through its Bottom) — the classic illegal mount.
        private static string IllegalMountCraft()
        {
            return Blueprint("illegal-mount",
                Block("Gyro", "SmallBlockGyro", new Vector3I(0, 0, 0)),
                Block("Thrust", "SmallBlockSmallThrust", new Vector3I(1, 0, 0), "Left", "Up"));
        }

        private static string OverlapCraft()
        {
            return Blueprint("overlap",
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(0, 0, 0)),
                Block("Gyro", "SmallBlockGyro", new Vector3I(0, 0, 0)));
        }

        private static string FloatingCraft()
        {
            // Two internally-valid pieces (armor chains) separated by a gap: each bonds
            // to its neighbor, but the grid is two components — must be rejected.
            return Blueprint("floating",
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(0, 0, 0)),
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(1, 0, 0)),
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(10, 0, 0)),
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(11, 0, 0)));
        }

        [Test]
        public void LegalCraft_ReadsAndRoundTrips()
        {
            SbcFile file = Reader.Read(LegalCraft());
            Assert.That(file.Name, Is.EqualTo("legal"));
            Assert.That(file.Grids.Count, Is.EqualTo(1));
            Assert.That(file.Grids[0].Blocks.Count, Is.EqualTo(4));

            // round-trip: write, re-read, must still validate and match block count.
            string round = Writer.Write(file);
            SbcFile re = Reader.Read(round);
            Assert.That(re.Grids[0].Blocks.Count, Is.EqualTo(4));
        }

        [Test]
        public void SingleBlock_IsLegal()
        {
            // A lone block has no face-neighbor, but a one-block grid is legal in game
            // (placing a single block in open space creates its own grid) — not "no legal
            // mount", and not "disconnected" (it is one component).
            SbcFile file = Reader.Read(Blueprint("solo",
                Block("CubeBlock", "SmallBlockArmorBlock", new Vector3I(0, 0, 0))));
            Assert.That(file.Grids[0].Blocks.Count, Is.EqualTo(1));
        }

        [Test]
        public void IllegalMountCraft_IsRejectedOnRead()
        {
            var ex = Assert.Throws<SbcValidationException>((Action)(() => Reader.Read(IllegalMountCraft())));
            Assert.That(ex.Message, Does.Contain("no legal mount"));
        }

        [Test]
        public void OverlapCraft_IsRejectedOnRead()
        {
            var ex = Assert.Throws<SbcValidationException>((Action)(() => Reader.Read(OverlapCraft())));
            Assert.That(ex.Message, Does.Contain("overlaps"));
        }

        [Test]
        public void FloatingCraft_IsRejectedOnRead()
        {
            var ex = Assert.Throws<SbcValidationException>((Action)(() => Reader.Read(FloatingCraft())));
            Assert.That(ex.Message, Does.Contain("disconnected"));
        }

        [Test]
        public void Writer_RejectsIllegalModel()
        {
            SbcGrid bad = new SbcGrid(SbcGridSize.Small, new[]
            {
                new SbcBlock("Gyro", "SmallBlockGyro", Vector3I.Zero),
                new SbcBlock("Thrust", "SmallBlockSmallThrust", new Vector3I(1, 0, 0), "Left", "Up"),
            });
            var ex = Assert.Throws<SbcValidationException>((Action)(() => Writer.Write(new SbcFile("bad", new[] { bad }))));
            Assert.That(ex.Message, Does.Contain("no legal mount"));
        }

        [Test]
        public void Writer_ProducesLegalReadableOutput()
        {
            string xml = Writer.Write(Reader.Read(LegalCraft()));
            Assert.That(xml, Does.Contain("SmallBlockSmallThrust"));
            // written output re-reads and stays valid
            Assert.That(Reader.Read(xml).Grids[0].Blocks.Count, Is.EqualTo(4));
        }

        // — integration: the real blueprints the user is testing —

        private static string AppDataLocal(string folder)
        {
            string appData = Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData);
            string bp = Path.Combine(appData, "SpaceEngineers", "Blueprints", "local", folder, "bp.sbc");
            if (!File.Exists(bp))
            {
                Assert.Ignore($"blueprint not present: {bp}");
            }
            return bp;
        }

        [Test]
        public void RealEquivalenceBlueprint_DecodesValid()
        {
            SbcFile file = Reader.ReadFile(AppDataLocal("FleetCommand_EquivalenceTest"));
            // 18-block cross (RC + 6 armor arms each a thruster + gyro/reactor/battery/PB/LCD)
            Assert.That(file.Grids[0].Blocks.Count, Is.EqualTo(18));
        }

        [Test]
        public void RealTestGridBlueprint_DecodesValid()
        {
            // The user's "TEST GRID": includes legal-but-weird gyro and thruster
            // placements that the validator must accept (not reject a legal grid).
            SbcFile file = Reader.ReadFile(AppDataLocal("TEST GRID"));
            Assert.That(file.Grids.Count, Is.GreaterThanOrEqualTo(1));
        }

        [Test]
        public void RealTestGridIllegalGyroVariant_IsRejected()
        {
            // "TEST GRID illegal": the side-mounted gyro rotated so its only mount face
            // (Bottom) points at an empty cell. The user checks it is illegal in-game;
            // the validator must reject it the same way.
            string bp = AppDataLocal("TEST GRID illegal");
            var ex = Assert.Throws<SbcValidationException>((Action)(() => Reader.ReadFile(bp)));
            Assert.That(ex.Message, Does.Contain("no legal mount"));
        }
    }
}
