using System;
using System.Globalization;
using System.Linq;
using System.Xml.Linq;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class SbcGridMetadataTests
    {
        private static string Blueprint(params string[] grids)
        {
            return "<Definitions><ShipBlueprints><ShipBlueprint><Id Subtype='assembly'/><CubeGrids>"
                + string.Concat(grids) + "</CubeGrids></ShipBlueprint></ShipBlueprints></Definitions>";
        }

        private static string Grid(string size, long gridId, long blockId, long connectedId, bool isStatic)
        {
            return "<CubeGrid custom='preserve'><EntityId>" + gridId + "</EntityId><GridSizeEnum>" + size
                + "</GridSizeEnum><IsStatic>" + isStatic.ToString().ToLowerInvariant()
                + "</IsStatic><DisplayName>Dock</DisplayName><UnknownGridState flag='keep'>opaque</UnknownGridState>"
                + "<PositionAndOrientation><Position x='12.5' y='-3.25' z='100'/><Forward x='1' y='0' z='0'/>"
                + "<Up x='0' y='1' z='0'/></PositionAndOrientation><CubeBlocks><MyObjectBuilder_CubeBlock>"
                + "<SubtypeName>" + (size == "Small" ? "SmallBlockArmorBlock" : "LargeBlockArmorBlock")
                + "</SubtypeName><EntityId>" + blockId + "</EntityId><ConnectedEntityId>" + connectedId
                + "</ConnectedEntityId></MyObjectBuilder_CubeBlock></CubeBlocks></CubeGrid>";
        }

        [Test]
        public void MultiGridRoundTrip_PreservesPlacementStaticUnknownStateAndPairedBlockIds()
        {
            var file = new SbcFileReader().Read(Blueprint(Grid("Large", 101, 201, 202, true), Grid("Small", 102, 202, 201, false)));
            var grids = XDocument.Parse(new SbcFileWriter().Write(file)).Descendants("CubeGrid").ToArray();
            Assert.That(grids[0].Element("IsStatic")?.Value, Is.EqualTo("true"));
            Assert.That(grids[1].Element("IsStatic")?.Value, Is.EqualTo("false"));
            Assert.That(grids[0].Element("PositionAndOrientation")?.Element("Position")?.Attribute("x")?.Value, Is.EqualTo("12.5"));
            Assert.That(grids[0].Element("EntityId")?.Value, Is.EqualTo("101"));
            Assert.That(grids[0].Element("DisplayName")?.Value, Is.EqualTo("Dock"));
            Assert.That(grids[0].Element("UnknownGridState")?.Attribute("flag")?.Value, Is.EqualTo("keep"));
            Assert.That(grids[0].Attribute("custom")?.Value, Is.EqualTo("preserve"));
            Assert.That(grids[0].Descendants("CubeBlocks").Descendants("EntityId").Single().Value, Is.EqualTo("201"));
            Assert.That(grids[1].Descendants("CubeBlocks").Descendants("EntityId").Single().Value, Is.EqualTo("202"));
            Assert.That(grids[0].Descendants("ConnectedEntityId").Single().Value, Is.EqualTo("202"));
            Assert.That(grids[1].Descendants("ConnectedEntityId").Single().Value, Is.EqualTo("201"));
            var reread = new SbcFileReader().Read(new SbcFileWriter().Write(file));
            Assert.That(reread.Grids[0].WorldMatrix.Translation, Is.EqualTo(new Vector3D(12.5, -3.25, 100)));
            Assert.That(reread.Grids[1].WorldMatrix.Forward, Is.EqualTo(Vector3D.Right));
        }

        [Test]
        public void MissingBlockIds_AreUniqueAcrossGridsAndAvoidExistingGridAndBlockIds()
        {
            var reader = new SbcFileReader();
            var file = reader.Read(Blueprint(Grid("Small", 9000000000001, 9000000000002, 0, false),
                Grid("Small", 12, 0, 0, false), Grid("Small", 13, 0, 0, false)));
            string written = new SbcFileWriter().Write(file);
            long[] ids = XDocument.Parse(written).Descendants("CubeBlocks").Descendants("EntityId").Select(e => (long)e).ToArray();
            Assert.That(ids, Is.Unique);
            Assert.That(ids, Has.None.EqualTo(0));
            Assert.That(ids, Has.None.EqualTo(9000000000001));
            Assert.That(ids[0], Is.EqualTo(9000000000002));
            Assert.That(new SbcFileWriter().Write(file), Is.EqualTo(written));
        }

        [Test]
        public void StaticGridConversion_PreservesStaticFlag()
        {
            var file = new SbcFileReader().Read(Blueprint(Grid("Large", 101, 201, 0, true)));
            Assert.That(SbcToShipTemplate.Convert(file, "station").IsStatic, Is.True);
            Assert.That(SbcToShipTemplate.Convert(file.Grids[0], "station").IsStatic, Is.True);
        }

        [Test]
        public void GridPlacement_UsesInvariantNumbersAndDefensivelyCopiesSerializedData()
        {
            CultureInfo previous = CultureInfo.CurrentCulture;
            try
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("pl-PL");
                var source = XElement.Parse(Grid("Large", 101, 201, 0, true));
                var grid = new SbcGrid(SbcGridSize.Large, new SbcBlock[0], source);
                source.Element("PositionAndOrientation").Element("Position").SetAttributeValue("x", "999");
                Assert.That(grid.WorldMatrix.Translation, Is.EqualTo(new Vector3D(12.5, -3.25, 100)));
                Assert.That(grid.WorldMatrix.Forward, Is.EqualTo(Vector3D.Right));
                Assert.That(grid.WorldMatrix.Up, Is.EqualTo(Vector3D.Up));
                Assert.That(grid.SerializedEntityId, Is.EqualTo(101));
                var empty = new SbcGrid(SbcGridSize.Small, new SbcBlock[0]);
                Assert.That(empty.WorldMatrix, Is.EqualTo(MatrixD.Identity));
                Assert.That(empty.IsStatic, Is.False);
                Assert.That(empty.SerializedEntityId, Is.Null);
            }
            finally { CultureInfo.CurrentCulture = previous; }
        }

        [Test]
        public void SingleGridFileConversion_StillRejectsAnAssembly()
        {
            var file = new SbcFileReader().Read(Blueprint(Grid("Large", 101, 201, 202, true), Grid("Small", 102, 202, 201, false)));
            Assert.Throws<ArgumentException>((Action)(() => SbcToShipTemplate.Convert(file, "assembly")));
            Assert.That(SbcToShipTemplate.Convert(file.Grids[1], "ship").IsStatic, Is.False);
        }
    }
}
