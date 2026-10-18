using System.Xml.Linq;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.Templates;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class SbcStateTests
    {
        [Test]
        public void BatteryIdentityAndState_SurviveRoundTripAndConversion()
        {
            const string xml = "<Definitions xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'><ShipBlueprints><ShipBlueprint><Id Subtype='state'/><CubeGrids><CubeGrid><GridSizeEnum>Small</GridSizeEnum><CubeBlocks><MyObjectBuilder_CubeBlock xsi:type='MyObjectBuilder_BatteryBlock'><SubtypeName>SmallBlockSmallBatteryBlock</SubtypeName><CustomName>FC Battery</CustomName><Enabled>false</Enabled><CurrentStoredPower>0.025</CurrentStoredPower><ChargeMode>Discharge</ChargeMode></MyObjectBuilder_CubeBlock></CubeBlocks></CubeGrid></CubeGrids></ShipBlueprint></ShipBlueprints></Definitions>";
            var reader = new SbcFileReader();
            var file = reader.Read(xml);
            var written = XDocument.Parse(new SbcFileWriter().Write(file));
            Assert.That(written.Descendants("CustomName").FirstOrDefaultValue(), Is.EqualTo("FC Battery"));
            var block = SbcToShipTemplate.Convert(reader.Read(written.ToString()), "state").Blocks[0];
            Assert.That(block.Name, Is.EqualTo("FC Battery"));
            Assert.That(block.InitialStoredPower, Is.EqualTo(0.5f).Within(1e-6));
        }
    }

    internal static class SbcTestElements
    {
        public static string FirstOrDefaultValue(this System.Collections.Generic.IEnumerable<XElement> elements)
        {
            foreach (var element in elements) return element.Value;
            return null;
        }
    }
}
