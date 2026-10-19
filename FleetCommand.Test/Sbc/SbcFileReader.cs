using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using VRageMath;

namespace FleetCommand.Test.Sbc
{
    /// <summary>
    /// Reads a Space Engineers ShipBlueprint SBC (the <c>bp.sbc</c> export format) into
    /// an <see cref="SbcFile"/>. Parses the envelope and block list, then validates
    /// every grid with <see cref="SbcValidator"/> so an illegal blueprint is rejected
    /// at read time. Keying is on <c>SubtypeId</c> under <c>Id</c> (armor blocks have
    /// no <c>xsi:type</c>), not on the <c>xsi:type</c> attribute.
    /// </summary>
    public sealed class SbcFileReader
    {
        private static readonly XNamespace Xsi = "http://www.w3.org/2001/XMLSchema-instance";

        /// <summary>Parses <paramref name="xml"/> into a validated <see cref="SbcFile"/>.
        /// Throws <see cref="SbcValidationException"/> for a structurally illegal
        /// blueprint, or <see cref="FormatException"/> for malformed SBC.</summary>
        public SbcFile Read(string xml)
        {
            if (xml == null)
            {
                throw new ArgumentNullException(nameof(xml));
            }

            XDocument doc;
            try
            {
                doc = XDocument.Parse(xml);
            }
            catch (System.Xml.XmlException ex)
            {
                throw new FormatException("The SBC file is not well-formed XML.", ex);
            }

            XElement ship = doc.Root
                ?.Elements(NoNs("ShipBlueprints")).Elements(NoNs("ShipBlueprint")).FirstOrDefault();
            if (ship == null)
            {
                throw new FormatException("The SBC file has no <ShipBlueprint> definition.");
            }

            string name = (string)ship.Element(NoNs("Id"))?.Attribute("Subtype")
                       ?? (string)ship.Element(NoNs("DisplayName"))
                       ?? "unspecified";

            var grids = new List<SbcGrid>();
            foreach (XElement gridEl in ship.Elements(NoNs("CubeGrids")).Elements(NoNs("CubeGrid")))
            {
                grids.Add(ParseGrid(gridEl));
            }
            if (grids.Count == 0)
            {
                throw new FormatException("The SBC file defines no <CubeGrid>.");
            }

            var file = new SbcFile(name, grids);
            foreach (SbcGrid grid in file.Grids)
            {
                SbcValidator.Validate(grid);
            }
            return file;
        }

        /// <summary>Reads and validates the SBC file at <paramref name="path"/>.</summary>
        public SbcFile ReadFile(string path)
        {
            if (path == null)
            {
                throw new ArgumentNullException(nameof(path));
            }
            return Read(File.ReadAllText(path));
        }

        private SbcGrid ParseGrid(XElement gridEl)
        {
            string sizeName = (string)gridEl.Element(NoNs("GridSizeEnum")) ?? "Small";
            SbcGridSize size;
            switch (sizeName)
            {
                case "Small": size = SbcGridSize.Small; break;
                case "Large": size = SbcGridSize.Large; break;
                default: throw new FormatException($"Unknown GridSizeEnum '{sizeName}'.");
            }

            var blocks = new List<SbcBlock>();
            foreach (XElement blockEl in gridEl.Elements(NoNs("CubeBlocks")).Elements(NoNs("MyObjectBuilder_CubeBlock")))
            {
                blocks.Add(ParseBlock(blockEl));
            }
            return new SbcGrid(size, blocks, gridEl);
        }

        private SbcBlock ParseBlock(XElement blockEl)
        {
            string typeId = (string)blockEl.Attribute(Xsi + "type") ?? string.Empty;
            int idx = typeId.LastIndexOf('.');
            string simpleType = idx >= 0 ? typeId.Substring(idx + 1) : typeId;

            // Blueprint block elements carry <SubtypeName> (the <SubtypeId> form is used by
            // block *definitions*, not blueprint blocks). Read either to be robust.
            string subtype = (string)blockEl.Element(NoNs("SubtypeName"))
                          ?? (string)blockEl.Element(NoNs("SubtypeId"))
                          ?? string.Empty;

            XElement minEl = blockEl.Element(NoNs("Min"));
            Vector3I min = minEl == null ? Vector3I.Zero : ParseMin(minEl);

            XElement orient = blockEl.Element(NoNs("BlockOrientation"));
            string forward = CubeFacing.OrientForward;
            string up = CubeFacing.OrientUp;
            if (orient != null)
            {
                forward = (string)orient.Attribute("Forward") ?? CubeFacing.OrientForward;
                up = (string)orient.Attribute("Up") ?? CubeFacing.OrientUp;
            }

            return new SbcBlock(simpleType, subtype, min, forward, up, blockEl);
        }

        private static Vector3I ParseMin(XElement minEl)
        {
            int x = (int)minEl.Attribute("x");
            int y = (int)minEl.Attribute("y");
            int z = (int)minEl.Attribute("z");
            return new Vector3I(x, y, z);
        }

        private static XName NoNs(string local) => local;
    }
}
