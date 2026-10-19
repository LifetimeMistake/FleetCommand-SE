using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Xml.Linq;
using VRageMath;

namespace FleetCommand.Test.Sbc
{
    /// <summary>
    /// Serializes a <see cref="SbcFile"/> back to the ShipBlueprint SBC format.
    /// Runs <see cref="SbcValidator"/> on every grid first, so an illegal model can
    /// never be written — the same structural rules as the reader.
    /// </summary>
    public sealed class SbcFileWriter
    {
        private static readonly XNamespace Xsi = "http://www.w3.org/2001/XMLSchema-instance";
        private static readonly XNamespace Xsd = "http://www.w3.org/2001/XMLSchema";

        /// <summary>Writes <paramref name="file"/> to an SBC string, validating first.</summary>
        public string Write(SbcFile file)
        {
            if (file == null)
            {
                throw new ArgumentNullException(nameof(file));
            }
            foreach (SbcGrid grid in file.Grids)
            {
                SbcValidator.Validate(grid);
            }

            // Reserve identities before assigning any synthetic IDs, including references
            // whose targets are outside this file. Existing links must retain their meaning.
            var reservedIds = new HashSet<long>();
            foreach (SbcGrid grid in file.Grids)
            {
                ReserveIds(grid.Data, reservedIds);
                foreach (SbcBlock block in grid.Blocks) ReserveIds(block.Data, reservedIds);
            }
            long nextEntityId = 9000000000001;
            var gridNodes = new List<XElement>();
            foreach (SbcGrid grid in file.Grids)
            {
                gridNodes.Add(WriteGrid(grid, reservedIds, ref nextEntityId));
            }

            var ship = new XElement("ShipBlueprint",
                new XAttribute(Xsi + "type", "MyObjectBuilder_ShipBlueprintDefinition"),
                new XElement("Id",
                    new XAttribute("Type", "MyObjectBuilder_ShipBlueprintDefinition"),
                    new XAttribute("Subtype", file.Name ?? string.Empty)),
                new XElement("CubeGrids", gridNodes));

            var root = new XElement("Definitions",
                new XAttribute(XNamespace.Xmlns + "xsd", Xsd),
                new XAttribute(XNamespace.Xmlns + "xsi", Xsi),
                new XElement("ShipBlueprints", ship));

            var doc = new XDocument(new XDeclaration("1.0", "utf-8", null), root);
            return doc.Declaration + Environment.NewLine + doc;
        }

        private XElement WriteGrid(SbcGrid grid, HashSet<long> reservedIds, ref long nextEntityId)
        {
            var blocks = new List<XElement>();
            foreach (SbcBlock block in grid.Blocks)
            {
                long entityId;
                if (!TryReadId(block.Data.Element("EntityId"), out entityId))
                {
                    while (reservedIds.Contains(nextEntityId)) nextEntityId++;
                    entityId = nextEntityId++;
                    reservedIds.Add(entityId);
                }
                blocks.Add(WriteBlock(block, entityId));
            }

            var result = new XElement(grid.Data) { Name = "CubeGrid" };
            ReplaceElement(result, "GridSizeEnum", new XElement("GridSizeEnum", grid.GridSize == SbcGridSize.Small ? "Small" : "Large"));
            ReplaceElement(result, "CubeBlocks", new XElement("CubeBlocks", blocks));
            return result;
        }

        private static void ReplaceElement(XElement parent, string name, XElement replacement)
        {
            XElement[] existing = parent.Elements(name).ToArray();
            if (existing.Length == 0) parent.Add(replacement);
            else
            {
                existing[0].ReplaceWith(replacement);
                foreach (XElement duplicate in existing.Skip(1)) duplicate.Remove();
            }
        }

        private static bool TryReadId(XElement element, out long id)
        {
            return long.TryParse((string)element, NumberStyles.Integer,
                CultureInfo.InvariantCulture, out id) && id != 0;
        }

        private static void ReserveIds(XElement data, HashSet<long> reservedIds)
        {
            foreach (XElement element in data.Descendants())
            {
                if (!element.Name.LocalName.EndsWith("EntityId", StringComparison.Ordinal)) continue;
                long id;
                if (TryReadId(element, out id)) reservedIds.Add(id);
            }
        }

        private XElement WriteBlock(SbcBlock block, long entityId)
        {
            var el = new XElement("MyObjectBuilder_CubeBlock",
                new XAttribute(Xsi + "type", "MyObjectBuilder_" + SafeTypeId(block.TypeId)),
                new XElement("SubtypeName", block.SubtypeId),
                new XElement("EntityId", entityId),
                new XElement("Min",
                    new XAttribute("x", block.Min.X),
                    new XAttribute("y", block.Min.Y),
                    new XAttribute("z", block.Min.Z)));

            bool identity = string.Equals(block.ForwardName, CubeFacing.OrientForward, StringComparison.Ordinal)
                         && string.Equals(block.UpName, CubeFacing.OrientUp, StringComparison.Ordinal);
            if (!identity)
            {
                el.Add(new XElement("BlockOrientation",
                    new XAttribute("Forward", block.ForwardName),
                    new XAttribute("Up", block.UpName)));
            }
            foreach (XElement child in block.Data.Elements())
            {
                string key = child.Name.LocalName;
                if (key != "SubtypeName" && key != "SubtypeId" && key != "EntityId"
                    && key != "Min" && key != "BlockOrientation")
                    el.Add(new XElement(child));
            }
            return el;
        }

        private static string SafeTypeId(string typeId)
        {
            if (string.IsNullOrEmpty(typeId))
            {
                return "CubeBlock";
            }
            return typeId.StartsWith("MyObjectBuilder_", StringComparison.Ordinal) ? typeId.Substring("MyObjectBuilder_".Length) : typeId;
        }
    }
}
