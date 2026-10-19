using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using VRageMath;

namespace FleetCommand.Validation.Launcher
{
    /// <summary>Instantiate the serialized rigid grids and initial connector links, not a second test implementation.</summary>
    public static class AssemblySpawn
    {
        public static SimShip Spawn(FleetTestContext context, SbcFile file,
            Vector3D position, MatrixD orientation, MatrixD? referenceOrientation = null)
        {
            int anchor = 0;
            for (int i = 0; i < file.Grids.Count; i++)
                foreach (var block in file.Grids[i].Blocks)
                    if ((string)block.Data.Element("CustomName") == "FC Validation") anchor = i;
            var templates = new List<ShipTemplate>();
            for (int i = 0; i < file.Grids.Count; i++)
                templates.Add(SbcToShipTemplate.Convert(file.Grids[i], file.Name + "/" + i));
            if (referenceOrientation.HasValue)
            {
                TemplateBlock reference = null;
                foreach (var block in templates[anchor].Blocks)
                    if (block.Name == "FC Reference") reference = block;
                if (reference == null) throw new InvalidOperationException("Blueprint has no FC Reference controller.");
                orientation = MatrixD.Invert((MatrixD)reference.LocalTransform.GetOrientation()) * referenceOrientation.Value;
            }
            MatrixD desired = orientation.GetOrientation(); desired.Translation = position;
            MatrixD transform = MatrixD.Invert(file.Grids[anchor].WorldMatrix) * desired;
            var ships = new List<SimShip>();
            var connectors = new Dictionary<long, SimShipConnector>();
            for (int i = 0; i < file.Grids.Count; i++)
            {
                MatrixD pose = file.Grids[i].WorldMatrix * transform;
                var ship = context.Spawn(templates[i], SpawnState.At(pose.Translation, pose.GetOrientation()));
                ships.Add(ship);
                int n = 0;
                foreach (var block in file.Grids[i].Blocks)
                {
                    n++;
                    var connector = ship.TerminalSystem.GetBlockWithName((string)block.Data.Element("CustomName") ?? "B" + n) as SimShipConnector;
                    long id = (long?)block.Data.Element("EntityId") ?? 0;
                    if (connector != null && id != 0) connectors.Add(id, connector);
                }
            }
            var restored = new HashSet<long>();
            foreach (var grid in file.Grids)
                foreach (var block in grid.Blocks)
                {
                    long id = (long?)block.Data.Element("EntityId") ?? 0;
                    long otherId = (long?)block.Data.Element("ConnectedEntityId") ?? 0;
                    if (otherId == 0 || restored.Contains(id)) continue;
                    if ((bool?)block.Data.Element("IsApproaching") == true)
                        throw new NotSupportedException("Initially approaching connector links are not supported; use a connected fixture.");
                    SimShipConnector a, b;
                    if (!connectors.TryGetValue(id, out a) || !connectors.TryGetValue(otherId, out b))
                        throw new FormatException("Serialized connector link points to an absent endpoint.");
                    if ((bool?)block.Data.Element("IsMaster") == false) continue;
                    a.RestoreSerializedConnection(b);
                    restored.Add(id); restored.Add(otherId);
                }
            foreach (var grid in file.Grids)
                foreach (var block in grid.Blocks)
                    if (((long?)block.Data.Element("ConnectedEntityId") ?? 0) != 0
                        && !restored.Contains((long?)block.Data.Element("EntityId") ?? 0))
                        throw new FormatException("Serialized connector link has no master.");
            return ships[anchor];
        }
    }
}
