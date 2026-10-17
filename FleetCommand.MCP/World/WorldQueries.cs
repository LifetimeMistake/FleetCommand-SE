using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.ModAPI;

namespace FleetCommand.MCP.World
{
    public static class WorldQueries
    {
        public static List<GridSnapshot> ReadGrids()
        {
            var result = new List<GridSnapshot>();
            var entities = new HashSet<IMyEntity>();
            MyAPIGateway.Entities.GetEntities(entities);

            foreach (var entity in entities)
            {
                if (entity is IMyCubeGrid grid)
                {
                    var pos = grid.WorldMatrix.Translation;
                    result.Add(new GridSnapshot
                    {
                        EntityId = grid.EntityId,
                        Name = grid.CustomName,
                        IsStatic = grid.IsStatic,
                        X = pos.X,
                        Y = pos.Y,
                        Z = pos.Z,
                    });
                }
            }
            return result;
        }
    }
}
