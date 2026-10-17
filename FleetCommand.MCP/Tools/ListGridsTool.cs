using System;
using System.Collections.Generic;
using System.Text.Json;
using FleetCommand.MCP.World;
using ModelContextProtocol.Server;

namespace FleetCommand.MCP.Tools
{
    public static class ListGridsTool
    {
        public static McpServerTool Create(GameWorldHandle world, Func<IReadOnlyList<GridSnapshot>> readGrids = null)
        {
            if (world == null)
            {
                throw new ArgumentNullException(nameof(world));
            }
            readGrids = readGrids ?? WorldQueries.ReadGrids;

            return McpServerTool.Create(
                (Func<string>)(() =>
                {
                    var grids = world.RunOnSimThreadAsync(readGrids).GetAwaiter().GetResult();
                    return JsonSerializer.Serialize(grids);
                }),
                new McpServerToolCreateOptions
                {
                    Name = "list_grids",
                    Description = "Lists all grids in the world with their id, name, and position.",
                });
        }
    }
}
