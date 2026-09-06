using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using FleetCommand.MCP;
using ModelContextProtocol.Client;
using ModelContextProtocol.Protocol;
using ModelContextProtocol.Server;
using Xunit;

namespace FleetCommand.MCP.Tests
{
    public class McpHttpHostTests
    {
        [Fact]
        public async Task Client_ListsAndCallsTool_OverLoopbackHttp()
        {
            var add = McpServerTool.Create(
                (Func<long, long, long>)((a, b) => a + b),
                new McpServerToolCreateOptions { Name = "add", Description = "Adds two integers." });

            using var endpoint = new McpEndpoint(new[] { add });
            var port = FreePort();
            using var host = new McpHttpHost(endpoint, $"http://localhost:{port}/");
            host.Start();

            var httpTransport = new HttpClientTransport(
                new HttpClientTransportOptions { Endpoint = new Uri($"http://localhost:{port}/") },
                loggerFactory: null);
            await using var client = await McpClient
                .CreateAsync(httpTransport, new McpClientOptions(), loggerFactory: null, CancellationToken.None);

            var tools = await client.ListToolsAsync();
            Assert.Contains(tools, t => t.Name == "add");

            var result = await client.CallToolAsync(
                "add",
                new Dictionary<string, object> { ["a"] = 2L, ["b"] = 3L });

            var text = string.Concat(result.Content.OfType<TextContentBlock>().Select(c => c.Text));
            Assert.Contains("5", text);
        }

        private static int FreePort()
        {
            var listener = new TcpListener(IPAddress.Loopback, 0);
            listener.Start();
            int port = ((IPEndPoint)listener.LocalEndpoint).Port;
            listener.Stop();
            return port;
        }
    }
}
