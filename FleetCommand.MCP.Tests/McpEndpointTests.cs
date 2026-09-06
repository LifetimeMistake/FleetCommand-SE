using System;
using System.Text;
using System.Threading.Tasks;
using FleetCommand.MCP;
using ModelContextProtocol.Server;
using Xunit;

namespace FleetCommand.MCP.Tests
{
    public class McpEndpointTests
    {
        private static McpEndpoint AddEndpoint()
        {
            var add = McpServerTool.Create(
                (Func<long, long, long>)((a, b) => a + b),
                new McpServerToolCreateOptions { Name = "add", Description = "Adds two integers." });
            return new McpEndpoint(new[] { add });
        }

        [Fact]
        public async Task ToolsList_ReturnsRegisteredTool()
        {
            using var endpoint = AddEndpoint();

            var response = await endpoint.ProcessAsync(
                "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"tools/list\",\"params\":{}}");

            Assert.True(response.WroteBody, "Expected a JSON-RPC response body.");
            Assert.Contains("\"name\":\"add\"", response.Body);
        }

        [Fact]
        public async Task CallTool_RunsAndReturnsResult()
        {
            using var endpoint = AddEndpoint();

            var response = await endpoint.ProcessAsync(
                "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/call\",\"params\":{\"name\":\"add\",\"arguments\":{\"a\":2,\"b\":3}}}");

            Assert.True(response.WroteBody);
            Assert.Contains("5", response.Body);
        }

        [Fact]
        public async Task UnknownMethod_ReturnsJsonRpcError()
        {
            using var endpoint = AddEndpoint();

            var response = await endpoint.ProcessAsync(
                "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"no_such_method\"}");

            Assert.True(response.WroteBody);
            Assert.Contains("\"error\"", response.Body);
        }

        [Fact]
        public void Disposed_MoreThanOnce_IsSafe()
        {
            var endpoint = AddEndpoint();
            endpoint.Dispose();
            endpoint.Dispose();
        }
    }
}
