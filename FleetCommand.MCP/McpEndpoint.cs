using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using ModelContextProtocol;
using ModelContextProtocol.Protocol;
using ModelContextProtocol.Server;

namespace FleetCommand.MCP
{
    public sealed class McpEndpoint : IDisposable
    {
        private const string ServerName = "fleetcommand-mcp";
        private const string ServerVersion = "0.1.0";

        private readonly McpServerTool[] _tools;
        private int _disposed;

        public sealed class McpResponse
        {
            public bool WroteBody { get; internal set; }

            public string Body { get; internal set; }
        }

        public McpEndpoint(IEnumerable<McpServerTool> tools)
        {
            _tools = tools as McpServerTool[] ?? new List<McpServerTool>(tools).ToArray();
        }

        public async Task<McpResponse> ProcessAsync(string requestBody, CancellationToken cancellationToken = default)
        {
            var result = new McpResponse();
            if (string.IsNullOrEmpty(requestBody))
            {
                return result;
            }

            var message = JsonSerializer.Deserialize<JsonRpcMessage>(requestBody, McpJsonUtilities.DefaultOptions);
            if (message == null)
            {
                return result;
            }

            var transport = new StreamableHttpServerTransport(loggerFactory: null) { Stateless = true };
            var server = McpServer.Create(transport, BuildOptions(), loggerFactory: null, serviceProvider: null);
            Task runTask = null;
            try
            {
                runTask = server.RunAsync(cancellationToken);
                using (var responseBody = new MemoryStream())
                {
                    var wrote = await transport
                        .HandlePostRequestAsync(message, responseBody, cancellationToken)
                        .ConfigureAwait(false);
                    result.WroteBody = wrote;
                    if (wrote)
                    {
                        result.Body = Encoding.UTF8.GetString(responseBody.ToArray());
                    }
                }
            }
            finally
            {
                await transport.DisposeAsync().ConfigureAwait(false);
                if (runTask != null)
                {
                    await runTask.ConfigureAwait(false);
                }
                await server.DisposeAsync().ConfigureAwait(false);
            }
            return result;
        }

        private McpServerOptions BuildOptions()
        {
            var options = new McpServerOptions
            {
                ServerInfo = new Implementation { Name = ServerName, Version = ServerVersion },
                ToolCollection = new McpServerPrimitiveCollection<McpServerTool>(),
            };
            foreach (var tool in _tools)
            {
                options.ToolCollection.Add(tool);
            }
            return options;
        }

        public void Dispose()
        {
            Interlocked.Exchange(ref _disposed, 1);
        }
    }
}
