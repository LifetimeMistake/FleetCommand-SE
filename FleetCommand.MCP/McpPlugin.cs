using System;
using System.IO;
using ModelContextProtocol.Server;
using NLog;
using NLog.Config;
using NLog.Targets;
using Torch;
using Torch.API;
using Torch.API.Managers;
using Torch.API.Session;

namespace FleetCommand.MCP
{
    public class McpPlugin : TorchPluginBase
    {
        private const int ServerPort = 45670;

        private Logger _log;
        private ITorchSessionManager _sessionManager;
        private SimDispatcher _dispatcher;
        private GameWorldHandle _world;
        private McpEndpoint _endpoint;
        private McpHttpHost _host;
        private bool _started;

        public override void Init(ITorchBase torch)
        {
            base.Init(torch);
            if (_started)
            {
                _log?.Info("FleetCommand.MCP already initialized; skipping.");
                return;
            }

            _log = CreateLogger();
            _log.Info("FleetCommand.MCP v{0} initializing.", Version);

            _dispatcher = new SimDispatcher();
            _world = new GameWorldHandle(_dispatcher);
            _endpoint = new McpEndpoint(BuildTools());

            var prefix = $"http://localhost:{ServerPort}/";
            _host = new McpHttpHost(_endpoint, prefix, msg => _log.Info(msg));

            _sessionManager = Torch.Managers?.GetManager<ITorchSessionManager>();
            if (_sessionManager != null)
            {
                _sessionManager.SessionStateChanged += OnSessionStateChanged;

                var current = _sessionManager.CurrentSession;
                if (current != null && current.State == TorchSessionState.Loaded)
                {
                    _world.SetReady();
                }
            }
            else
            {
                _log.Warn("No Torch session manager found; world readiness will never be reported.");
            }

            _host.Start();
            _started = true;
            _log.Info("MCP endpoint listening on {0}", prefix);
        }

        public override void Update()
        {
            if (_started)
            {
                _dispatcher.Tick();
            }
        }

        public override void Dispose()
        {
            if (_started)
            {
                _started = false;
                _log?.Info("FleetCommand.MCP shutting down.");

                if (_sessionManager != null)
                {
                    _sessionManager.SessionStateChanged -= OnSessionStateChanged;
                    _sessionManager = null;
                }

                _host?.Dispose();
                _host = null;
                _endpoint?.Dispose();
                _endpoint = null;
                _world?.Dispose();
                _world = null;
                _dispatcher = null;
            }
            base.Dispose();
        }

        private void OnSessionStateChanged(ITorchSession session, TorchSessionState newState)
        {
            switch (newState)
            {
                case TorchSessionState.Loaded:
                    _log.Info("Game session loaded; world ready.");
                    _world.SetReady();
                    break;

                case TorchSessionState.Unloading:
                case TorchSessionState.Unloaded:
                    _log.Info("Game session {0}; world not ready.", newState);
                    _world.SetNotReady();
                    break;
            }
        }

        private static McpServerTool[] BuildTools()
        {
            return Array.Empty<McpServerTool>();
        }

        private Logger CreateLogger()
        {
            Directory.CreateDirectory(StoragePath);
            var config = new LoggingConfiguration();
            var file = new FileTarget("fleetfile")
            {
                FileName = Path.Combine(StoragePath, "FleetCommand.MCP", "FleetCommand.MCP.log"),
                Layout = "${longdate} [${level:uppercase=true}] ${message}",
            };
            config.AddRule(LogLevel.Info, LogLevel.Fatal, file);

            var factory = new LogFactory();
            factory.Configuration = config;
            return factory.GetLogger("FleetCommand.MCP");
        }
    }
}
