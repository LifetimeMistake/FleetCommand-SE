using System;
using System.IO;
using NLog;
using NLog.Config;
using NLog.Targets;
using Torch;
using Torch.API;

namespace FleetCommand.MCP
{
    public class McpPlugin : TorchPluginBase
    {
        private Logger _log;

        public override void Init(ITorchBase torch)
        {
            base.Init(torch);
            _log = CreateLogger();
            _log.Info("FleetCommand.MCP loaded. Plugin: {0} v{1} (guid {2})", Name, Version, Id);
            _log.Info("StoragePath: {0}", StoragePath);
        }

        public override void Update()
        {
            
        }

        public override void Dispose()
        {
            _log?.Info("FleetCommand.MCP unloading.");
            base.Dispose();
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
