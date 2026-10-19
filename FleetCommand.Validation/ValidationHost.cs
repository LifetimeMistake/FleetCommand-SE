using System;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Validation
{
    /// <summary>Both hosts route commands here; each experiment owns its complete LCD report.</summary>
    public sealed class ValidationHost : IValidationExperiment
    {
        readonly IMyGridTerminalSystem _gts;
        readonly IMyGridProgramRuntimeInfo _runtime;
        readonly IMyProgrammableBlock _me;
        readonly Action<string> _echo;
        IValidationExperiment _active;
        public bool IsRunning { get { return _active.IsRunning; } }
        public string Report { get { return _active.Report; } }

        public ValidationHost(IMyGridTerminalSystem gts, IMyGridProgramRuntimeInfo runtime,
            IMyProgrammableBlock me, Action<string> echo, string previousReport)
        {
            _gts = gts; _runtime = runtime; _me = me; _echo = echo;
            if (previousReport != null && previousReport.StartsWith(DockingProbeRunner.Header))
                _active = new DockingProbeRunner(gts, runtime, me, echo, previousReport);
            else _active = new ValidationRunner(gts, runtime, me, echo, previousReport);
        }

        public void Main(string argument, UpdateType source)
        {
            string command = (argument ?? "").Trim().ToLowerInvariant();
            if (command == "flightprobe" || command == "dockprobe")
            {
                if (IsRunning) { _echo("An experiment is already running. Use abort before switching."); return; }
                _active = command == "flightprobe"
                    ? (IValidationExperiment)new FlightProbeRunner(_gts, _runtime, _me, _echo, Report)
                    : new DockingProbeRunner(_gts, _runtime, _me, _echo, Report);
            }
            else if ((command == "start" || command == "restprobe") && !(_active is ValidationRunner))
            {
                if (IsRunning) { _echo("An experiment is already running. Use abort before switching."); return; }
                _active = new ValidationRunner(_gts, _runtime, _me, _echo, Report);
            }
            _active.Main(argument, source);
        }
    }
}
