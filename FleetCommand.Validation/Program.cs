using FleetCommand.Validation;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        readonly ValidationRunner _runner;

        public Program()
        {
            _runner = new ValidationRunner(GridTerminalSystem, Runtime, Me, Echo, Storage);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            _runner.Main(argument, updateSource);
            if (!_runner.IsRunning) Storage = _runner.Report;
        }

        public void Save()
        {
            Storage = _runner.Report;
        }
    }
}
