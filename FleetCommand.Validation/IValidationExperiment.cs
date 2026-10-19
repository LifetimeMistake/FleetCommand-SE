using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Validation
{
    public interface IValidationExperiment
    {
        bool IsRunning { get; }
        string Report { get; }
        void Main(string argument, UpdateType source);
    }
}
