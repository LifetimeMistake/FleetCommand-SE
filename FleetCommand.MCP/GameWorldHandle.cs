using System;
using System.Threading.Tasks;

namespace FleetCommand.MCP
{
    public sealed class GameWorldHandle
    {
        private readonly SimDispatcher _dispatcher;
        private volatile bool _ready;
        private volatile bool _disposed;

        public GameWorldHandle(SimDispatcher dispatcher)
        {
            _dispatcher = dispatcher ?? throw new ArgumentNullException(nameof(dispatcher));
        }

        public bool IsGameReady => _ready && !_disposed;

        public Task<T> RunOnSimThreadAsync<T>(Func<T> work)
        {
            if (!IsGameReady)
            {
                throw new InvalidOperationException(
                    "The game world is not ready — a session must be loaded before querying game state.");
            }
            return _dispatcher.RunAsync(work);
        }

        public void SetReady() => _ready = true;

        public void SetNotReady() => _ready = false;

        public void Dispose()
        {
            _disposed = true;
            _ready = false;
        }
    }
}
