using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace FleetCommand.MCP
{
    public sealed class SimDispatcher
    {
        private readonly object _lock = new object();
        private readonly Queue<Action> _queue = new Queue<Action>();

     
        public int PendingCount                                                                 
        {
            get { lock (_lock) return _queue.Count; }
        }

        public Task<T> RunAsync<T>(Func<T> work)
        {
            var tcs = new TaskCompletionSource<T>(TaskCreationOptions.RunContinuationsAsynchronously);
            lock (_lock)
            {
                _queue.Enqueue(() =>
                {
                    try
                    {
                        tcs.SetResult(work());
                    }
                    catch (Exception ex)
                    {
                        tcs.SetException(ex);
                    }
                });
            }
            return tcs.Task;
        }

        public Task Enqueue(Action work)
        {
            return RunAsync(() =>
            {
                work();
                return true;
            });
        }

        public void Tick()
        {
            while (true)
            {
                Action action;
                lock (_lock)
                {
                    if (_queue.Count == 0)
                    {
                        return;
                    }
                    action = _queue.Dequeue();
                }
                try
                {
                    action();
                }
                catch
                {
                    
                }
            }
        }
    }
}
