using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Xunit;

namespace FleetCommand.MCP.Tests
{
    public class SimDispatcherTests
    {
        [Fact]
        public async Task RunAsync_CompletesWithResult_WhenDrainedOnSimThread()
        {
            var dispatcher = new SimDispatcher();
            var simThread = Thread.CurrentThread;

            var task = dispatcher.RunAsync(() =>
            {
                Assert.Same(simThread, Thread.CurrentThread);
                return 21 * 2;
            });

            Assert.False(task.IsCompleted);
            dispatcher.Tick();
            Assert.Equal(42, await task);
        }

        [Fact]
        public async Task RunAsync_PropagatesWorkException_ToCaller_WithoutCrashingPump()
        {
            var dispatcher = new SimDispatcher();

            var task = dispatcher.RunAsync<int>(() => { throw new InvalidOperationException("boom"); });

            dispatcher.Tick();
            await Assert.ThrowsAsync<InvalidOperationException>(() => task);
        }

        [Fact]
        public async Task Enqueue_RunsWork_InFifoOrder_OnePumpDrainsAll()
        {
            var dispatcher = new SimDispatcher();
            var order = new List<int>();
            var tasks = new List<Task>();

            for (int i = 0; i < 5; i++)
            {
                int n = i;
                tasks.Add(dispatcher.Enqueue(() => order.Add(n)));
            }

            Assert.Equal(5, dispatcher.PendingCount);
            dispatcher.Tick();
            await Task.WhenAll(tasks);

            Assert.Equal(new[] { 0, 1, 2, 3, 4 }, order);
            Assert.Equal(0, dispatcher.PendingCount);
        }

        [Fact]
        public void Tick_DoesNothing_WhenQueueEmpty()
        {
            var dispatcher = new SimDispatcher();
            dispatcher.Tick();
            Assert.Equal(0, dispatcher.PendingCount);
        }

        [Fact]
        public async Task ConcurrentEnqueues_AllComplete_AndRunOnPump()
        {
            const int count = 200;
            var dispatcher = new SimDispatcher();

            var tasks = new Task<int>[count];
            for (int i = 0; i < count; i++)
            {
                int n = i;
                tasks[i] = Task.Run(() => dispatcher.RunAsync<int>(() => n));
            }

            var deadline = DateTime.UtcNow.AddSeconds(10);
            while (tasks.Any(t => !t.IsCompleted) && DateTime.UtcNow < deadline)
            {
                dispatcher.Tick();
                Thread.Sleep(2);
            }

            var results = await Task.WhenAll(tasks);
            Assert.Equal(count, results.Length);
            for (int i = 0; i < count; i++)
            {
                Assert.Equal(i, results[i]);
            }
            Assert.Equal(0, dispatcher.PendingCount);
        }
    }
}
