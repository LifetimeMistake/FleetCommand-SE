using System;
using System.Threading;
using System.Threading.Tasks;
using Xunit;

namespace FleetCommand.MCP.Tests
{
    public class GameWorldHandleTests
    {
        [Fact]
        public async Task RunOnSimThreadAsync_WhenNotReady_Throws()
        {
            var dispatcher = new SimDispatcher();
            var world = new GameWorldHandle(dispatcher);

            await Assert.ThrowsAsync<InvalidOperationException>(() => (Task)world.RunOnSimThreadAsync(() => 42));
        }

        [Fact]
        public async Task RunOnSimThreadAsync_WhenReady_RunsOnSimThreadAndReturnsResult()
        {
            var dispatcher = new SimDispatcher();
            var world = new GameWorldHandle(dispatcher);
            world.SetReady();

            var task = world.RunOnSimThreadAsync(() => Thread.CurrentThread.ManagedThreadId);

            int pumpThread = Thread.CurrentThread.ManagedThreadId;
            dispatcher.Tick();

            Assert.Equal(pumpThread, await task);
        }

        [Fact]
        public async Task RunOnSimThreadAsync_PropagatesException()
        {
            var dispatcher = new SimDispatcher();
            var world = new GameWorldHandle(dispatcher);
            world.SetReady();

            var task = world.RunOnSimThreadAsync<int>(() => throw new InvalidOperationException("boom"));
            dispatcher.Tick();

            var ex = await Assert.ThrowsAsync<InvalidOperationException>(() => task);
            Assert.Equal("boom", ex.Message);
        }

        [Fact]
        public async Task SetNotReady_AfterReady_GatesAgain()
        {
            var dispatcher = new SimDispatcher();
            var world = new GameWorldHandle(dispatcher);
            world.SetReady();
            world.SetNotReady();

            Assert.False(world.IsGameReady);
            await Assert.ThrowsAsync<InvalidOperationException>(() => (Task)world.RunOnSimThreadAsync(() => 1));
        }
    }
}
