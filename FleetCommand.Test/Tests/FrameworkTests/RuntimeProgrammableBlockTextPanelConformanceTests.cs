using System;
using System.Text;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;
using VRageMath;
using static FleetCommand.Test.Shared.TestHelpers;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (, ): the programmable-block
    /// runtime surface (<see cref="SimRuntimeInfo"/> — elapsed sim time from
    /// the fixed tick clock, update frequency, host instruction limits), the
    /// programmable block's explicit execution hooks and storage context
    /// (), and the text panel's logger-compatible text
    /// writes and reads.
    /// </summary>
    [TestFixture]
    public class RuntimeProgrammableBlockTextPanelConformanceTests : FleetTestFixture
    {
        // — runtime: deterministic time from the fixed clock —

        [Test]
        public void Runtime_TimeSinceLastRun_ZeroBeforeFirstRun()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            Assert.That(rt.TimeSinceLastRun, Is.EqualTo(TimeSpan.Zero),
                "Before the program first runs, TimeSinceLastRun has no valid data (game convention).");
        }

        [Test]
        public void Runtime_TimeSinceLastRun_ReflectsElapsedFixedSteps()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            Fc.Step(10);

            rt.MarkRun(); // the program's Main ran at tick 10

            Fc.Step(30); // 30 fixed steps elapse
            Assert.That(rt.TimeSinceLastRun.TotalSeconds, Is.EqualTo(30.0 / 60.0).Within(1e-9),
                "TimeSinceLastRun must be the fixed-step interval since the last run — never wall-clock.");
        }

        [Test]
        public void Runtime_TimeSinceLastRun_AdvancesWithEachRun()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            rt.MarkRun(); // tick 0

            Fc.Step(60);
            Assert.That(rt.TimeSinceLastRun.TotalSeconds, Is.EqualTo(60.0 / 60.0).Within(1e-9));

            rt.MarkRun(); // resets the interval
            Assert.That(rt.TimeSinceLastRun.TotalSeconds, Is.EqualTo(0.0).Within(1e-9));
        }

        [Test]
        public void Runtime_LifetimeTicks_AdvanceWithSimulation()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            long start = rt.LifetimeTicks;
            Fc.Step(25);
            Assert.That(rt.LifetimeTicks, Is.EqualTo(start + 25),
                "LifetimeTicks must count fixed simulation ticks since the program started.");
        }

        [Test]
        public void Runtime_UpdateFrequency_Settable()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            Assert.That(rt.UpdateFrequency, Is.EqualTo(UpdateFrequency.None));
            rt.UpdateFrequency = UpdateFrequency.Update10;
            Assert.That(rt.UpdateFrequency, Is.EqualTo(UpdateFrequency.Update10));
            rt.UpdateFrequency = UpdateFrequency.Update100;
            Assert.That(rt.UpdateFrequency, Is.EqualTo(UpdateFrequency.Update100));
        }

        [Test]
        public void Runtime_HostLimits_Exposed()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            Assert.That(rt.MaxInstructionCount, Is.EqualTo(SimRuntimeInfo.InstructionLimit));
            Assert.That(rt.MaxCallChainDepth, Is.EqualTo(SimRuntimeInfo.CallChainLimit));
        }

        [Test]
        public void Runtime_UnsupportedCounters_ThrowNamed()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            AssertThrowsNamed(() => rt.LastRunTimeMs, "IMyGridProgramRuntimeInfo.LastRunTimeMs");
            AssertThrowsNamed(() => rt.CurrentInstructionCount, "IMyGridProgramRuntimeInfo.CurrentInstructionCount");
            AssertThrowsNamed(() => rt.CurrentCallChainDepth, "IMyGridProgramRuntimeInfo.CurrentCallChainDepth");
        }

        // — programmable block: identity and explicit execution () —

        [Test]
        public void ProgrammableBlock_ProvidesIdentityAndFunctionalSurface()
        {
            SimShip ship = Fc.Spawn(PbTemplate("PB"), SpawnState.At(Vector3D.Zero));
            SimProgrammableBlock pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("PB");

            Assert.That(pb, Is.Not.Null);
            Assert.That(pb, Is.InstanceOf<IMyProgrammableBlock>());
            Assert.That(pb, Is.InstanceOf<IMyFunctionalBlock>());
            Assert.That(pb.CustomName, Is.EqualTo("PB"));
            Assert.That(pb.CubeGrid, Is.SameAs(ship.Grid));
            Assert.That(pb.IsWorking, Is.True);

            pb.Enabled = false;
            Assert.That(pb.IsWorking, Is.False, "A programmable block is a functional block.");
            pb.Enabled = true;
        }

        [Test]
        public void ProgrammableBlock_TryRun_InvokesExplicitHook()
        {
            SimProgrammableBlock pb = SpawnPb();
            string captured = null;
            pb.RunHandler = arg => captured = arg;

            bool ran = pb.TryRun("run-argument");
            Assert.That(ran, Is.True);
            Assert.That(captured, Is.EqualTo("run-argument"), "TryRun must invoke the explicit execution hook with the argument.");
            Assert.That(pb.IsRunning, Is.False, "IsRunning must be false once the synchronous run completes.");
        }

        [Test]
        public void ProgrammableBlock_TryRun_RefusedWhileRunning()
        {
            SimProgrammableBlock pb = SpawnPb();
            bool reentered = false;
            pb.RunHandler = _ =>
            {
                Assert.That(pb.IsRunning, Is.True, "IsRunning must be true during execution.");
                reentered = !pb.TryRun("again");
            };

            Assert.That(pb.TryRun("first"), Is.True);
            Assert.That(reentered, Is.True, "An already-running programmable block cannot be run again.");
        }

        [Test]
        public void ProgrammableBlock_TryRun_WithNoHandler_StillRunsAndMarks()
        {
            SimProgrammableBlock pb = SpawnPb();
            SimRuntimeInfo rt = pb.Runtime;
            Assert.That(rt.TimeSinceLastRun, Is.EqualTo(TimeSpan.Zero));

            Fc.Step(5);
            Assert.That(pb.TryRun(""), Is.True, "TryRun must also work without an attached handler (identity/storage tests).");

            Fc.Step(30); // 30 fixed steps = exactly 0.5 s (TimeSpan-safe interval)
            Assert.That(rt.TimeSinceLastRun.TotalSeconds, Is.EqualTo(30.0 / 60.0).Within(1e-9),
                "TryRun must mark the program's run so the runtime's interval tracks the fixed clock.");
        }

        [Test]
        public void ProgrammableBlock_Storage_IsProjectService()
        {
            SimProgrammableBlock pb = SpawnPb();
            Assert.That(pb.Storage, Is.EqualTo(string.Empty));
            pb.Storage = "saved across saves";
            Assert.That(pb.Storage, Is.EqualTo("saved across saves"));
        }

        [Test]
        public void ProgrammableBlock_TerminalRunArgument_Settable()
        {
            SimProgrammableBlock pb = SpawnPb();
            pb.TerminalRunArgument = "default";
            Assert.That(pb.TerminalRunArgument, Is.EqualTo("default"));
        }

        [Test]
        public void ProgrammableBlock_TryRun_NullArgument_ForwardsTerminalRunArgument()
        {
            // A null explicit argument must still run and forward the
            // default terminal run argument (the game's OnRunDefaultApplied).
            // (Non-default explicit arguments are covered by
            // ProgrammableBlock_TryRun_InvokesExplicitHook.)
            SimProgrammableBlock pb = SpawnPb();
            pb.TerminalRunArgument = "default-arg";
            string captured = null;
            pb.RunHandler = s => captured = s;

            Assert.That(pb.TryRun(null), Is.True, "A null explicit argument must still run.");
            Assert.That(captured, Is.EqualTo("default-arg"),
                "A null explicit argument must forward TerminalRunArgument (game OnRunDefaultApplied).");
        }

        [Test]
        public void ProgrammableBlock_TextSurfaceProvider_NoSurface()
        {
            SimProgrammableBlock pb = SpawnPb();
            IMyTextSurfaceProvider provider = pb;
            Assert.That(provider.SurfaceCount, Is.EqualTo(0));
            Assert.That(provider.GetSurface(0), Is.Null);
        }

        // — text panel: logger-compatible writes and reads —

        [Test]
        public void TextPanel_WriteText_ReplaceAndAppend()
        {
            SimTextPanel panel = SpawnPanel();
            panel.WriteText("line one");
            Assert.That(panel.GetText(), Is.EqualTo("line one"));

            panel.WriteText("\nline two", append: true);
            Assert.That(panel.GetText(), Is.EqualTo("line one\nline two"),
                "append must add to the existing public text.");

            panel.WriteText("overwritten");
            Assert.That(panel.GetText(), Is.EqualTo("overwritten"),
                "non-append must replace the public text.");
        }

        [Test]
        public void TextPanel_WriteText_StringBuilder_AndReadText_RoundTrip()
        {
            SimTextPanel panel = SpawnPanel();
            Assert.That(panel.WriteText(new StringBuilder("from builder")), Is.True);

            var read = new StringBuilder();
            panel.ReadText(read);
            Assert.That(read.ToString(), Is.EqualTo("from builder"));

            var appended = new StringBuilder("prefix; ");
            panel.ReadText(appended, append: true);
            Assert.That(appended.ToString(), Is.EqualTo("prefix; from builder"),
                "ReadText with append must not clear the target buffer.");
        }

        [Test]
        public void TextPanel_PublicTitle_WriteAndRead()
        {
            SimTextPanel panel = SpawnPanel();
            Assert.That(panel.WritePublicTitle("Sea state"), Is.True);
            Assert.That(panel.GetPublicTitle(), Is.EqualTo("Sea state"));
            panel.WritePublicTitle(" report", append: true);
            Assert.That(panel.GetPublicTitle(), Is.EqualTo("Sea state report"));
        }

        [Test]
        public void TextPanel_WriteText_CappedAtMaxLength()
        {
            SimTextPanel panel = SpawnPanel();
            panel.WriteText(new string('a', SimTextPanel.MaxTextLength + 5000));
            Assert.That(panel.GetText().Length, Is.EqualTo(SimTextPanel.MaxTextLength),
                "Public text must be capped at the documented 100000-symbol limit.");
        }

        [Test]
        public void TextPanel_SurfaceAttributes_Stored()
        {
            SimTextPanel panel = SpawnPanel();
            Assert.That(panel.Name, Is.EqualTo("Panel"));
            Assert.That(panel.DisplayName, Is.EqualTo("Panel"));

            panel.FontSize = 2f;
            panel.FontColor = Color.Teal;
            panel.BackgroundColor = Color.DarkGray;
            panel.Alignment = TextAlignment.CENTER;
            panel.Font = "Monospace";
            panel.ContentType = ContentType.TEXT_AND_IMAGE;
            panel.PreserveAspectRatio = true;
            panel.TextPadding = 8f;

            Assert.That(panel.FontSize, Is.EqualTo(2f));
            Assert.That(panel.FontColor, Is.EqualTo(Color.Teal));
            Assert.That(panel.BackgroundColor, Is.EqualTo(Color.DarkGray));
            Assert.That(panel.Alignment, Is.EqualTo(TextAlignment.CENTER));
            Assert.That(panel.Font, Is.EqualTo("Monospace"));
            Assert.That(panel.ContentType, Is.EqualTo(ContentType.TEXT_AND_IMAGE));
            Assert.That(panel.PreserveAspectRatio, Is.True);
            Assert.That(panel.TextPadding, Is.EqualTo(8f));
        }

        [Test]
        public void TextPanel_IsFunctionalBlock()
        {
            SimTextPanel panel = SpawnPanel();
            Assert.That(panel.IsWorking, Is.True);
            panel.Enabled = false;
            Assert.That(panel.IsWorking, Is.False);
        }

        [Test]
        public void TextPanel_RenderOnlyMembers_ThrowNamed()
        {
            SimTextPanel panel = SpawnPanel();
            IMyTextSurface surface = panel;

            AssertThrowsNamed(() => surface.SurfaceSize, "IMyTextSurface.SurfaceSize");
            AssertThrowsNamed(() => surface.TextureSize, "IMyTextSurface.TextureSize");
            AssertThrowsNamed(() => surface.CurrentlyShownImage, "IMyTextSurface.CurrentlyShownImage");
            AssertThrowsNamed(() => { var images = new System.Collections.Generic.List<string>(); surface.GetSelectedImages(images); return null; }, "IMyTextSurface.GetSelectedImages()");
            AssertThrowsNamed(() => { var fonts = new System.Collections.Generic.List<string>(); surface.GetFonts(fonts); return null; }, "IMyTextSurface.GetFonts()");
            AssertThrowsNamed(() => surface.MeasureStringInPixels(new StringBuilder("x"), "Debug", 1f), "IMyTextSurface.MeasureStringInPixels()");
            AssertThrowsNamed(() => { using (surface.DrawFrame()) { } return null; }, "IMyTextSurface.DrawFrame()");
        }

        // — helpers —

        private SimRuntimeInfo SpawnRuntime()
        {
            return SpawnPb().Runtime;
        }

        private SimProgrammableBlock SpawnPb()
        {
            SimShip ship = Fc.Spawn(PbTemplate("PB"), SpawnState.At(Vector3D.Zero));
            SimProgrammableBlock pb = ship.TerminalSystem.GetBlockWithName("PB") as SimProgrammableBlock;
            Assert.That(pb, Is.Not.Null);
            return pb;
        }

        private SimTextPanel SpawnPanel()
        {
            SimShip ship = Fc.Spawn(PanelTemplate("Panel"), SpawnState.At(Vector3D.Zero));
            SimTextPanel panel = ship.TerminalSystem.GetBlockWithName("Panel") as SimTextPanel;
            Assert.That(panel, Is.Not.Null);
            return panel;
        }

        private static ShipTemplate PbTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("PB", new Vector3I(0, 0, 0), 100f, 2.5f, kind: BlockKind.ProgrammableBlock),
                });
        }

        private static ShipTemplate PanelTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("Panel", new Vector3I(0, 0, 0), 100f, 2.5f, kind: BlockKind.TextPanel),
                });
        }
    }
}
