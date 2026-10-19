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
    /// <summary>Conformance of the programmable-block runtime, explicit execution hooks, storage,
    /// and text-panel text writes/reads.</summary>
    [TestFixture]
    public class RuntimeProgrammableBlockTextPanelConformanceTests : FleetTestFixture
    {
        [TestCase("disabled")]
        [TestCase("broken")]
        [TestCase("unpowered")]
        public void HostRun_RefusesUnavailableProgrammableBlock(string state)
        {
            var pb = SpawnPb();
            int runs = 0;
            pb.RunHandler = _ => runs++;
            if (state == "disabled") pb.Enabled = false;
            if (state == "broken") pb.IsFunctional = false;
            if (state == "unpowered") { pb.PowerDemand = 1; Fc.Step(); }
            Assert.That(pb.IsWorking, Is.False);
            Assert.That(pb.Run("start", UpdateType.Terminal), Is.False);
            Assert.That(runs, Is.Zero);
        }

        [Test]
        public void ScheduledCadence_CombinesSourcesAndUsesEmptyArgument()
        {
            var pb = SpawnPb();
            pb.TerminalRunArgument = "terminal-only";
            int runs = 0;
            pb.RunHandler = argument => {
                runs++;
                Assert.That(argument, Is.Empty);
                var expected = UpdateType.Update1;
                if (Fc.Tick % 10 == 0) expected |= UpdateType.Update10;
                if (Fc.Tick % 100 == 0) expected |= UpdateType.Update100;
                Assert.That(pb.CurrentUpdateSource, Is.EqualTo(expected));
                if (runs > 1) Assert.That(pb.Runtime.TimeSinceLastRun.Ticks, Is.EqualTo(166666));
                if (runs == 100) pb.Runtime.UpdateFrequency = UpdateFrequency.None;
            };
            pb.Runtime.UpdateFrequency = UpdateFrequency.Update1 | UpdateFrequency.Update10 | UpdateFrequency.Update100;
            Fc.Step(105);
            Assert.That(runs, Is.EqualTo(100));
        }

        [Test]
        public void ScheduledProgram_DoesNotExecuteWhileDisabled()
        {
            var pb = SpawnPb();
            int runs = 0;
            pb.RunHandler = _ => runs++;
            pb.Runtime.UpdateFrequency = UpdateFrequency.Update1;
            pb.Enabled = false;
            Fc.Step(3);
            Assert.That(runs, Is.Zero);
            pb.Enabled = true;
            Fc.Step();
            Assert.That(runs, Is.EqualTo(1));
        }

        [Test]
        public void ScheduledUpdate1_SeesCompletedPhysicsAndWritesForFollowingInterval()
        {
            var ship = Fc.Spawn(new ShipTemplate("pb-phase", 2.5f, new[] {
                new TemplateBlock("PB", Vector3I.Zero, 100, 2.5f, kind: BlockKind.ProgrammableBlock),
                new TemplateBlock("Engine", new Vector3I(2, 0, 0), 100, 2.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 12000),
            }), SpawnState.At(Vector3D.Zero).WithVelocity(Vector3D.UnitX));
            var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("PB");
            var engine = (SimThrust)ship.TerminalSystem.GetBlockWithName("Engine");
            int runs = 0;
            double observedPosition = 0;
            pb.RunHandler = _ => {
                runs++;
                observedPosition = ship.Body.PositionWorld.X;
                engine.ThrustOverridePercentage = 1;
            };
            pb.Runtime.UpdateFrequency = UpdateFrequency.Update1;

            Fc.Step();
            Assert.That(runs, Is.EqualTo(1));
            Assert.That(observedPosition, Is.EqualTo(1.0 / 60).Within(1e-6),
                "MyIngameScriptComponent runs Main after this interval's physics.");
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(1).Within(1e-6),
                "Commands written after physics cannot affect the completed interval.");
            Fc.Step();
            Assert.That(runs, Is.EqualTo(2));
            Assert.That(ship.Body.LinearVelocity.X, Is.EqualTo(2).Within(1e-6));
        }

        [Test]
        public void ScheduledOnce_IsConsumedAndCanRescheduleItself()
        {
            var pb = SpawnPb();
            int runs = 0;
            pb.RunHandler = _ => {
                runs++;
                Assert.That(pb.Runtime.UpdateFrequency, Is.EqualTo(UpdateFrequency.None));
                if (runs == 1) pb.Runtime.UpdateFrequency = UpdateFrequency.Once;
            };
            pb.Runtime.UpdateFrequency = UpdateFrequency.Once;
            Fc.Step(3);
            Assert.That(runs, Is.EqualTo(2));
        }

        [Test]
        public void Runtime_OneFrame_UsesGameTimespanTicksWithoutMillisecondRounding()
        {
            var rt = SpawnRuntime();
            rt.MarkRun();
            Fc.Step();
            Assert.That(rt.TimeSinceLastRun.Ticks, Is.EqualTo(166666L));
        }

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
            Assert.That(rt.TimeSinceLastRun.Ticks, Is.EqualTo(30 * 166666L),
                "TimeSinceLastRun must be the fixed-step interval since the last run — never wall-clock.");
        }

        [Test]
        public void Runtime_TimeSinceLastRun_AdvancesWithEachRun()
        {
            SimRuntimeInfo rt = SpawnRuntime();
            rt.MarkRun(); // tick 0

            Fc.Step(60);
            Assert.That(rt.TimeSinceLastRun.Ticks, Is.EqualTo(60 * 166666L));

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

        // — programmable block: identity and explicit execution —

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
            Assert.That(rt.TimeSinceLastRun.Ticks, Is.EqualTo(30 * 166666L),
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
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo("line one"));

            panel.WriteText("\nline two", append: true);
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo("line one\nline two"),
                "append must add to the existing public text.");

            panel.WriteText("overwritten");
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo("overwritten"),
                "non-append must replace the public text.");
        }

        [Test]
        public void TextPanel_WriteText_StringBuilder_AndReadText_RoundTrip()
        {
            SimTextPanel panel = SpawnPanel();
            Assert.That(panel.WriteText(new StringBuilder("from builder")), Is.True);
            Fc.Step(10);

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
        public void TextPanel_PublicationClipsBothBuffersAt65000()
        {
            SimTextPanel panel = SpawnPanel();
            panel.WriteText(new string('a', 90000));
            Assert.That(panel.GetText(), Is.Empty, "Writes remain pending until publication.");
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo(new string('a', 65000)));
            var read = new StringBuilder();
            panel.ReadText(read);
            Assert.That(read.Length, Is.EqualTo(65000));

            // Publication must truncate the helper too: appending after publication
            // cannot resurrect the previously discarded suffix.
            panel.WriteText("suffix", append: true);
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo(new string('a', 65000)));
            panel.WriteText(new StringBuilder(new string('b', 110000)));
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo(new string('b', 65000)));
            panel.WriteText("replacement");
            Fc.Step(10);
            Assert.That(panel.GetText(), Is.EqualTo("replacement"));
        }

        [Test]
        public void TextPanel_WriteText_CappedAtMaxLength()
        {
            SimTextPanel panel = SpawnPanel();
            panel.WriteText(new string('a', SimTextPanel.MaxTextLength + 5000));
            Fc.Step(10);
            Assert.That(panel.GetText().Length, Is.EqualTo(SimTextPanel.MaxTextLength),
                "Published text must be capped at the game's 65000-character description limit.");
        }

        [Test]
        public void TextPanel_WriteText_StringBuilder_CappedAndTruncatesHead()
        {
            // Both overloads ultimately publish through PublicDescription, which clips
            // to 65000. The separate 100000-character string-input cap is not the
            // observable publication limit (MyTextPanel.cs:209-211, :1310-1312).
            SimTextPanel panel = SpawnPanel();

            // StringBuilder has no pending-input cap, but publication still clips.
            panel.WriteText(new StringBuilder(new string('b', SimTextPanel.MaxTextLength + 5000)));
            Fc.Step(10);
            Assert.That(panel.GetText().Length, Is.EqualTo(SimTextPanel.MaxTextLength),
                "The StringBuilder overload must publish at most 65000 characters.");

            // Publication preserves the FIRST 65000 characters, including the head
            // of the appended value when the existing text is near capacity.
            SimTextPanel panel2 = SpawnPanel();
            panel2.WriteText(new string('x', SimTextPanel.MaxTextLength - 10));
            panel2.WriteText(new string('y', 100), append: true); // 90 over the cap
            Fc.Step(10);
            string text = panel2.GetText();
            Assert.That(text.Length, Is.EqualTo(SimTextPanel.MaxTextLength),
                "append to a near-full panel must publish at most 65000 total.");
            Assert.That(text.Substring(0, 10), Is.EqualTo(new string('x', 10)),
                "The head of the existing text must be preserved.");
            string tail = text.Substring(text.Length - 10);
            Assert.That(tail, Is.EqualTo(new string('y', 10)),
                "The tail must be the FIRST characters of the appended value (the game keeps the head of the write).");
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

        [Test]
        public void TextPanel_OpenEditorRejectsBothWriteOverloadsWithoutChangingText()
        {
            var panel = SpawnPanel();
            panel.WriteText("before"); Fc.Step(10);
            var editor = typeof(SimTextPanel).GetProperty("IsTextPanelOpen");
            Assert.That(editor, Is.Not.Null, "MyTextPanel rejects writes while its text editor is open.");
            editor.SetValue(panel, true, null);
            Assert.That(panel.WriteText("replacement"), Is.False);
            Assert.That(panel.WriteText(new System.Text.StringBuilder("append"), true), Is.False);
            Fc.Step(10); Assert.That(panel.GetText(), Is.EqualTo("before"));
            editor.SetValue(panel, false, null);
            Assert.That(panel.WriteText("after"), Is.True);
            Fc.Step(10); Assert.That(panel.GetText(), Is.EqualTo("after"));
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
