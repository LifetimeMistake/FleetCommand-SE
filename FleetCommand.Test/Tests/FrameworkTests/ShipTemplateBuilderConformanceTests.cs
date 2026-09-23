using System;
using System.Collections.Generic;
using System.Linq;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using Sandbox.ModAPI.Ingame;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the validating ShipTemplateBuilder, the seven baseline templates,
    /// and the immutability / variant / determinism guarantees.</summary>
    [TestFixture]
    public class ShipTemplateBuilderConformanceTests : FleetTestFixture
    {
        // — 1. each of the seven templates builds and spawns, physically consistent —

        private static IEnumerable<TestCaseData> AllTemplates()
        {
            yield return new TestCaseData("SixAxisCraft", (Func<ShipTemplate>)(() => ShipTemplates.SixAxisCraft));
            yield return new TestCaseData("AnisotropicCraft", (Func<ShipTemplate>)(() => ShipTemplates.AnisotropicCraft));
            yield return new TestCaseData("ForwardMissile", (Func<ShipTemplate>)(() => ShipTemplates.ForwardMissile));
            yield return new TestCaseData("NoRetroCraft", (Func<ShipTemplate>)(() => ShipTemplates.NoRetroCraft));
            yield return new TestCaseData("OrbitGunship", (Func<ShipTemplate>)(() => ShipTemplates.OrbitGunship));
            yield return new TestCaseData("DockingCraft", (Func<ShipTemplate>)(() => ShipTemplates.DockingCraft));
            yield return new TestCaseData("HeavyAsymmetricCraft", (Func<ShipTemplate>)(() => ShipTemplates.HeavyAsymmetricCraft));
        }

        [TestCaseSource(nameof(AllTemplates))]
        public void Template_BuildsAndSpawns_PhysicallyConsistent(string name, Func<ShipTemplate> factory)
        {
            ShipTemplate template = factory();
            Assert.That(template.Blocks.Count, Is.GreaterThan(0), $"{name} must have blocks.");
            Assert.That(template.DryMass, Is.GreaterThan(0f), $"{name} must have positive physical mass.");

            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));

            // Every declared block is present on the spawned grid, by name.
            foreach (TemplateBlock block in template.Blocks)
            {
                Assert.That(ship.TerminalSystem.GetBlockWithName(block.Name), Is.Not.Null,
                    $"{name} spawn must carry block '{block.Name}'.");
            }

            // Physically consistent: positive body mass and a finite position
            // after advancing the simulation.
            Assert.That(ship.Grid.Body.Mass, Is.GreaterThan(0f), $"{name} must have positive body mass.");
            Assert.That(IsFinite(ship.Body.PositionWorld), Is.True, $"{name} must spawn at a finite position.");

            Fc.RunFor(0.5);
            Assert.That(IsFinite(ship.Body.PositionWorld), Is.True,
                $"{name} must remain finite after integration (no NaN drift).");
        }

        // — 2. validation rejects bad input, one ArgumentException per rule —

        [Test]
        public void Build_Rejects_DuplicateBlockNames()
        {
            var b = ShipTemplateBuilder.LargeGrid("dup")
                .AddStandard("A", Vector3I.Zero, 1000f)
                .AddStandard("A", new Vector3I(2, 0, 0), 1000f);
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("'A'"), "The duplicate block name must be named.");
        }

        [Test]
        public void Build_Rejects_NegativeMass()
        {
            var b = ShipTemplateBuilder.LargeGrid("neg").AddStandard("B", Vector3I.Zero, -5f);
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("'B'"), "The offending block must be named.");
        }

        [Test]
        public void Build_Rejects_ZeroMass()
        {
            var b = ShipTemplateBuilder.LargeGrid("zero").AddStandard("C", Vector3I.Zero, 0f);
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("'C'"), "The offending block must be named.");
        }

        [Test]
        public void Build_Rejects_NonCardinalThrusterDirection()
        {
            var b = ShipTemplateBuilder.LargeGrid("nc")
                .AddThrusterGroup("T", new Vector3I(1, 1, 0), 1, 100000f);
            Assert.Throws<ArgumentException>((Action)(() => b.Build()));
        }

        [Test]
        public void Build_Rejects_MissingDesignatedController()
        {
            var b = ShipTemplateBuilder.LargeGrid("mc")
                .AddStandard("H", Vector3I.Zero, 1000f)
                .WithController("Ghost");
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("Ghost"), "The missing controller must be named.");
        }

        [Test]
        public void Build_Rejects_DesignatedNonController()
        {
            var b = ShipTemplateBuilder.LargeGrid("dnc")
                .AddStandard("H", Vector3I.Zero, 1000f)
                .AddStandard("Hood", new Vector3I(1, 0, 0), 1000f)
                .WithController("Hood");
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("not a controller"), "A designated controller must be a controller kind.");
        }

        [Test]
        public void Build_Rejects_NonFiniteOrientation()
        {
            var b = ShipTemplateBuilder.LargeGrid("nft")
                .AddStandard("D", Vector3I.Zero, 1000f, orientation: new Quaternion(float.NaN, 0f, 0f, 0f));
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("'D'"), "The offending block must be named.");
        }

        [Test]
        public void Build_Rejects_BlocksOutsideSupportedModel()
        {
            var b = ShipTemplateBuilder.LargeGrid("oor")
                .AddStandard("E", new Vector3I(20000, 0, 0), 1000f);
            var ex = Assert.Throws<ArgumentException>((Action)(() => b.Build()));
            Assert.That(ex.Message, Does.Contain("'E'"), "The offending block must be named.");
        }

        // — 3. immutability across spawns —

        [Test]
        public void Spawns_AreIndependent_AndDoNotMutateTemplate()
        {
            ShipTemplate template = ShipTemplates.SixAxisCraft;
            SimShip a = Fc.Spawn(template, SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(template, SpawnState.At(new Vector3D(0, 0, 500)));

            // Identical block-name sets on both spawns.
            string[] aNames = BlockNames(a).OrderBy(x => x, StringComparer.Ordinal).ToArray();
            string[] bNames = BlockNames(b).OrderBy(x => x, StringComparer.Ordinal).ToArray();
            Assert.That(aNames, Is.EqualTo(bNames), "Both spawns of one template must have identical block sets.");

            // Entity IDs are assigned at spawn, so the two spawns share no IDs.
            Assert.That(aNames.Length, Is.GreaterThan(0));
            foreach (string name in aNames)
            {
                IMyTerminalBlock ab = a.TerminalSystem.GetBlockWithName(name);
                IMyTerminalBlock bb = b.TerminalSystem.GetBlockWithName(name);
                Assert.That(ab.EntityId, Is.Not.EqualTo(bb.EntityId),
                    $"Entity IDs must be per-spawn ({name}).");
            }

            // Mutating one spawn's block must not affect the template or the
            // other spawn (template block names are unchanged, other spawn
            // unchanged).
            string original = "Pilot";
            a.TerminalSystem.GetBlockWithName(original).CustomName = "Renamed";
            Assert.That(b.TerminalSystem.GetBlockWithName(original), Is.Not.Null,
                "Mutating one spawn must not change the other spawn.");
            Assert.That(template.Blocks.Any(x => x.Name == original), Is.True,
                "Mutating a spawn must not mutate the template's block list.");
        }

        // — 4. variant derivation through the builder (no craft-type branch) —

        [Test]
        public void DerivedVariant_NoRetro_HasForwardButNoReverseThrust()
        {
            // Derive a no-retro craft from SixAxisCraft by removing its
            // reverse-thruster block — behavior must follow template data.
            ShipTemplateBuilder builder = ShipTemplateBuilder.From(ShipTemplates.SixAxisCraft);
            builder.RemoveBlock("Rev");
            ShipTemplate variant = builder.Build();

            // Contrast: the unmodified six-axis craft keeps its retro.
            SimShip full = Fc.Spawn(ShipTemplates.SixAxisCraft, SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip noRetro = Fc.Spawn(variant, SpawnState.At(new Vector3D(0, 0, 400)));

            // No-craft-type-branch check. Forward command: both accelerate +X
            // (the variant's Fwd thrusters push +X from template data).
            Controlled(noRetro).DampenersOverride = false;
            Controlled(noRetro).MoveIndicator = new Vector3(1, 0, 0);
            Controlled(noRetro).IsUnderControl = true;
            Fc.RunFor(0.5);
            Assert.That(noRetro.Body.LinearVelocity.X, Is.GreaterThan(5f),
                "A derived no-retro variant must still accelerate forward (data-driven).");

            // Reverse command with dampeners off: the no-retro variant has no
            // reverse thrust, so it must not accelerate aft; the full craft
            // (which has retro thrusters) must.
            ResetVelocity(noRetro);
            Controlled(noRetro).MoveIndicator = new Vector3(-1, 0, 0);
            Controlled(full).DampenersOverride = false;
            Controlled(full).MoveIndicator = new Vector3(-1, 0, 0);
            Controlled(full).IsUnderControl = true;

            Fc.RunFor(0.5);

            Assert.That(full.Body.LinearVelocity.X, Is.LessThan(-0.5f),
                "A craft with retro thrusters must decelerate under a reverse command.");
            Assert.That(noRetro.Body.LinearVelocity.X, Is.EqualTo(0f).Within(0.05),
                "A no-retro variant must produce no reverse thrust (no craft-type branch, purely template data).");
        }

        // — 5. deterministic across repeated runs —

        [Test]
        public void Template_SpawnAndDrive_Deterministic_AcrossTwoWorlds()
        {
            (double a, double b) = DeterminismHelper.RunTwice(RunForwardThenMeasure);
            Assert.That(a, Is.EqualTo(b).Within(1e-9),
                "The driven final speed must be identical across two independent worlds.");
        }

        private static double RunForwardThenMeasure(FleetTestContext ctx)
        {
            SimShip ship = ctx.Spawn(ShipTemplates.SixAxisCraft, SpawnState.At(Vector3D.Zero));
            Controlled(ship).IsUnderControl = true;
            Controlled(ship).DampenersOverride = false;
            Controlled(ship).MoveIndicator = new Vector3(1, 0, 0);
            ctx.RunFor(1.0);
            return ship.Body.LinearVelocity.X;
        }

        // — helpers —

        private static SimShipController Controlled(SimShip ship)
        {
            return (SimShipController)ship.Controller;
        }

        private static IEnumerable<string> BlockNames(SimShip ship)
        {
            var blocks = new List<IMyTerminalBlock>();
            ship.TerminalSystem.GetBlocks(blocks);
            return blocks.Select(x => x.CustomName);
        }

        private static void ResetVelocity(SimShip ship)
        {
            ship.SetState(ship.Body.WithLinearVelocity(Vector3D.Zero));
        }

        private static bool IsFinite(Vector3D v)
        {
            return IsFinite(v.X) && IsFinite(v.Y) && IsFinite(v.Z);
        }

        private static bool IsFinite(double d)
        {
            return !double.IsNaN(d) && !double.IsInfinity(d);
        }
    }
}
