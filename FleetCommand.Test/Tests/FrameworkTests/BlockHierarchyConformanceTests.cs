using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

// Obsolete interface members (SetCustomName, UpdateVisual, ...) are exercised
// deliberately by the conformance suite; the obsolete-call usage warnings
// would otherwise be noise here.
#pragma warning disable CS0612, CS0618

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (, /): the entity →
    /// terminal-block → functional-block hierarchy, terminal-system
    /// discovery (typed lookup and same-grid filtering), working-state
    /// transitions recomputed in tick phase 4, and the rule that every
    /// interface member outside the implemented contract throws a
    /// <see cref="NotSupportedException"/> naming the member.
    /// </summary>
    [TestFixture]
    public class BlockHierarchyConformanceTests : FleetTestFixture
    {
        // — hierarchy surface —

        [Test]
        public void Grid_ExposesEntityAndCubeGridSurfaces()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(new Vector3D(10, 0, 0)));

            IMyEntity entity = ship.Grid as IMyEntity;
            IMyCubeGrid cubeGrid = ship.Grid as IMyCubeGrid;

            Assert.That(entity, Is.Not.Null, "The grid must implement IMyEntity.");
            Assert.That(cubeGrid, Is.Not.Null, "The grid must implement IMyCubeGrid.");
            Assert.That(ship.Grid.EntityId, Is.EqualTo(ship.Id));
            Assert.That(ship.Grid.Name, Is.EqualTo("A"));
            Assert.That(ship.Grid.DisplayName, Is.EqualTo("A"));
            Assert.That(ship.Grid.GridSize, Is.EqualTo(2.5f));
            Assert.That(ship.Grid.GridSizeEnum, Is.EqualTo(VRage.Game.MyCubeSize.Large));
            Assert.That(ship.Grid.IsStatic, Is.False);
            Assert.That(ship.Grid.Closed, Is.False);
            Assert.That(ship.Grid.WorldMatrix.Translation, Is.EqualTo(new Vector3D(10, 0, 0)));
            Assert.That(entity.GetPosition(), Is.EqualTo(new Vector3D(10, 0, 0)));
        }

        [Test]
        public void Block_ExposesTerminalBlockAndBlockSurfaces()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(new Vector3D(10, 0, 0)));
            SimTerminalBlock block = (SimTerminalBlock)ship.TerminalSystem.GetBlockWithName("MainThruster");

            Assert.That(block, Is.Not.Null);
            Assert.That(block, Is.InstanceOf<IMyTerminalBlock>());
            Assert.That(block.EntityId, Is.GreaterThan(0));
            Assert.That(block.Name, Is.EqualTo("MainThruster"));
            Assert.That(block.CustomName, Is.EqualTo("MainThruster"));
            Assert.That(block.DisplayNameText, Is.EqualTo("MainThruster"));
            Assert.That(block.CubeGrid, Is.SameAs(ship.Grid));
            Assert.That(block.Grid, Is.SameAs(ship.Grid));
            Assert.That(block.Position, Is.EqualTo(new Vector3I(0, 0, 0)));
            Assert.That(block.Mass, Is.EqualTo(400f));
        }

        // — terminal-system discovery —

        [Test]
        public void TerminalSystem_TypedLookup_FiltersToThisGrid()
        {
            SimShip shipA = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(new Vector3D(100, 0, 0)));
            SimShip shipB = Fc.Spawn(ThrusterOnlyTemplate("B"), SpawnState.At(new Vector3D(-100, 0, 0)));

            IMyGridTerminalSystem sysA = shipA.TerminalSystem;
            IMyTerminalBlock otherBlocksBlock = (IMyTerminalBlock)shipB.TerminalSystem.GetBlockWithName("MainThruster");

            // GetBlocks must return only this grid's blocks, never ship B's.
            var all = new List<IMyTerminalBlock>();
            sysA.GetBlocks(all);
            Assert.That(all, Has.Count.EqualTo(2));
            foreach (IMyTerminalBlock block in all)
            {
                Assert.That(block.CubeGrid, Is.SameAs(shipA.Grid), "Lookup must not leak blocks from other grids.");
            }
            Assert.That(all, Does.Not.Contain(otherBlocksBlock));

            // Typed enumeration: functional blocks only (the thruster), and
            // terminal blocks (all).
            var functional = new List<IMyTerminalBlock>();
            sysA.GetBlocksOfType<IMyFunctionalBlock>(functional);
            Assert.That(functional, Has.Count.EqualTo(1));
            Assert.That(functional[0].CustomName, Is.EqualTo("MainThruster"));

            var terminal = new List<SimTerminalBlock>();
            sysA.GetBlocksOfType<SimTerminalBlock>(terminal);
            Assert.That(terminal, Has.Count.EqualTo(2));

            var typedFunctional = new List<SimFunctionalBlock>();
            sysA.GetBlocksOfType<SimFunctionalBlock>(typedFunctional);
            Assert.That(typedFunctional, Has.Count.EqualTo(1));

            var predicateFiltered = new List<IMyTerminalBlock>();
            sysA.GetBlocksOfType<IMyTerminalBlock>(predicateFiltered, b => b.CustomName.StartsWith("Main"));
            Assert.That(predicateFiltered, Has.Count.EqualTo(1));

            // Ship B's system sees only B's blocks.
            var bBlocks = new List<IMyTerminalBlock>();
            shipB.TerminalSystem.GetBlocks(bBlocks);
            Assert.That(bBlocks, Has.Count.EqualTo(1));
            Assert.That(bBlocks[0].CubeGrid, Is.SameAs(shipB.Grid));
        }

        [Test]
        public void TerminalSystem_NameAndIdLookup_Exact_CaseInsensitive()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));

            IMyTerminalBlock exact = ship.TerminalSystem.GetBlockWithName("mainthruster");
            Assert.That(exact, Is.Not.Null, "Name lookup must be case-insensitive.");
            Assert.That(exact.CustomName, Is.EqualTo("MainThruster"));

            Assert.That(ship.TerminalSystem.GetBlockWithName("missing"), Is.Null);
            Assert.That(ship.TerminalSystem.GetBlockWithId(1234567L), Is.Null);

            IMyTerminalBlock byId = ship.TerminalSystem.GetBlockWithId(exact.EntityId);
            Assert.That(byId, Is.SameAs(exact));
        }

        [Test]
        public void TerminalSystem_SearchBySubstring()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));

            var matches = new List<IMyTerminalBlock>();
            ship.TerminalSystem.SearchBlocksOfName("thrust", matches);
            Assert.That(matches, Has.Count.EqualTo(1));
            Assert.That(matches[0].CustomName, Is.EqualTo("MainThruster"));
        }

        [Test]
        public void TerminalSystem_CanAccess_OnlySameGrid()
        {
            SimShip shipA = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));
            SimShip shipB = Fc.Spawn(ThrusterOnlyTemplate("B"), SpawnState.At(new Vector3D(50, 0, 0)));

            IMyGridTerminalSystem sysA = shipA.TerminalSystem;
            IMyTerminalBlock ownBlock = sysA.GetBlockWithName("MainThruster");
            IMyTerminalBlock otherBlock = (IMyTerminalBlock)shipB.TerminalSystem.GetBlockWithName("MainThruster");

            Assert.That(sysA.CanAccess(ownBlock), Is.True);
            Assert.That(sysA.CanAccess(otherBlock), Is.False, "A different grid's block must not be accessible.");
            Assert.That(sysA.CanAccess(shipA.Grid), Is.True);
            Assert.That(sysA.CanAccess(shipB.Grid), Is.False, "A different grid must not be accessible.");
            Assert.That(sysA.CanAccess((IMyTerminalBlock)null), Is.False);
        }

        [Test]
        public void BlockGroups_CanBeCreated_AndFoundByName()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));
            SimGridTerminalSystem system = ship.Grid.TerminalSystem;
            SimTerminalBlock thruster = (SimTerminalBlock)ship.TerminalSystem.GetBlockWithName("MainThruster");

            SimBlockGroup group = system.CreateGroup("Engines", new[] { thruster });

            Assert.That(group.Name, Is.EqualTo("Engines"));
            IMyBlockGroup found = system.GetBlockGroupWithName("engines");
            Assert.That(found, Is.SameAs(group), "Group lookup must be case-insensitive.");

            var members = new List<IMyTerminalBlock>();
            group.GetBlocks(members);
            Assert.That(members, Has.Count.EqualTo(1));
            Assert.That(members[0], Is.SameAs(thruster));

            var groups = new List<IMyBlockGroup>();
            system.GetBlockGroups(groups);
            Assert.That(groups, Does.Contain(group));
        }

        // — functional / working / power state transitions —

        [Test]
        public void FunctionalBlock_Default_IsWorking()
        {
            SimFunctionalBlock block = SpawnFunctionalBlock();
            Assert.That(block.Enabled, Is.True);
            Assert.That(block.IsFunctional, Is.True);
            Assert.That(block.IsPowered, Is.True, "An ideal source (no battery) fully powers a functional block.");
            Assert.That(block.IsWorking, Is.True);
        }

        [Test]
        public void FunctionalBlock_EnableToggle_TransitionsWorking()
        {
            SimFunctionalBlock block = SpawnFunctionalBlock();
            int workingEvents = 0;
            int enabledEvents = 0;
            block.WorkingChanged += _ => workingEvents++;
            block.EnabledChanged += _ => enabledEvents++;

            block.Enabled = false;
            Assert.That(block.IsWorking, Is.False, "Disabling a functional block must make it stop working.");
            Assert.That(enabledEvents, Is.EqualTo(1));
            Assert.That(workingEvents, Is.EqualTo(1));

            Fc.Step();
            Assert.That(block.IsWorking, Is.False, "Phase 4 must keep the disabled block non-working.");

            block.Enabled = true;
            Assert.That(block.IsWorking, Is.True, "Re-enabling must restore working state.");
            Assert.That(enabledEvents, Is.EqualTo(2));
            Assert.That(workingEvents, Is.EqualTo(2));
        }

        [Test]
        public void FunctionalBlock_PowerTransition_AppliedInPhase4()
        {
            // Power is a system quantity recomputed in tick phase 4: a
            // supply change alone must take effect only at the next Step.
            // : the ship's battery switching to Recharge cuts the
            // supply; the block's working state must not change until
            // phase 4 applies it (no immediate recompute from the control
            // write).
            ShipTemplate template = new ShipTemplate(
                "T", 2.5f,
                new[]
                {
                    new TemplateBlock("Battery", new Vector3I(0, 0, 0), 400f, 2.5f,
                        kind: BlockKind.Battery, maxOutput: 4f, maxStoredPower: 10f),
                    new TemplateBlock("Load", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                });
            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            SimFunctionalBlock block = (SimFunctionalBlock)ship.TerminalSystem.GetBlockWithName("Load");
            block.PowerDemand = 4f;
            SimBatteryBlock battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");

            Fc.Step();
            Assert.That(block.IsWorking, Is.True, "A powered load works.");

            int workingEvents = 0;
            block.WorkingChanged += _ => workingEvents++;

            battery.ChargeMode = ChargeMode.Recharge; // supply cut
            Assert.That(block.IsWorking, Is.True,
                "A power change alone is applied by phase 4; IsWorking is unchanged until the next Step.");
            Assert.That(workingEvents, Is.EqualTo(0));

            Fc.Step();
            Assert.That(block.IsWorking, Is.False, "Phase 4 must apply the power change.");
            Assert.That(workingEvents, Is.EqualTo(1), "Phase 4 must fire WorkingChanged on the transition.");

            battery.ChargeMode = ChargeMode.Auto; // restore supply
            Fc.Step();
            Assert.That(block.IsWorking, Is.True, "Restoring power must restore working state at the next Step.");
        }

        [Test]
        public void FunctionalBlock_FunctionalityTransition()
        {
            SimFunctionalBlock block = SpawnFunctionalBlock();
            block.IsFunctional = false;
            Assert.That(block.IsWorking, Is.False, "A non-functional block must not work.");
            Fc.Step();
            Assert.That(block.IsWorking, Is.False);
            block.IsFunctional = true;
            Assert.That(block.IsWorking, Is.True);
        }

        [Test]
        public void Grid_Disposed_DoesNotCrashSubsequentSteps()
        {
            // Carry-forward NRE guard: a grid (and its body) disposed while
            // still in the world's collections must not be dereferenced by
            // subsequent Step() calls (, "per-ship removal becomes
            // reachable soon").
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));
            ship.Grid.Dispose();
            Assert.That(ship.Grid.Closed, Is.True);
            Assert.That(ship.Grid.Body.IsDisposed, Is.True);

            Fc.Step();
            Fc.Step(5);

            // The terminal system skips the disposed grid's closed blocks.
            var blocks = new List<IMyTerminalBlock>();
            ship.TerminalSystem.GetBlocks(blocks);
            Assert.That(blocks, Is.Empty, "A disposed grid's closed blocks must not be enumerated.");
        }

        // — unimplemented members throw naming the member —

        [Test]
        public void UnimplementedMembers_ThrowNamedNotSupported()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));
            IMyTerminalBlock block = ship.TerminalSystem.GetBlockWithName("MainThruster");
            IMyCubeGrid cubeGrid = ship.Grid;
            IMyEntity entity = ship.Grid;
            IMySlimBlock slim = cubeGrid.GetCubeBlock(block.Position);
            Assert.That(slim, Is.Not.Null);

            AssertThrowsNamed(() => block.DetailedInfo, "IMyTerminalBlock.DetailedInfo");
            AssertThrowsNamed(() => block.CustomInfo, "IMyTerminalBlock.CustomInfo");
            AssertThrowsNamed(() => block.CustomNameWithFaction, "IMyTerminalBlock.CustomNameWithFaction");
            AssertThrowsNamed(() => block.GetActionWithName("OnOff"), "IMyTerminalBlock.GetActionWithName()");
            AssertThrowsNamed(() => block.GetProperty("any"), "IMyTerminalBlock.GetProperty()");
            AssertThrowsNamed(() => block.HasLocalPlayerAccess(), "IMyTerminalBlock.HasLocalPlayerAccess()");
            AssertThrowsNamed(() => block.ShowOnHUD, "IMyTerminalBlock.ShowOnHUD");
            AssertThrowsNamed(() => block.HasPlayerAccess(1), "IMyTerminalBlock.HasPlayerAccess()");

            AssertThrowsNamed(() => block.BlockDefinition, "IMyCubeBlock.BlockDefinition");
            AssertThrowsNamed(() => block.NumberInGrid, "IMyCubeBlock.NumberInGrid");
            AssertThrowsNamed(() => block.OwnerId, "IMyCubeBlock.OwnerId");
            AssertThrowsNamed(() => { block.UpdateVisual(); return null; }, "IMyCubeBlock.UpdateVisual()");

            AssertThrowsNamed(() => entity.WorldVolume, "IMyEntity.WorldVolume");
            AssertThrowsNamed(() => entity.WorldAABBHr, "IMyEntity.WorldAABBHr");
            AssertThrowsNamed(() => entity.Components, "IMyEntity.Components");
            AssertThrowsNamed(() => entity.HasInventory, "IMyEntity.HasInventory");
            AssertThrowsNamed(() => entity.GetInventory(), "IMyEntity.GetInventory()");

            AssertThrowsNamed(() => slim.BuildIntegrity, "IMySlimBlock.BuildIntegrity");
            AssertThrowsNamed(() => slim.IsDestroyed, "IMySlimBlock.IsDestroyed");
            AssertThrowsNamed(() => slim.ColorMaskHSV, "IMySlimBlock.ColorMaskHSV");
            AssertThrowsNamed(() => { slim.GetMissingComponents(new Dictionary<string, int>()); return null; }, "IMySlimBlock.GetMissingComponents()");
        }

        private static void AssertThrowsNamed(Func<object> accessor, string member)
        {
            try
            {
                accessor();
            }
            catch (NotSupportedException ex)
            {
                Assert.That(ex.Message, Does.Contain(member),
                    "An unimplemented member must throw naming the member '" + member + "', not a silent default.");
                return;
            }
            Assert.Fail("Expected NotSupportedException naming '" + member + "', but the member did not throw.");
        }

        // — same-grid / construct identity —

        [Test]
        public void ConstructIdentity_SameGridOnly()
        {
            SimShip shipA = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(Vector3D.Zero));
            SimShip shipB = Fc.Spawn(ThrusterOnlyTemplate("B"), SpawnState.At(new Vector3D(30, 0, 0)));

            SimTerminalBlock a1 = (SimTerminalBlock)shipA.TerminalSystem.GetBlockWithName("MainThruster");
            SimTerminalBlock a2 = (SimTerminalBlock)shipA.TerminalSystem.GetBlockWithName("Payload");
            SimTerminalBlock b1 = (SimTerminalBlock)shipB.TerminalSystem.GetBlockWithName("MainThruster");

            Assert.That(a1.IsSameConstructAs(a2), Is.True, "Blocks on the same grid are the same construct.");
            Assert.That(a1.IsSameConstructAs(b1), Is.False, "Blocks on different grids are different constructs.");
            Assert.That(shipA.Grid.IsSameConstructAs(shipA.Grid), Is.True);
            Assert.That(shipA.Grid.IsSameConstructAs(shipB.Grid), Is.False);
        }

        // — block membership and position mapping —

        [Test]
        public void Grid_BlockMembership_AndCoordinateMapping()
        {
            SimShip ship = Fc.Spawn(TwoBlockTemplate("A"), SpawnState.At(new Vector3D(100, 200, 300)));
            SimTerminalBlock thruster = (SimTerminalBlock)ship.TerminalSystem.GetBlockWithName("MainThruster");
            Vector3I cell = thruster.Position;

            Assert.That(ship.Grid.CubeExists(cell), Is.True);
            Assert.That(ship.Grid.CubeExists(cell + new Vector3I(0, 5, 0)), Is.False);

            IMySlimBlock slim = ship.Grid.GetCubeBlock(cell);
            Assert.That(slim, Is.Not.Null);
            Assert.That(slim.FatBlock, Is.SameAs(thruster));
            Assert.That(slim.Position, Is.EqualTo(cell));
            Assert.That(slim.Mass, Is.EqualTo(thruster.Mass));

            // Grid cells map to world space through the grid matrix; a block
            // stands at the world transform of its local pose.
            Vector3D cellWorld = ship.Grid.GridIntegerToWorld(cell);
            Assert.That(Vector3D.Distance(cellWorld, thruster.GetPosition()), Is.LessThan(1e-3),
                "The grid's cell-to-world mapping must agree with the block's derived world transform.");
            Assert.That(ship.Grid.WorldToGridInteger(thruster.GetPosition()), Is.EqualTo(cell));
        }

        // — helpers —

        private SimFunctionalBlock SpawnFunctionalBlock()
        {
            SimShip ship = Fc.Spawn(ThrusterOnlyTemplate("T"), SpawnState.At(Vector3D.Zero));
            IMyTerminalBlock found = ship.TerminalSystem.GetBlockWithName("MainThruster");
            SimFunctionalBlock functional = found as SimFunctionalBlock;
            Assert.That(functional, Is.Not.Null);
            return functional;
        }

        private static ShipTemplate ThrusterOnlyTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("MainThruster", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                });
        }

        private static ShipTemplate TwoBlockTemplate(string name)
        {
            return new ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new TemplateBlock("MainThruster", new Vector3I(0, 0, 0), 400f, 2.5f, kind: BlockKind.FixedThruster),
                    new TemplateBlock("Payload", new Vector3I(3, 0, 0), 100f, 2.5f),
                });
        }
    }
}
