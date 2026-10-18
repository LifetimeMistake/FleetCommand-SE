using System;
using System.Collections.Generic;
using System.Text;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRage.ObjectBuilders;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>Base for all simulator blocks, implementing the real <see cref="IMyTerminalBlock"/>
    /// plus its <see cref="IMyCubeBlock"/> / <see cref="IMyEntity"/> parents. The block's position,
    /// orientation, mass, and local transform come from its <see cref="TemplateBlock"/>; its world
    /// transform is derived from the owning grid's rigid body plus that local pose. Custom name and
    /// custom data are settable; grid identity, same-construct identity, functional/working
    /// defaults, and the block's position mask are implemented. Every other member throws a naming
    /// <see cref="NotSupportedException"/> (no plausible defaults).</summary>
    public class SimTerminalBlock : SimEntity, IMyTerminalBlock
    {
        private readonly SimCubeGrid _grid;
        private readonly TemplateBlock _template;
        private readonly MyBlockOrientation _orientation;
        private float _inventoryMass;

        /// <summary>The owning rigid grid (concrete, for scenario use).</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>The template block this instance was spawned from.</summary>
        public TemplateBlock Template => _template;

        /// <inheritdoc />
        public string CustomName { get; set; }

        /// <inheritdoc />
        public string CustomData { get; set; }

        /// <inheritdoc />
        public string DisplayNameText => CustomName;

        /// <summary>Structural integrity: true while the block is intact (no damage model).
        /// Functional blocks override with a setter that also recomputes working state.</summary>
        public virtual bool IsFunctional { get; set; } = true;

        /// <summary>True while the block is able to do its work. For a plain (non-functional)
        /// terminal block this is always true; functional blocks override with the
        /// phase-4-recomputed value.</summary>
        public virtual bool IsWorking => true;

        /// <inheritdoc />
        public IMyCubeGrid CubeGrid => _grid;

        /// <inheritdoc />
        public Vector3I Position => _template.Min;

        /// <inheritdoc />
        public Vector3I Min => _template.Min;

        /// <inheritdoc />
        public Vector3I Max => _template.Min + _template.Size - Vector3I.One;

        /// <summary>Scenario-set inventory (cargo) mass in kilograms, starting from the template's
        /// value; callers must rebuild the grid's body mass afterwards
        /// (<see cref="FleetCommand.Test.Framework.FleetTestContext.SetInventoryMass"/>).</summary>
        public float InventoryMass
        {
            get { return _inventoryMass; }
            set { _inventoryMass = value < 0f ? 0f : value; }
        }

        /// <inheritdoc />
        public float Mass => _template.DryMass + _inventoryMass;

        /// <inheritdoc />
        public MyBlockOrientation Orientation => _orientation;

        /// <inheritdoc />
        public override MatrixD WorldMatrix
        {
            get
            {
                // VRage uses row-vector transforms: local then parent.
                // MyPositionComponentBase.cs:457,567 multiplies local * parentWorld.
                MatrixD gridWorld = _grid.WorldMatrix;
                return (MatrixD)_template.LocalTransform * gridWorld;
            }
        }

        public SimTerminalBlock(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            if (template == null)
            {
                throw new ArgumentNullException(nameof(template));
            }
            _grid = grid;
            _template = template;
            CustomName = template.Name;
            CustomData = string.Empty;
            _inventoryMass = template.InventoryMass;

            Quaternion orientation = template.Orientation;
            _orientation = new MyBlockOrientation(ref orientation);
        }

        /// <inheritdoc />
        public bool IsSameConstructAs(IMyTerminalBlock other)
        {
            // For a single rigid grid, "same construct" equals "same grid";
            // there are no rotors or pistons to follow yet.
            return other != null
                && other.CubeGrid != null
                && other.CubeGrid.EntityId == _grid.EntityId;
        }

        /// <summary>Closes this block (scenario / teardown). Idempotent; closed blocks are skipped
        /// by terminal-system enumeration and the world's tick rather than dereferenced (NRE
        /// guard).</summary>
        public void Dispose()
        {
            IsDisposed = true;
        }

        /// <inheritdoc />
        [Obsolete("Provided for interface conformance; has no effect on non-functional blocks.")]
        public void UpdateIsWorking()
        {
            RecomputeIsWorking();
        }

        /// <summary>Refreshes <see cref="IsWorking"/> from the block's state sources. The base
        /// implementation is a no-op for non-functional blocks; <see cref="SimFunctionalBlock"/>
        /// overrides it with the enablement + functionality + power recompute.</summary>
        protected internal virtual void RecomputeIsWorking()
        {
        }

        // Obsolete aliases of the CustomName setter.

        /// <inheritdoc />
        [Obsolete("Use the CustomName setter.")]
        public void SetCustomName(string text)
        {
            CustomName = text;
        }

        /// <inheritdoc />
        [Obsolete("Use the CustomName setter.")]
        public void SetCustomName(StringBuilder text)
        {
            CustomName = text == null ? null : text.ToString();
        }

        // — IMyCubeBlock members outside the implemented contract —

        /// <inheritdoc />
        public virtual SerializableDefinitionId BlockDefinition => ThrowNotSupported<SerializableDefinitionId>("IMyCubeBlock.BlockDefinition");

        /// <inheritdoc />
        public virtual string DefinitionDisplayNameText => ThrowNotSupported<string>("IMyCubeBlock.DefinitionDisplayNameText");

        /// <inheritdoc />
        public virtual float DisassembleRatio => ThrowNotSupported<float>("IMyCubeBlock.DisassembleRatio");

        /// <inheritdoc />
        public virtual bool IsBeingHacked => ThrowNotSupported<bool>("IMyCubeBlock.IsBeingHacked");

        /// <inheritdoc />
        public virtual int NumberInGrid => ThrowNotSupported<int>("IMyCubeBlock.NumberInGrid");

        /// <inheritdoc />
        public virtual long OwnerId => ThrowNotSupported<long>("IMyCubeBlock.OwnerId");

        /// <inheritdoc />
        public virtual string GetOwnerFactionTag() => ThrowNotSupported<string>("IMyCubeBlock.GetOwnerFactionTag()");

        /// <inheritdoc />
        [Obsolete]
        public virtual MyRelationsBetweenPlayerAndBlock GetPlayerRelationToOwner() => ThrowNotSupported<MyRelationsBetweenPlayerAndBlock>("IMyCubeBlock.GetPlayerRelationToOwner()");

        /// <inheritdoc />
        public virtual MyRelationsBetweenPlayerAndBlock GetUserRelationToOwner(long playerId, MyRelationsBetweenPlayerAndBlock defaultNoUser = MyRelationsBetweenPlayerAndBlock.NoOwnership)
        {
            return ThrowNotSupported<MyRelationsBetweenPlayerAndBlock>("IMyCubeBlock.GetUserRelationToOwner()");
        }

        /// <inheritdoc />
        [Obsolete]
        public virtual void UpdateVisual() => ThrowNotSupportedVoid("IMyCubeBlock.UpdateVisual()");

        // — IMyTerminalBlock members outside the implemented contract —

        /// <inheritdoc />
        public virtual string CustomNameWithFaction => ThrowNotSupported<string>("IMyTerminalBlock.CustomNameWithFaction");

        /// <inheritdoc />
        public virtual string DetailedInfo => ThrowNotSupported<string>("IMyTerminalBlock.DetailedInfo");

        /// <inheritdoc />
        public virtual string CustomInfo => ThrowNotSupported<string>("IMyTerminalBlock.CustomInfo");

        /// <inheritdoc />
        public virtual bool ShowOnHUD
        {
            get => ThrowNotSupported<bool>("IMyTerminalBlock.ShowOnHUD");
            set => ThrowSetNotSupported("IMyTerminalBlock.ShowOnHUD");
        }

        /// <inheritdoc />
        public virtual bool ShowInTerminal
        {
            get => ThrowNotSupported<bool>("IMyTerminalBlock.ShowInTerminal");
            set => ThrowSetNotSupported("IMyTerminalBlock.ShowInTerminal");
        }

        /// <inheritdoc />
        public virtual bool ShowInToolbarConfig
        {
            get => ThrowNotSupported<bool>("IMyTerminalBlock.ShowInToolbarConfig");
            set => ThrowSetNotSupported("IMyTerminalBlock.ShowInToolbarConfig");
        }

        /// <inheritdoc />
        public virtual bool ShowInInventory
        {
            get => ThrowNotSupported<bool>("IMyTerminalBlock.ShowInInventory");
            set => ThrowSetNotSupported("IMyTerminalBlock.ShowInInventory");
        }

        /// <inheritdoc />
        public virtual bool HasLocalPlayerAccess() => ThrowNotSupported<bool>("IMyTerminalBlock.HasLocalPlayerAccess()");

        /// <inheritdoc />
        public virtual bool HasPlayerAccess(long playerId, MyRelationsBetweenPlayerAndBlock defaultNoUser = MyRelationsBetweenPlayerAndBlock.NoOwnership)
        {
            return ThrowNotSupported<bool>("IMyTerminalBlock.HasPlayerAccess()");
        }

        /// <inheritdoc />
        public virtual bool HasNobodyPlayerAccessToBlock() => ThrowNotSupported<bool>("IMyTerminalBlock.HasNobodyPlayerAccessToBlock()");

        /// <inheritdoc />
        public virtual bool HasPlayerAccessWithNobodyCheck(long playerId, bool isForPB = false)
        {
            return ThrowNotSupported<bool>("IMyTerminalBlock.HasPlayerAccessWithNobodyCheck()");
        }

        /// <inheritdoc />
        public virtual void GetActions(List<ITerminalAction> resultList, Func<ITerminalAction, bool> collect = null)
        {
            ThrowNotSupportedVoid("IMyTerminalBlock.GetActions()");
        }

        /// <inheritdoc />
        public virtual void SearchActionsOfName(string name, List<ITerminalAction> resultList, Func<ITerminalAction, bool> collect = null)
        {
            ThrowNotSupportedVoid("IMyTerminalBlock.SearchActionsOfName()");
        }

        /// <inheritdoc />
        public virtual ITerminalAction GetActionWithName(string name) => ThrowNotSupported<ITerminalAction>("IMyTerminalBlock.GetActionWithName()");

        /// <inheritdoc />
        public virtual ITerminalProperty GetProperty(string id) => ThrowNotSupported<ITerminalProperty>("IMyTerminalBlock.GetProperty()");

        /// <inheritdoc />
        public virtual void GetProperties(List<ITerminalProperty> resultList, Func<ITerminalProperty, bool> collect = null)
        {
            ThrowNotSupportedVoid("IMyTerminalBlock.GetProperties()");
        }

        /// <summary>Throws for a void member, naming <paramref name="member"/>.</summary>
        protected static void ThrowNotSupportedVoid(string member)
        {
            throw NotSupported(member);
        }

        /// <summary>Throws for a property setter, naming <paramref name="member"/>.</summary>
        protected static void ThrowSetNotSupported(string member)
        {
            throw NotSupported(member);
        }
    }
}
