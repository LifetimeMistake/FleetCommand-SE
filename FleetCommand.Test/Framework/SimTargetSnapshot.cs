using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// A guidance target snapshot a test (or the code under test) publishes under a name;
    /// the framework records when it was set and can mark it stale to model a target that
    /// stopped being refreshed. No synthetic noise — staleness is a distinct, deterministic event.
    /// </summary>
    public struct SimTargetSnapshot
    {
        /// <summary>World position of the target.</summary>
        public Vector3D Position;

        /// <summary>World orientation of the target.</summary>
        public QuaternionD Orientation;

        /// <summary>World linear velocity of the target.</summary>
        public Vector3D Velocity;

        /// <summary>True once <see cref="SimTargetStore.StopRefreshing"/> marked it
        /// stale.</summary>
        public bool IsStale;

        /// <summary>The tick on which this snapshot was last set.</summary>
        public long SetTick;

        /// <summary>Builds a target snapshot at <paramref name="position"/>.</summary>
        public static SimTargetSnapshot At(Vector3D position)
        {
            return new SimTargetSnapshot
            {
                Position = position,
                Orientation = QuaternionD.Identity,
                Velocity = Vector3D.Zero,
                IsStale = false,
            };
        }

        /// <summary>A target snapshot with the given velocity.</summary>
        public static SimTargetSnapshot Moving(Vector3D position, Vector3D velocity)
        {
            SimTargetSnapshot snapshot = At(position);
            snapshot.Velocity = velocity;
            return snapshot;
        }
    }

    /// <summary>
    /// A named register of guidance target snapshots owned by the world. Sets replace and
    /// refresh a snapshot (<see cref="IsStale"/> false); <see cref="StopRefreshing"/> marks
    /// one stale without changing its value. Deterministic.
    /// </summary>
    public sealed class SimTargetStore
    {
        private readonly Dictionary<string, SimTargetSnapshot> _targets = new Dictionary<string, SimTargetSnapshot>();

        /// <summary>The number of named targets currently held.</summary>
        public int Count => _targets.Count;

        /// <summary>Publishes (or replaces) <paramref name="name"/> with a fresh
        /// snapshot.</summary>
        public void Set(string name, SimTargetSnapshot snapshot, long tick)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("A target needs a non-empty name.", nameof(name));
            }
            snapshot.SetTick = tick;
            snapshot.IsStale = false;
            _targets[name] = snapshot;
        }

        /// <summary>Marks <paramref name="name"/>'s snapshot stale (a target that stopped being
        /// updated); no-op when unknown.</summary>
        public void StopRefreshing(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                return;
            }
            SimTargetSnapshot snapshot;
            if (_targets.TryGetValue(name, out snapshot))
            {
                snapshot.IsStale = true;
                _targets[name] = snapshot;
            }
        }

        /// <summary>Reads the current snapshot for <paramref name="name"/>.</summary>
        public bool TryGet(string name, out SimTargetSnapshot snapshot)
        {
            return _targets.TryGetValue(name, out snapshot);
        }
    }
}
