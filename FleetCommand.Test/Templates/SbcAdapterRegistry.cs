using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// Discovers every <see cref="ISbcBlockAdapter"/> in the assembly by reflection and
    /// maps subtype ids to the adapter that converts them. Adding a block to the sim is
    /// then the single step of adding its adapter — the SBC→ShipTemplate converter picks
    /// it up automatically (see <see cref="SbcToShipTemplate"/>).
    /// </summary>
    public static class SbcAdapterRegistry
    {
        private static readonly Dictionary<string, ISbcBlockAdapter> BySubtype = Build();

        /// <summary>
        /// Looks up the adapter that converts <paramref name="subtypeId"/>; false when no
        /// implemented block carries that subtype (the converter rejects it).
        /// </summary>
        public static bool TryGet(string subtypeId, out ISbcBlockAdapter adapter)
        {
            return BySubtype.TryGetValue(subtypeId, out adapter);
        }

        /// <summary>The number of distinct subtypes the framework supports (test hook).</summary>
        public static int Count => BySubtype.Count;

        private static Dictionary<string, ISbcBlockAdapter> Build()
        {
            var bySubtype = new Dictionary<string, ISbcBlockAdapter>(StringComparer.Ordinal);

            foreach (Type type in AdapterTypes())
            {
                // Instantiate the adapter and register each subtype it owns.
                ISbcBlockAdapter adapter = (ISbcBlockAdapter)Activator.CreateInstance(type);
                foreach (string subtype in adapter.SubtypeIds)
                {
                    if (bySubtype.ContainsKey(subtype))
                    {
                        throw new InvalidOperationException(
                            $"Duplicate SBC adapter for subtype '{subtype}' (owned by {type.Name} and " +
                            $"{bySubtype[subtype].GetType().Name}).");
                    }
                    bySubtype.Add(subtype, adapter);
                }
            }
            return bySubtype;
        }

        private static IEnumerable<Type> AdapterTypes()
        {
            Type[] types;
            try
            {
                types = typeof(SbcAdapterRegistry).Assembly.GetTypes();
            }
            catch (ReflectionTypeLoadException ex)
            {
                // A referenced assembly failed to load; use whatever types loaded.
                types = ex.Types.Where(t => t != null).ToArray();
            }

            return types.Where(t =>
                t.IsClass
                && !t.IsAbstract
                && typeof(ISbcBlockAdapter).IsAssignableFrom(t)
                && t.GetConstructor(Type.EmptyTypes) != null);
        }
    }
}
