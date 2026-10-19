using System;
using System.IO;
using System.Reflection;

namespace FleetCommand.Test.Harness
{
    /// <summary>
    /// Resolves every assembly in the Space Engineers runtime set by simple name
    /// from a single directory, so <c>HavokWrapper</c>, <c>VRage.*</c> and
    /// <c>Sandbox.*</c> all load from the same folder with no version mixing.
    /// </summary>
    public static class SeReferenceResolver
    {
        private static bool s_registered;
        private static readonly object s_lock = new object();

        /// <summary>
        /// Registers an <see cref="AppDomain.AssemblyResolve"/> handler that loads
        /// any simple-named assembly from the directory containing
        /// <c>HavokWrapper</c>. Idempotent; safe to call more than once.
        /// </summary>
        public static void EnsureRegistered()
        {
            lock (s_lock)
            {
                if (s_registered)
                {
                    return;
                }

                string directory = GameBinDirectory();
                AppDomain.CurrentDomain.AssemblyResolve += (sender, args) =>
                {
                    string simpleName = new AssemblyName(args.Name).Name;
                    string path = Path.Combine(directory, simpleName + ".dll");
                    return File.Exists(path) ? Assembly.LoadFrom(path) : null;
                };

                s_registered = true;
            }
        }

        /// <summary>
        /// The single game directory the whole runtime set is served from: the
        /// folder that <c>HavokWrapper.dll</c> itself was loaded out of (the
        /// test output directory, where the entire set is copied local).
        /// </summary>
        private static string GameBinDirectory()
        {
            Assembly havok = typeof(global::Havok.HkBaseSystem).Assembly;
            string location = havok.Location;
            return string.IsNullOrEmpty(location)
                ? Directory.GetCurrentDirectory()
                : Path.GetDirectoryName(location);
        }
    }
}
