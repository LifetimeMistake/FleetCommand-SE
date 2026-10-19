using System;
using System.IO;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Validation.Launcher
{
    public static class Program
    {
        public static int Main(string[] args)
        {
            try
            {
                if (args.Length == 3 && args[0] == "compare")
                {
                    var result = ReportComparison.Compare(File.ReadAllText(args[1]), File.ReadAllText(args[2]));
                    Console.WriteLine(result);
                    return result.Success ? 0 : 1;
                }
                if (args.Length == 0 || args[0] == "run" || args[0] == "restprobe" || args[0] == "flightprobe" || args[0] == "dockprobe")
                {
                    int positionalCount = args.Length;
                    MatrixD? referenceOrientation = null;
                    if (args.Length >= 3 && args[args.Length - 2] == "--frame-from")
                    {
                        referenceOrientation = ReportInitialOrientation.Read(File.ReadAllText(args[args.Length - 1]));
                        positionalCount -= 2;
                    }
                    if (positionalCount > 3) throw new ArgumentException("Unexpected run arguments.");
                    string output = positionalCount > 1 ? args[1] : "artifacts/simulator-report.txt";
                    string command = args.Length == 0 || args[0] == "run" ? "start" : args[0];
                    string blueprint = positionalCount > 2 ? args[2]
                        : command == "dockprobe" ? SimulationRun.StationBlueprint : SimulationRun.DefaultBlueprint;
                    var report = SimulationRun.Execute(blueprint, new Vector3D(12500, -3600, 8100),
                        MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2),
                        command: command, referenceOrientation: referenceOrientation,
                        additionalArtifact: (kind, data) => {
                            if (data.Length == 0) return;
                            string path = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(output)),
                                Path.GetFileNameWithoutExtension(output) + "." + kind + Path.GetExtension(output));
                            Directory.CreateDirectory(Path.GetDirectoryName(path));
                            File.WriteAllText(path, data);
                            Console.WriteLine("Additional LCD exported: " + path + " (" + data.Length + " characters)");
                        });
                    Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)));
                    File.WriteAllText(output, report);
                    Console.WriteLine("LCD exported: " + Path.GetFullPath(output) + " (" + report.Length + " characters)");
                    string end = "";
                    foreach (var line in report.Split('\n')) if (line.StartsWith("END|")) end = line;
                    Console.WriteLine(end);
                    return end.StartsWith("END|COMPLETE|") ? 0 : 1;
                }
                Console.Error.WriteLine("Usage: FleetCommand.Validation.Launcher.exe run|restprobe|flightprobe|dockprobe [report.txt] [bp.sbc] [--frame-from game-report.txt]\n       FleetCommand.Validation.Launcher.exe compare simulator.txt game.txt");
                return 2;
            }
            catch (Exception error) { Console.Error.WriteLine(error); return 2; }
            finally { PhysicsHost.Shutdown(); }
        }
    }
}
