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
                if (args.Length == 0 || args[0] == "run")
                {
                    string output = args.Length > 1 ? args[1] : "artifacts/simulator-report.txt";
                    string blueprint = args.Length > 2 ? args[2] : SimulationRun.DefaultBlueprint;
                    var report = SimulationRun.Execute(blueprint, new Vector3D(12500, -3600, 8100),
                        MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2));
                    Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)));
                    File.WriteAllText(output, report);
                    Console.WriteLine("LCD exported: " + Path.GetFullPath(output) + " (" + report.Length + " characters)");
                    string end = "";
                    foreach (var line in report.Split('\n')) if (line.StartsWith("END|")) end = line;
                    Console.WriteLine(end);
                    return end.StartsWith("END|COMPLETE|") ? 0 : 1;
                }
                Console.Error.WriteLine("Usage: FleetCommand.Validation.Launcher.exe run [report.txt] [bp.sbc]\n       FleetCommand.Validation.Launcher.exe compare simulator.txt game.txt");
                return 2;
            }
            catch (Exception error) { Console.Error.WriteLine(error); return 2; }
            finally { PhysicsHost.Shutdown(); }
        }
    }
}
