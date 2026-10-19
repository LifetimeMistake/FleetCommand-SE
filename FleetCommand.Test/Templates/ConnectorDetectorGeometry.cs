using System;
using VRageMath;

namespace FleetCommand.Test.Templates
{
    /// <summary>Detector dummy matrices extracted from the installed 1.210.014 MWM
    /// assets, using MyModelImporter's Dummies layout. No measured dynamics are encoded.</summary>
    internal static class ConnectorDetectorGeometry
    {
        internal static Matrix ForSubtype(string subtype)
        {
            switch (subtype)
            {
                // Models/Cubes/Small/Connector.mwm: detector_small_connector
                case "ConnectorSmall": return new Matrix(
                    0.276796341f, 0, 0, 0,
                    0, 0.276796341f, -1.20991537E-08f, 0,
                    0, 5.86923132E-09f, 0.134272367f, 0,
                    -0.0006716348f, 1.64839822E-08f, -0.335196674f, 1);
                // Models/Cubes/Large/Connector.mwm: detector_Connector_001
                case "Connector": return new Matrix(
                    1.2310884f, 0, 0, 0,
                    0, -1.15358006E-07f, 0.9676931f, 0,
                    0, -1.2310884f, -1.46757188E-07f, 0,
                    0, -5.80822324E-08f, -1.31683457f, 1);
                // Models/Cubes/Small/ConnectorMedium.mwm: detector_connector_001
                case "ConnectorMedium": return new Matrix(
                    1.24357367f, 0, 0, 0,
                    0, 1.27212715f, -5.56064421E-08f, 0,
                    0, 2.30376624E-08f, 0.527040243f, 0,
                    -2.60669822E-07f, -2.25294752E-08f, -0.5154143f, 1);
                // Models/Cubes/Small/InsetConnector.mwm: detector_small_connector
                case "SmallBlockInsetConnector": return new Matrix(
                    0.276796341f, 0, 0, 0,
                    0, 0.276796341f, -1.20991537E-08f, 0,
                    0, 5.86923132E-09f, 0.134272367f, 0,
                    -3.81469718E-08f, 2.4687008E-08f, -0.3352f, 1);
                // Models/Cubes/Large/InsetConnector.mwm: detector_Connector_001
                case "LargeBlockInsetConnector": return new Matrix(
                    1.2310884f, 0, 0, 0,
                    0, -1.15358006E-07f, 0.9676931f, 0,
                    0, -1.2310884f, -1.46757188E-07f, 0,
                    0, -2.10670123E-07f, -1.32876658f, 1);
                default: throw new ArgumentException("Unknown connector detector model: " + subtype);
            }
        }
    }
}
