namespace FleetCommand.MCP.World
{
    public sealed class GridSnapshot
    {
        public long EntityId { get; set; }

        public string Name { get; set; }

        public bool IsStatic { get; set; }

        public double X { get; set; }

        public double Y { get; set; }

        public double Z { get; set; }
    }
}
