using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Common
{
    public static class TimeSource
    {
        public static TimeSpan Timestamp => _timestamp;
        private static TimeSpan _timestamp;

        public static void UpdateTime(TimeSpan timestamp)
        {
            _timestamp = timestamp;
        }
    }
}
