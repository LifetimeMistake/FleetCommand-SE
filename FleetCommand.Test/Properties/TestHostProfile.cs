using System;
using NUnit.Framework;

// Physics tests run serially on a single worker: Havok is process-global
// state, so nothing may execute in parallel with it.
[assembly: NonParallelizable]
[assembly: LevelOfParallelism(1)]
