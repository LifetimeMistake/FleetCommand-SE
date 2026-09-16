using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Observations;
using Havok;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// The isolated simulation world: a fixed 1/60 s tick pipeline over one
    /// owned <see cref="HkWorld"/>, plus the deterministic clock, scheduler,
    /// and caller-visible scenario controls. One world per test; it owns every
    /// Havok object it creates and disposes them all on <see cref="Dispose"/>
    /// ().  added the rigid bodies (one per grid). 
    /// adds the grids and their blocks: the world owns each
    /// <see cref="SimCubeGrid"/> (with its block stack and terminal
    /// system), recomputes block <c>IsWorking</c> in phase 4, and carries
    /// the NRE guard — a grid or block disposed mid-world while still in the
    /// world's collections is skipped by subsequent steps rather than
    /// dereferencing disposed natives.
    /// </summary>
    public sealed class SimWorld : IDisposable
    {
        /// <summary>The deterministic clock; time is derived from ticks, never wall-clock.</summary>
        public TickClock Clock { get; }

        /// <summary>The active physics compatibility profile.</summary>
        public SePhysicsProfile Profile { get; }

        /// <summary>
        /// Fixed scenario origin for numeric locality. World-visible positions
        /// are computed from this origin plus a local float offset ().
        /// </summary>
        public Vector3D ScenarioOrigin { get; }

        /// <summary>
        /// Natural gravity field owned by this world (). Defaults to
        /// <see cref="SimGravityField.Zero"/>; tests set it per scenario.
        /// </summary>
        public SimGravityField Gravity { get; private set; }

        /// <summary>
        /// The shared "short failure ring buffer" (): the one
        /// always-on recorder. Each tick it retains a short rolling window
        /// of recent actuator history (per grid), and an uncaught
        /// component exception is added to it. The fixture surfaces it on
        /// failure.
        /// </summary>
        public FailureRingBuffer FailureLog { get; }

        /// <summary>
        /// A short rolling log of recent Update1 callback invocations —
        /// callback cadence and exact command-application intervals
        /// (), kept automatically for observation and failure
        /// output.
        /// </summary>
        public UpdateCadenceLog UpdateCadence { get; }

        /// <summary>The number of opt-in time-series recorders currently sampling.</summary>
        public int RecorderCount => _samplers.Count;

        /// <summary>
        /// Environmental atmosphere effectiveness in [0, 1] ():
        /// the multiplier applied to every thruster's template
        /// effectiveness when aggregating delivered thrust (an atmospheric
        /// thruster in vacuum has effectiveness 0). Default 1.
        /// </summary>
        public float AtmosphereEffectiveness { get; private set; }

        /// <summary>The world's named guidance-target snapshot register ().</summary>
        public SimTargetStore Targets { get; }

        /// <summary>The rigid bodies the world owns, in deterministic insertion (spawn) order.</summary>
        public IReadOnlyList<SimRigidBody> Bodies => _bodies;

        /// <summary>The grids the world owns, in deterministic spawn order.</summary>
        public IReadOnlyList<SimCubeGrid> Grids => _grids;

        private readonly HkWorld _hkWorld;
        private readonly List<SimRigidBody> _bodies = new List<SimRigidBody>();
        private readonly List<SimCubeGrid> _grids = new List<SimCubeGrid>();
        private readonly List<SimFunctionalBlock> _functionalBlocks = new List<SimFunctionalBlock>();
        private readonly List<SimShipConnector> _connectors = new List<SimShipConnector>();
        private readonly List<ITickSampler> _samplers = new List<ITickSampler>();
        private long _nextEntityId = 1;
        private bool _disposed;

        // Names of the phases executed by the most recent Step(), in order.
        // Exposed so the nine-phase ordering is observable to conformance
        // tests and to later stages as they fill the bodies in.
        private readonly List<string> _phaseTrace = new List<string>();

        // Scheduler: scheduled scenario mutations (phase 2) and per-Update1
        // callbacks (phase 3).
        private readonly Dictionary<long, List<Action>> _scheduled = new Dictionary<long, List<Action>>();
        private readonly List<UpdateCallback> _updateCallbacks = new List<UpdateCallback>();
        private bool _suppressNextUpdate1;

        private static readonly string[] PhaseNames =
        {
            "PublishCompletedState",
            "ScheduledMutations",
            "ComponentCallbacks",
            "RecomputePower",
            "AggregateForce",
            "ApplyImpulses",
            "AdvancePhysics",
            "SynchronizeState",
            "SampleAndEvaluate",
        };

        public SimWorld()
            : this(SePhysicsProfile.Default, Vector3D.Zero)
        {
        }

        public SimWorld(SePhysicsProfile profile, Vector3D scenarioOrigin)
        {
            Clock = new TickClock();
            Profile = profile ?? SePhysicsProfile.Default;
            ScenarioOrigin = scenarioOrigin;
            Gravity = SimGravityField.Zero;
            FailureLog = new FailureRingBuffer(24);
            UpdateCadence = new UpdateCadenceLog(24);
            AtmosphereEffectiveness = 1f;
            Targets = new SimTargetStore();

            // Mirror the game's single-threaded world creation (CreateWorldCInfo)
            // for a no-global-gravity, deterministic host.
            _hkWorld = new HkWorld(
                enableGlobalGravity: false,
                broadphaseCubeSideLength: 100000f,
                contactRestingVelocity: float.MaxValue,
                enableMultithreading: false,
                solverIterations: 8);
        }

        /// <summary>The names of the phases executed by the last <see cref="Step"/>.</summary>
        public IReadOnlyList<string> LastStepPhases => _phaseTrace;

        /// <summary>True once <see cref="Dispose"/> has run.</summary>
        public bool IsDisposed => _disposed;

        /// <summary>Replaces the world's natural gravity field.</summary>
        public void SetGravity(SimGravityField gravity)
        {
            Gravity = gravity ?? SimGravityField.Zero;
        }

        /// <summary>
        /// Sets the environmental atmosphere effectiveness in [0, 1]
        /// ( "change atmosphere effectiveness"). Clamped; 0 means
        /// no atmospheric thrust. Applied from the next tick's aggregation.
        /// </summary>
        public void SetAtmosphereEffectiveness(float ratio)
        {
            AtmosphereEffectiveness = ratio < 0f ? 0f : ratio > 1f ? 1f : ratio;
        }

        /// <summary>Registers an opt-in time-series recorder to sample each tick (phase 9).</summary>
        internal void RegisterSampler(ITickSampler sampler)
        {
            if (sampler == null)
            {
                throw new ArgumentNullException(nameof(sampler));
            }
            _samplers.Add(sampler);
        }

        /// <summary>Allocates the next stable entity id (ascending, deterministic).</summary>
        internal long NextEntityId()
        {
            return _nextEntityId++;
        }

        /// <summary>Registers a rigid body with the world (spawn). Deterministic insertion order.</summary>
        internal void AddBody(SimRigidBody body)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body));
            }
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(SimWorld));
            }
            body.AddToWorld(_hkWorld);
            _bodies.Add(body);
        }

        /// <summary>Registers a grid (with its block stack) with the world (spawn).</summary>
        internal void AddGrid(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(SimWorld));
            }
            _grids.Add(grid);
        }

        /// <summary>Registers a functional block so phase 4 can recompute its working state each tick.</summary>
        internal void RegisterFunctionalBlock(SimFunctionalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            _functionalBlocks.Add(block);
        }

        /// <summary>Registers a connector so phase 5 can evaluate docking capture each tick.</summary>
        internal void RegisterConnector(SimShipConnector connector)
        {
            if (connector == null)
            {
                throw new ArgumentNullException(nameof(connector));
            }
            _connectors.Add(connector);
        }

        /// <summary>Removes a functional block from the phase-4 recompute list (scenario block removal).</summary>
        internal void UnregisterFunctionalBlock(SimFunctionalBlock block)
        {
            if (block != null)
            {
                _functionalBlocks.Remove(block);
            }
        }

        /// <summary>Removes a connector from the phase-5 capture registry (scenario block removal).</summary>
        internal void UnregisterConnector(SimShipConnector connector)
        {
            if (connector != null)
            {
                _connectors.Remove(connector);
            }
        }

        /// <summary>Adds a rigid constraint (e.g. a connector's fixed constraint) to the world.</summary>
        internal void AddConstraint(HkConstraint constraint)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _hkWorld.AddConstraint(constraint);
            }
        }

        /// <summary>Removes a rigid constraint from the world.</summary>
        internal void RemoveConstraint(HkConstraint constraint)
        {
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                if (constraint.InWorld)
                {
                    _hkWorld.RemoveConstraint(constraint);
                }
            }
        }

        /// <summary>
        /// Runs one fixed 1/60 s interval through the nine-phase pipeline:
        /// publish completed state, run scheduled mutations, invoke due
        /// callbacks, recompute power, aggregate forces, apply impulses,
        /// advance Havok, synchronize state, sample and evaluate.
        /// </summary>
        public void Step()
        {
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(SimWorld));
            }

            _phaseTrace.Clear();

            Phase1_PublishCompletedState();
            Phase2_ScheduledMutations();
            Phase3_ComponentCallbacks();
            Phase4_RecomputePower();
            Phase5_AggregateForce();
            Phase6_ApplyImpulses();
            Phase7_AdvancePhysics();
            Phase8_SynchronizeState();
            Phase9_SampleAndEvaluate();

            Clock.Advance();
        }

        /// <summary>Runs <paramref name="ticks"/> fixed intervals.</summary>
        public void Step(int ticks)
        {
            if (ticks < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(ticks));
            }
            for (int i = 0; i < ticks; i++)
            {
                Step();
            }
        }

        /// <summary>
        /// Runs for <paramref name="seconds"/> of simulation time. The count of
        /// steps is derived from the fixed 1/60 s step; partial steps are
        /// rounded up so the full duration is covered.
        /// </summary>
        public void RunFor(double seconds)
        {
            if (seconds < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(seconds));
            }
            int steps = (int)Math.Ceiling(seconds / TickClock.FixedStep);
            Step(steps);
        }

        /// <summary>
        /// Steps until <paramref name="condition"/> is satisfied by a
        /// completed-state observation, or <paramref name="timeoutSeconds"/>
        /// elapses. Returns true if satisfied, false on timeout.
        /// </summary>
        public bool RunUntil(Func<bool> condition, double timeoutSeconds)
        {
            int maxSteps = (int)Math.Ceiling(timeoutSeconds / TickClock.FixedStep);
            for (int i = 0; i < maxSteps; i++)
            {
                Step();
                if (condition())
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Schedules a scenario mutation to run on <paramref name="tick"/> in
        /// phase 2. Mutations due on the same tick run in registration order.
        /// </summary>
        public void AtTick(long tick, Action action)
        {
            if (tick < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(tick));
            }
            List<Action> list;
            if (!_scheduled.TryGetValue(tick, out list))
            {
                list = new List<Action>();
                _scheduled.Add(tick, list);
            }
            list.Add(action);
        }

        /// <summary>
        /// Registers a callback invoked on every Update1 in phase 3 with
        /// start-of-tick time. <see cref="SimUpdateContext.TimeSinceLastRun"/>
        /// reflects the accumulated interval since the callback last ran, so a
        /// suppressed (missed) Update1 still advances simulation time for the
        /// next invocation without skipping physics.
        /// </summary>
        public void EveryUpdate1(Action<SimUpdateContext> action)
        {
            _updateCallbacks.Add(new UpdateCallback(action));
        }

        /// <summary>
        /// Suppresses the next Update1 (phase 3) without advancing fewer
        /// physics steps — models a missed programmable-block update ().
        /// The suppression is one-shot; the following tick's callbacks see the
        /// accumulated interval.
        /// </summary>
        public void SuppressNextUpdate1()
        {
            _suppressNextUpdate1 = true;
        }

        private void Phase1_PublishCompletedState()
        {
            _phaseTrace.Add(PhaseNames[0]);
            // The state completed by the previous physics interval is published
            // here. Body state is readable directly (SimRigidBody.GetState);
            // observers and recorders sample it via the same surface.
        }

        private void Phase2_ScheduledMutations()
        {
            _phaseTrace.Add(PhaseNames[1]);
            List<Action> list;
            if (_scheduled.TryGetValue(Clock.Tick, out list))
            {
                // Iterate by index so a mutation may schedule further work for
                // the same tick; it runs after the already-registered
                // actions, preserving registration order. The list is shared
                // with the caller's registration, so it stays alive past this
                // tick only while the tick's own list is still referenced.
                for (int i = 0; i < list.Count; i++)
                {
                    list[i]();
                }
                _scheduled.Remove(Clock.Tick);
            }
        }

        private void Phase3_ComponentCallbacks()
        {
            _phaseTrace.Add(PhaseNames[2]);
            bool invoke = Clock.IsUpdateDue(UpdateFrequency.Update1, Clock.Tick) && !_suppressNextUpdate1;
            _suppressNextUpdate1 = false;
            if (!invoke)
            {
                return;
            }
            foreach (UpdateCallback callback in _updateCallbacks)
            {
                UpdateInvocation invocation;
                try
                {
                    invocation = callback.Invoke(Clock);
                }
                catch (Exception ex)
                {
                    // An uncaught component exception fills the short failure
                    // ring buffer () so the fixture can surface the
                    // final state and recent history. Rethrow so the test
                    // still fails at the point the component threw.
                    FailureLog.Add("tick " + Clock.Tick + " uncaught component exception "
                        + ex.GetType().Name + ": " + ex.Message);
                    throw;
                }
                UpdateCadence.Add(invocation);
            }
        }

        private void Phase4_RecomputePower()
        {
            _phaseTrace.Add(PhaseNames[3]);
            // Recompute power availability and block IsWorking state before
            // actuation.  adds the deterministic per-grid power
            // bus: phase 4 first recomputes every functional block's
            // delivered power ratio (priority-ordered demand against the
            // ideal or finite battery supply), which updates IsPowered,
            // then recomputes IsWorking from enablement, functionality,
            // and power (, ). Disposed grids/blocks (NRE
            // guard) are skipped.
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.PowerBus.Recompute((float)TickClock.FixedStep);
                }
            }
            for (int i = 0; i < _functionalBlocks.Count; i++)
            {
                SimFunctionalBlock block = _functionalBlocks[i];
                if (!block.IsDisposed)
                {
                    block.RecomputeIsWorking();
                }
            }
        }

        private void Phase5_AggregateForce()
        {
            _phaseTrace.Add(PhaseNames[4]);
            // Aggregate thruster force (and later, gyro torque) from values
            // written by callbacks. : each grid's ForceAggregator
            // gates its thrusters (enabled/functional/powered/
            // environmentally effective), converts override or group
            // demand into delivered thrust scaled by the bus's phase-4
            // DeliveredRatio, and sums it by grid-body direction. Disposed
            // grids (NRE guard) are skipped; their natives are gone.
            // .5: run the grid's controller-driven base thrust
            // demand (occupied controllers' MoveIndicator + inertia
            // dampening + anti-gravity, the game's ComputeBaseThrust) BEFORE
            // the force aggregation, so the aggregator consumes it as its
            // group demand (effectiveness + power + direction all apply).
            // The base path is idle when no controller is under control.
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.BaseControl.ComputeBaseDemand();
                }
            }
            // .6: the controller's rotation control writes the
            // gyro ControlTorque (last-under-control-wins), then the
            // remote autopilot — which flies through the SAME gyro
            // channel — runs after so it wins when enabled (the game's
            // per-frame last-writer assignment).
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.RotationControl.ComputeControlTorque();
                }
            }
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.Autopilot.ComputeAutopilot();
                }
            }
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.ForceAggregator.Aggregate();
                }
            }
            // : each grid's gyro-rate system aggregates the
            // force-weighted override target rate and the non-override
            // authority pool ( step 2, ).
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.GyroRateSystem.Aggregate();
                }
            }
            // : evaluate connector docking capture () over
            // the deterministic connector registry, so a connector's
            // Connectable state is established before actuation each tick
            // (Individual grids need not be non-disposed here — the
            // registry only holds connectors; a closed connector is
            // skipped by the gate).
            ConnectorSystem.EvaluateCapture(_connectors, Profile);
        }

        private void Phase6_ApplyImpulses()
        {
            _phaseTrace.Add(PhaseNames[5]);
            // Apply natural gravity (and, later, thruster/gyro impulses) to
            // each dynamic rigid body. The same gravity vector is what a
            // controller's GetNaturalGravity would return ().
            // Disposed bodies (NRE guard) are skipped; their natives are
            // gone.
            float dt = (float)TickClock.FixedStep;
            for (int i = 0; i < _bodies.Count; i++)
            {
                SimRigidBody body = _bodies[i];
                if (body.IsDisposed)
                {
                    continue;
                }
                if (!body.IsDynamic)
                {
                    continue;
                }
                Vector3 gravity = Gravity.GetNaturalGravity(body.PositionWorld);
                if (gravity != Vector3.Zero)
                {
                    body.ApplyLinearImpulse(gravity * body.Mass * dt);
                }
            }
            // : apply each grid's ONE resultant body-space thrust
            // impulse at the grid center of mass (the verified game
            // application point, MyThrusterBlockThrustComponent.cs:201 —
            // so fixed thrusters generate no torque from block position).
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.ForceAggregator.ApplyThrust(dt);
                }
            }
            // : apply each grid's ONE resultant body-space angular
            // impulse from the gyro override loop ( step 8).
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.GyroRateSystem.ApplyTorque(dt);
                }
            }
            // .5: apply the connector approach convergence (the
            // magnetic pull at the connector points + axis alignment) for
            // any approaching pre-lock pair.
            ConnectorSystem.ApplyApproach(_connectors);
        }

        private void Phase7_AdvancePhysics()
        {
            _phaseTrace.Add(PhaseNames[6]);
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                _hkWorld.StepSimulation((float)TickClock.FixedStep, false);
            }
        }

        private void Phase8_SynchronizeState()
        {
            _phaseTrace.Add(PhaseNames[7]);
            // Synchronize grid/block world matrices, velocities, connector
            // state, and actuator telemetry. This stage synchronizes each
            // rigid body's pose and velocities back from Havok; blocks and
            // connectors arrive in later stages. Disposed bodies (NRE
            // guard) are skipped — their natives are gone.
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                for (int i = 0; i < _bodies.Count; i++)
                {
                    if (!_bodies[i].IsDisposed)
                    {
                        _bodies[i].SynchronizeFromHavok();
                    }
                }
                // .6: clamp each grid's post-physics velocity to its
                // enabled autopilot's SpeedLimit (MovementBlockMaxSpeed), so
                // the cap holds on the observed post-step velocity.
                for (int i = 0; i < _grids.Count; i++)
                {
                    SimCubeGrid grid = _grids[i];
                    if (!grid.IsDisposed)
                    {
                        grid.Autopilot.ApplySpeedLimitAfterPhysics();
                    }
                }
            }
            // ,  step 6: update each thruster's
            // CurrentThrust after the physics interval (the delivered
            // force the phase 5 aggregation computed and phase 6 applied),
            // so the value is readable from the next tick's callbacks.
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.ForceAggregator.UpdateThrusterTelemetry();
                }
            }
            // , : publish each gyro's delivered torque and
            // saturation after the physics interval.
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    grid.GyroRateSystem.UpdateGyroTelemetry();
                }
            }
            // .5: advance the connector approach — an approaching
            // pair that has converged is locked (final fixed constraint,
            // both Connected) once the connectors physically meet.
            ConnectorSystem.AdvanceApproach(_connectors, Profile);
        }

        private void Phase9_SampleAndEvaluate()
        {
            _phaseTrace.Add(PhaseNames[8]);
            // Keep the short failure ring buffer topped with recent actuator
            // history () and sample the opt-in recorders. Neither
            // exposes future state: both run after the post-step sync.
            for (int i = 0; i < _grids.Count; i++)
            {
                SimCubeGrid grid = _grids[i];
                if (!grid.IsDisposed)
                {
                    FailureLog.Add(ActuatorSnapshot.Capture(grid, Clock.Tick, Clock.Time).ToLine());
                }
            }
            for (int i = 0; i < _samplers.Count; i++)
            {
                _samplers[i].Sample();
            }
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            // Deterministic teardown in reverse spawn order: disconnect any
            // live connector (removing its Havok constraint while the world
            // and the bodies are still alive), remove each body from the
            // world, then release the body and its shape, close each grid
            // (and its blocks), then dispose the HkWorld.
            for (int i = 0; i < _connectors.Count; i++)
            {
                SimShipConnector connector = _connectors[i];
                if (!connector.Closed && (connector.PartnerInternal != null || connector.ApproachingInternal))
                {
                    connector.Disconnect();
                }
            }
            for (int i = _bodies.Count - 1; i >= 0; i--)
            {
                SimRigidBody body = _bodies[i];
                body.RemoveFromWorld(_hkWorld);
                body.Dispose();
            }
            for (int i = _grids.Count - 1; i >= 0; i--)
            {
                _grids[i].Dispose();
            }
            _bodies.Clear();
            _grids.Clear();
            _functionalBlocks.Clear();
            _connectors.Clear();
            _samplers.Clear();
            _scheduled.Clear();
            _updateCallbacks.Clear();
            _hkWorld.Dispose();
            _disposed = true;
        }

        /// <summary>
        /// Tracks a per-Update1 callback and the interval since it last ran.
        /// <see cref="TimeSinceLastRun"/> accumulates over suppressed
        /// (missed) Update1s so the consumer always sees real elapsed
        /// simulation time, and never future state.
        /// </summary>
        private sealed class UpdateCallback
        {
            private readonly Action<SimUpdateContext> _action;
            private long _lastRunTick;
            private bool _hasRun;

            public UpdateCallback(Action<SimUpdateContext> action)
            {
                _action = action;
            }

            public UpdateInvocation Invoke(TickClock clock)
            {
                long elapsedTicks = _hasRun ? (clock.Tick - _lastRunTick) : 0;
                double sinceLastRun = clock.TimeSinceLastRun(elapsedTicks);
                _lastRunTick = clock.Tick;
                _hasRun = true;
                _action(new SimUpdateContext(clock.Time, sinceLastRun, UpdateType.Update1));
                return new UpdateInvocation
                {
                    Tick = clock.Tick,
                    Time = clock.Time,
                    SinceLastRun = sinceLastRun,
                    AppliedInterval = TickClock.FixedStep,
                };
            }
        }
    }
}
