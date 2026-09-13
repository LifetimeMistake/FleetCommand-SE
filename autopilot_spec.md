> Subsequent implementation note: `SPEC_RECHECK.md` adds the passive inertia observer, latched timing and Keen override model in an opt-in benchmark path. Earlier unimplemented-observer statements below describe the original v0.9 run. The recheck finds that the ideal timing window can underestimate actual settling.

---
title: Autonomous Craft Autopilot
subtitle: Control & Guidance Specification — experimental revision
version: 0.9
date: 2026-09-06
platform: Space Engineers — intended PB/sandboxed C# port
status: Python reference iteration; not flight-qualified
---

## 0. Purpose, scope, and relationship to v0.8

The fleet needs one predictable autopilot core: a missile, a six-faced spacecraft,
and a docking drone should not each carry unrelated assumptions about physics.
This revision preserves v0.8's structure and its central decision: guidance emits
desired acceleration and an attitude mask; allocation resolves what the craft can
produce; gyros receive rate commands and thrusters receive force fractions.

Consistency means shared contracts, capability estimation, actuation, status and
failure semantics. It does not mean every actuator geometry can execute every
maneuver. A forward-only vehicle can intercept and reverse direction by turning;
it cannot independently hold arbitrary attitude and arbitrary acceleration.

Fleet coordination, target assignment, communications reliability, obstacle mapping,
resource harvesting and weapons remain above or adjacent to this layer. This
iteration extends individual-vessel guidance to moving reference frames so those
higher layers can request formation offsets and docking approaches consistently.

The supplied v0.8 files are preserved in `reference/`. This document supersedes
their mathematical claims where explicitly corrected below. `RESULTS.md` and
`docs/implementation_report.md` distinguish runnable evidence from design contracts.
Public API and engine-source qualifications are in `docs/se_integration.md`.

### Changes with a reason, not a replacement architecture

| v0.8 component | v0.9 treatment | Reason |
|---|---|---|
| Acceleration seam | Preserve | Fits all guidance behaviors without coupling actuator code to a mission |
| Separate attitude objective | Preserve and represent FREE/POINT/FULL explicitly | A direction alone cannot express docking roll |
| Asymmetric thrust box | Preserve for a rigid supplied grid | Encodes underactuation without craft-type branches |
| Current-attitude projection | Preserve; correct direction-preservation claims | It remains useful during slews, but can change force direction |
| Feasibility -> mask -> optimizer | Preserve for soft masks; add explicit hard terminal constraint | Terminal pose cannot be silently abandoned while claiming docking success |
| Capability centroid | Retain as possible heuristic, not optimizer proof | Symmetric thrust boxes can still be strongly anisotropic |
| Demand-rate feedforward | Preserve, correct damping to rate error | Absolute-rate damping creates tracking bias |
| Passive inertia observation | Preserve as optional response diagnostic | Torque saturation and current engine response are not known from command alone |
| Continuous guidance laws | Preserve, repair braking/reference derivatives | Acceleration interface alone does not ensure consistent trajectories |
| Late arrival bind | Preserve as scheduling heuristic plus measured terminal gate | Ideal slew time is not a completion guarantee |

## 1. Why a custom controller is still justified

The useful departure from an RC waypoint list is not merely smoother steering.
The caller can request simultaneous motion and pointing: match a moving vessel,
hold a weapon on a target while orbiting it, or bring two moving connector faces
together with small relative velocity. Different mission states can release or
bind attitude freedom without changing the lower-level control implementation.

Point-to-point flight remains a guidance behavior. A waypoint is input to that
behavior, not the universal interface between navigation and the actuators.

## 2. Architecture and contracts

Each control step uses a coherent state and capability snapshot:

1. **Navigation:** self state, timestamped contacts, reference-frame estimates.
2. **Guidance:** net world acceleration, attitude request, mission status and a
   time estimate. Guidance alone interprets the mission.
3. **Avoidance seam:** modifies guidance subject to reachable acceleration and
   attitude limits. A concrete obstacle solver is not implemented in this iteration.
4. **Allocation:** selects admissible attitude and projects force at current attitude.
5. **Control/adapter:** gyro rate and thruster fractions, with measured outcome feedback.
6. **Passive diagnostic:** observed response/availability, age and confidence.

The output is not only an actuator command. Allocation returns the requested force,
achieved modeled force, acceleration residual, saturation and mask-relaxation state.
Guidance must not interpret `command accepted` as `motion achieved`.

These feedback fields preserve v0.8's layering: the allocator still knows no target,
obstacle, weapon or docking port. They let the layers above reason about a command
that cannot be executed. Capability inquiries are data, not a second guidance loop.

Suggested mission statuses are RUNNING, SETTLED, CAPTURE_READY, DEGRADED, REJECTED
and STALE. The Python implementation's concrete enums/fields are indexed in its
implementation report; this list describes their intended meaning.

## 3. Frames, state and reference points

Use a right-handed body frame B and inertial world frame W. R maps body column
vectors into world coordinates: x_W=R x_B. Use transpose(R) in the reverse direction.
Self state is COM position p, COM velocity v, orientation R and angular velocity
omega_B. Gravity g is a WORLD acceleration. Linear force is in newtons; mass in kg;
position in m; angular rates in rad/s; elapsed time in simulation seconds.

For a body-fixed point b measured from COM:

    x = p + R b
    xdot = v + omega_W × (R b)
    xddot = a_COM + alpha_W × (R b) + omega_W × (omega_W × R b)

This point transform is essential for docking. It is also why cockpit position,
grid position, a camera impact point and COM cannot be used interchangeably.

API angular velocity is converted once into the chosen body frame. A gyro's own
local frame is a separate final transform. The Python basis is mathematical; the
SE adapter must explicitly translate VRage matrix/basis conventions and signs.

## 4. Plant and limits

For the ideal rigid-body translational model:

    pdot = v
    vdot = R f / m + g
    Rdot = R [omega_B]cross

For a general rigid body, rotational dynamics are:

    J omegadot + omega × (J omega) = torque

The reference test bench may use a diagonal effective rate plant and an Euler-
coupled variation. Neither defines the game's physics. Gyro override is treated as
a rate request with finite response and authority, not as direct angular velocity
assignment. Unknown actuator dynamics are a reason for response tests, not for
assuming an arbitrary exact time constant.

The force used over an integration interval is transformed by the state at the
start of that interval. State, target, metric and telemetry timestamps must agree.
The speed limiter is a configurable test-bench approximation. The guidance layer
must respect the server's usable speed envelope rather than exploiting velocity
renormalization as free lateral braking.

The rigid-grid scope excludes articulated thrust, strong mechanically coupled
subgrids, gravity drives, aerodynamic mods, connectors while physically attached,
and external contact forces unless explicitly modeled.

## 5. Separate objectives with explicit feasibility

Keep translation and attitude distinct. A six-faced craft can often track both;
its finite limits still matter. An underactuated craft couples translation to
attitude, as v0.8 correctly recognized.

A mission declares whether a pointing constraint is **soft** (yield to motion)
or **hard** (cannot be violated while claiming the mission is admissible). FREE
has no orientation requirement. POINT aligns one body vector and leaves roll
free. FULL specifies a complete orientation, including roll.

A full hard pose does not make impossible translation possible. The allocator
holds the requested pose target, reports the residual and lets guidance reject,
hold or replan. Saturation during a normal slew is not itself a mission rejection;
mission admission considers the required pose and a sustained control margin.

## 6. Thrust capability

For fixed body-aligned thrusters with adequate supply:

    Fbox = [-Tx-, Tx+] × [-Ty-, Ty+] × [-Tz-, Tz+]

Cache references/geometry separately from effective available thrust. T values
depend on working blocks and environment; mass can change with cargo. Six numbers
describe an instantaneous ideal force set, not resources, slew ability or geometry.

Feasibility uses the original asymmetric gauge:

    gamma(f) = max_i (f_i/Ti+ if f_i>=0 else -f_i/Ti-)

with positive/zero -> infinity and zero/zero -> zero. A zero or missing thrust axis
is valid, not an exceptional craft class. Feasible means gamma <= 1 within numerical
tolerance, not merely that the support axes span three dimensions.

Projection remains:

    project(f)_i = clip(f_i, -Ti-, Ti+)

It minimizes Euclidean force error. In general it changes force direction. It
reduces to scalar scaling on a single thrust axis and on special rays whose active
components saturate proportionally; it is not equivalent to radial scaling for
all six-faced craft or all centroid-aligned demands.

Directional capacity for a unit body direction u is:

    radial_limit(u) = min_{i: |u_i|>epsilon} matching_limit(u_i) / |u_i|

The zero vector has no direction and is handled separately. An inaccessible
nonzero component produces zero radial capacity. Do not derive acceleration
authority using min(Tp): it is zero on the intended forward-only craft.

With fully free orientation, maximum static resultant force is the norm of a
farthest box vertex q, where each component picks the farther positive/negative
endpoint. Rotating q onto the demand reaches that capacity. This is an upper
capability, not the braking force available before a turn has finished.

## 7. Net acceleration and availability feedback

Guidance commands NET acceleration, including the desired gravitational outcome:

    Fstar_W = m * (a_cmd_W - g_W)
    fstar_B = R.T * Fstar_W
    f_ach_B = project(fstar_B)
    a_ach_W = R * f_ach_B / m + g_W
    residual_W = a_cmd_W - a_ach_W

Hover is a_cmd=0 and Fstar=-m*g. Ballistic freefall is a_cmd=g and Fstar=0.
An upstream acceleration clamp must not remove the hover requirement. Shared
resource limits can make the box optimistic; compare modeled and measured motion
before certifying a mission margin.

At a fixed pose, the acceleration set is g + R Fbox/m. Mission admission evaluates
the required reference acceleration plus a correction reserve within that set.
For hard precision docking, sufficient operational conditions include directional
margin for errors on both signs of each necessary translation axis and sufficient
angular tracking authority. They are conservative admission conditions, not a
proof that every rejected craft is physically incapable of every possible docking.

## 8. Attitude allocation: retain the mask and priorities

### 8.1 Why optimize, and what to optimize

v0.8's reason is preserved: free orientation can point stronger thrusters into
the demand. But spending seconds slewing to maximize force for a tiny correction
is not useful. There are two different questions:

- Can the current orientation satisfy this demand with useful margin?
- If not, which nearby admissible orientation substantially improves capability?

The original max-radial-thrust objective remains a capability metric. Operational
selection first seeks demand feasibility, then continuity/low rotation cost, with
extra force margin as a tie-breaker. Use deadbands/hysteresis to avoid flipping
between equivalent box corners as a small force direction changes.

### 8.2 Priorities

For **soft** masks: first seek force accessibility, then satisfy as much of the
mask as possible, then optimize remaining freedom. If the mask is relaxed, report
it explicitly. This retains the missile/gunship rationale of v0.8.

For **hard** masks: stay on the mask manifold and expose any infeasibility. Mission
guidance changes the maneuver, rather than the allocator silently changing what
`docking pose` means. FULL has no remaining freedom; POINT has roll; FREE has three.

The count 3-r for thrust-support rank r identifies generic missing-subspace equality
constraints only. One-sided signs and finite magnitude still restrict feasibility.
A rank-three no-retro craft cannot thrust in every direction at its current pose.
A pointed forward-only missile still has roll freedom, even if roll yields no
thrust improvement.

### 8.3 Practical search

The reference iteration uses bounded candidate selection rather than a general
nonlinear optimizer. Useful candidates include current/previous attitude, alignments
of nonzero box vertices or supported directions with demand, and roll candidates
within a pointing constraint. All candidates are evaluated with the same box.

Full-free corner alignment supplies maximum static magnitude, but does not prove
the chosen candidate minimizes maneuver time. A finite roll search approximates
the best roll and can miss a narrow feasible interval; failure to find a feasible
candidate is not a mathematical certificate. Current force projection always runs
independently of this search so the craft continues to use available thrust.

### 8.4 Projection and avoidance

Projection is parameter-free and remains the default saturation policy. Additional
gating is a behavior experiment, not a required craft-specific branch. Nonzero
projection cannot have a negative dot product with the requested force for a box
containing the origin, but this is not a collision-avoidance guarantee.

Avoidance must reason over achievable motion including turn delay, or revalidate
the allocated trajectory. A modified acceleration can lose its safety property
when force components are clipped. Neither potential fields nor simple priority
arbitration universally eliminate deadlocks.

## 9. Control and passive response observation

### 9.1 Attitude error and rate command

Use a shortest rotation-vector error, not Euler-angle subtraction. For POINT,
compute the minimal alignment rotation using atan2(norm(cross),dot); at pi choose
a deterministic perpendicular axis. For FULL, compute the SO(3) relative-rotation
logarithm with robust near-zero and near-pi handling.

In body coordinates:

    omega_cmd = omega_ref + Kp*e - Kd*(omega - omega_ref)

Absolute-rate damping (-Kd*omega) would produce constant lag when omega_ref is
nonzero. Rate-error damping preserves feedforward equilibrium. The engine's inner
rate controller already supplies damping; Kd=0 is a valid outer-loop baseline.

For a moving unit direction d, minimal feedforward is d × (d/dt d). For full attitude, use the reference angular
velocity. Derive feedforward from the actual selected reference, not the target
line of sight unless those are the same direction. Reset/filter derivative state
at mode switches and near vanishing force; do not differentiate a discontinuous
candidate switch as though it were a physical reference rate.

Rate limits and turn-response limits are controller configuration tied to observed
craft performance. Zero force does not disable FULL/POINT attitude control.

### 9.2 Translation actuation

At current attitude project required body force, split signs into opposing groups,
and command a common normalized fraction to each working thruster in a group.
Zero the opposite group. No integrator is required in this ideal force allocator.
External disturbances and model mismatch can justify an outer disturbance observer,
but are not hidden inside the six-limit geometry model.

### 9.3 Keep passive observation, qualify its use

The useful v0.8 insight survives: response identification can ride along with normal
slews, and the attitude loop need not depend on an identified inertia. Observe
timestamped rate changes, commanded rate error, configuration generation and age.
Publish effective angular-response samples with confidence and excitation direction.

Only under known dominant-axis torque saturation does tau/abs(alpha) identify an
axis inertia. A spherical shared torque budget, unknown power, changing inner-loop
ramp, or cross-axis coupling invalidates that simple readout. A plateau detector
can suggest saturation; it does not measure torque. Do not seed a supposedly safe
flight guarantee with an approximate unknown torque.

Keep a conservative configured response envelope until calibrated. Invalidate
observations after configuration changes. Passive samples may inform scheduling;
measured terminal pose/rate gates determine arrival. The reference implementation
report states which diagnostic pieces are implemented versus specified for a port.

## 10. Navigation and moving references

Detections provide velocity as well as position. Preserve timestamps and source
identity, estimate acceleration from past measurements, and predict to the common
control time. Never query future target truth in guidance. Cooperative frame data
should include orientation and angular velocity; acceleration and angular acceleration
may be estimated with confidence limits. Stale data expands uncertainty and eventually
blocks terminal admission, even if extrapolation still produces finite numbers.

For a moving frame with origin C, orientation Q, world angular velocity w and
angular acceleration alpha, a local point r(t) has:

    p_d = C + Q r
    v_d = v_C + w × (Q r) + Q rdot
    a_d = a_C + alpha × (Q r) + w × (w × Q r)
          + 2 w × (Q rdot) + Q rddot

The terms are respectively frame translation, angular acceleration, centripetal,
Coriolis and local relative acceleration. Omitting w cross offset makes a follower
chase a moving formation slot. Omitting the second derivative makes the velocity
loop supply a steady error while turning.

For a fixed slot rdot=rddot=0. For an orbit or docking approach, r varies smoothly.
This is one reference transform shared by several guidance strategies; it does not
change the acceleration contract or make allocation aware of another vessel.

## 11. Guidance strategies

### 11.1 Velocity/flight-controller behavior

For an already admissible velocity reference use:

    a_cmd = a_ref + Kv*(v_ref-v)

The caller supplies a consistent derivative a_ref, or a bounded reference generator
provides it. Attitude may be free, point-constrained or full. Saturation/residual
feedback prevents the caller from mistaking unreachable velocity for achieved flight.

### 11.2 Point-to-point transit and rendezvous

Retain braking-aware velocity planning, but do not treat sqrt(2*a*d) followed by
finite-gain velocity tracking as a no-overshoot guarantee. On the stopping envelope,
braking must already be active. Include a stopping supervisor or consistent
reference acceleration, and use the actual directional braking authority.

Near a feasible reference trajectory, a local tracker is:

    a_cmd = a_ref + Kp_pos*(p_ref-p) + Kv*(v_ref-v)

This is the feedback correction around the trajectory, not a substitute for
planning a stopping trajectory under saturation. Cruise speed, braking margin and
terminal tolerances are mission parameters. A rendezvous adds target reference
velocity and acceleration rather than trying to stop in world coordinates.

For a vehicle without suitable retro-thrust, reserve time/distance to turn before
using main thrust to brake. An idealized rest-to-rest slew estimate can initialize
this reserve; its conservatism must be checked against observed turn time. A
terminal pose is bound only when the final translation remains admissible. If an
underactuated pose/translation combination is infeasible, report it instead of
issuing conflicting targets indefinitely.

### 11.3 Orbit

Construct a continuous local circle in orthonormal plane basis u,v:

    r = radius*(cos(theta)*u + sin(theta)*v)
    rdot = radius*theta_dot*(-sin(theta)*u + cos(theta)*v)
    rddot = -theta_dot^2*r + radius*theta_ddot*(-sin(theta)*u + cos(theta)*v)

Transform through section 10, then track the resulting p/v/a. A ramp in theta_dot
must include theta_ddot. Acquisition may use a bounded transition before entering
the orbit rather than jumping to an unreachable phase. Choosing the initial phase
from projected position reduces needless cross-orbit flight.

This retains closed-form continuous geometry and centripetal feedforward. Unlike
the old spherical-radius field it defines the plane explicitly. A weapon-pointing
POINT mask leaves roll for the same allocator. If the available axes cannot support
the circle while holding aim, the soft-mask or hard-mask policy decides the outcome.

### 11.4 Intercept

Retain PN with axial closing and optional APN:

    r = p_target-p; d = r/|r|; vrel=v_target-v
    Vc = -dot(d,vrel)
    Omega = cross(r,vrel)/dot(r,r)
    a_cmd = a_close*d + N*Vc*cross(Omega,d) + (N/2)*a_target_perp

Use an explicit acquisition treatment for non-closing engagements and a guarded
terminal range. a_close is an operational parameter, not min(Tp)/m. APN uses a
bounded estimate derived from historical velocity samples. If the target is stale,
propagate only within an allowed horizon and expose degraded confidence.

Interception success is a swept proximity event, not a requirement to stop at the
target. The simulator can continue after first contact to study re-engagement, but
those passes are hypothetical for a spent munition. Missile guidance is validated
separately from rendezvous/docking because their terminal velocity requirements differ.

### 11.5 Formation

Formation slot selection belongs to Fleet Command. Guidance receives a local
offset in another vessel's published frame and uses section 10. The same local
reference tracker follows its p/v/a; requested relative attitude is a mask.
World speed margin includes both leader translation and omega cross slot offset.
Far-away slots around a fast-rotating leader can be infeasible even when its COM
speed is modest. Collision-free assignment and inter-drone avoidance are not supplied
by reference tracking alone.

### 11.6 Moving docking

The guidance reference is the receiving **mating frame**, not its COM or the first
camera hit. Let desired own attitude be Rd, own connector offset from COM be b,
and desired mating-point trajectory be x_d/v_x/a_x. Then:

    p_COM_d = x_d - Rd*b
    v_COM_d = v_x - omega_d × (Rd*b)
    a_COM_d = a_x - alpha_d × (Rd*b) - omega_d × (omega_d × Rd*b)

Construct Rd from the two connector local frames and a calibrated mating transform.
Opposed faces, roll policy and block-center spacing belong to this transform.
The mathematical reference uses abstract mating points; it does not simulate
connector magnets or block collision geometry.

Use staged guidance:

1. **Approach:** intercept a stand-off point outside the target hull/corridor.
2. **Align:** reduce relative position, velocity, attitude and angular-rate error.
3. **Final:** advance a smooth local axial reference with hard pose and correction
   reserve. Suspend advancement when freshness, pose, corridor or authority gates fail.
4. **Capture-ready:** all terminal errors remain within configured tolerances for a
   dwell time, with fresh reference and admitted authority.
5. **Connected:** an external SE adapter confirms actual attachment and hands over
   actuation. The simulator never claims this game state.

The runnable supervisor uses a C2 quintic axial reference, with configured local
speed/acceleration limits. Suspension smoothly brakes that reference before holding;
it cannot freeze a moving reference instantly while keeping consistent derivatives.
Admission combines translation reserves with configured gyro availability and angular
rate/acceleration limits. These are calibration inputs, not identified guarantees.

Critical gates are relative connector velocity and relative angular velocity, not
world speed. The receiving craft may translate and rotate continuously. Its motion
must stay inside the advertised/assumed envelope; no controller can guarantee capture
if the port makes an arbitrary unannounced maneuver beyond available thrust.

Without mapped hull clearance, the implemented point/corridor supervision is not
a collision-safe docking planner. Tests establish port tracking and capture gating.

## 12. Arrival timing and constraint release

Keep guidance's late-binding rule and its latch. An ideal one-axis rest-to-rest
slew estimate with angle theta, acceleration alpha and rate limit wmax is:

    t = 2*sqrt(theta/alpha), if theta <= wmax^2/alpha
    t = theta/wmax + wmax/alpha, otherwise

It is a minimum-time idealization. Add initial angular velocity, response delay,
settling and capability uncertainty when using it as a scheduling estimate. Range
over closing speed is also an estimate, not an unconditional lower bound on time.

Bind early enough to enter a measured alignment window. Once bound, latch until
completion or an explicit mission-state transition. Do not release on a noisy
time estimate. Arrival is established by position/velocity/attitude/rate tolerances
over dwell time, not elapsed predicted slew duration. The worst-case estimator
remains useful for efficiency; its error cannot be allowed to assert false success.

## 13. Boundaries and failure semantics

The shared core supports capability-dependent missions, not universal flight.
Reject malformed/nonfinite state, zero mass, invalid attitude and unusable timing.
No-thrust/gyro-loss cases must not silently produce a successful mission. Stale
contact data must block precision capture. A stopped script cannot guarantee it
will clear persistent overrides; production integration needs an explicit watchdog
and a recovery policy appropriate to the craft.

Collision avoidance, articulated-body dynamics, shared-resource optimization,
formation deconfliction, network identity/authentication and actual attachment are
outside the runnable reference unless the implementation report says otherwise.
These are not hidden fallback behaviors. Simulator success cannot promote them to
implemented or validated features.

## 14. Validation standard

Preserve the original four actuator geometries, add strongly anisotropic symmetric
craft, and test both successful motion and correctly rejected requests. Use fixed
seeds, explicit start state, reference motion, actuator parameters, time step,
horizon and tolerances. Metrics must not label all future hypothetical passes as
successful physical interceptions.

Record at least: swept closest approach for intercept; terminal/worst-window
position and relative velocity for tracking; full attitude/rate errors for docking;
capture dwell and rejection reason; saturation/residual; and actuator impulse
sum_i(abs(f_i))*dt. Net-resultant impulse is a different metric and may also be logged.

Run direct regressions for v0.8's zero acceleration limit, centroid anisotropy,
projection-direction claim, rotating-rate damping bias, zero-force attitude control,
and inconsistent moving-reference derivatives. Add scenario sensitivity to limited
update rate, non-isotropic inertia, slower gyro response, gravity and timestamped
target data. Any exploratory failures stay in the results.

`RESULTS.md` records the actual run, not predicted success. The complete original
v0.8 experimental appendix is not reproducible from its supplied entry point alone;
this package does not copy unverified historical figure claims as new evidence.

## Appendix A — deployment work remaining

Port tested math into sandboxed C# with double-precision vectors; use a bounded
candidate count; cache references and precomputed geometry; keep text parsing and
enumeration off the hot loop. Feed actual simulation dt and packet age. Verify
adapter signs, force fractions, gyro dynamics, resource delivery, COM/reference
point semantics and connector lifecycle against the target build, following
`docs/se_integration.md`.

The reference package is a concrete iteration on the existing design. It is not a
replacement for those game tests or a claim that a fleet can be deployed unchanged.

## Appendix B — bounded attitude profile implementation note

The next-test plan has now been implemented as an opt-in controller path. The
selected design uses a conservative shared-torque acceleration profile, an
explicit 60 Hz override-rate bridge, passive generation-scoped inertia
observation, constant-time slew estimates, and measured readiness latches.
Historical controller defaults remain unchanged for reproducibility.

The frozen run and limitations are recorded in `ATTITUDE_PROFILE_RESULTS.md` and
`results/attitude_profile/metrics.json`. In the source-derived diagonal fixture,
the profile-plus-bridge controller passed all 48 isolated cases and all 28
rest-start timing windows. This result does not widen the deployment claims in
Appendix A: the C# adapter and validation against the target game build remain
required.
