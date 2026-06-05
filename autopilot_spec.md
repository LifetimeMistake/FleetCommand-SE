---
title: "Autonomous Craft Autopilot"
subtitle: "Control & Guidance Specification"
author: "Platform: Space Engineers — in-game Programmable Block (sandboxed C#)"
date: "Version 0.7 · 3 June 2026"
---

## 0. Purpose and scope

This document specifies the control and guidance core of a custom autopilot for autonomous craft: how a single vessel moves precisely and predictably to a commanded state, including the ability to arrive at a position with a chosen orientation and to follow continuous paths such as orbits and intercepts.

The autopilot is one capability among several — resource harvesting, combat logic, drone orchestration — that a fleet-command protocol will eventually coordinate. That protocol and those other capabilities sit above this layer and are out of scope. What is in scope is everything from the moment a behavior decides it wants a particular acceleration down to the gyros and thrusters that produce it.

Each section that introduces a complication first explains the problem, meaning why the obvious approach fails, and then the approach, meaning what the chosen solution does. The mathematics follows the motivation. Section 14 is an experimental appendix recording how the inertia estimator was chosen and validated; it can be skimmed or skipped entirely, as nothing in the core mechanism depends on reading it.

---

## 1. Why the stock autopilot is inadequate

Space Engineers ships with a Remote Control block that provides a basic autopilot, but it is fundamentally a point-to-point position follower. You hand it a list of GPS coordinates and it flies to each in turn, slowing as it nears a point. For autonomous fleet craft this is too rigid, and the rigidity has several distinct roots.

The most consequential is that heading is welded to travel. The stock autopilot orients a chosen face of the ship toward the next waypoint, so there is no way to express "arrive at this point while facing that direction." For docking, weapon alignment, or holding a formation, the arrival orientation is the entire point, and it is exactly what the stock system cannot control independently of where the ship is going. Compounding this, a waypoint is nothing but a position; it carries no velocity or acceleration. The controller therefore has no notion of how fast it should be moving or which way it should be pointing when it gets somewhere — it can only try to stop at a point. Matching a moving target's velocity, passing through a point at speed, or intercepting a maneuvering craft are all simply outside what the representation can express. On top of these representational limits the control itself is crude, tending to overshoot and oscillate, wasting thrust, and making no attempt to reason about which orientation or which thrusters would reach the target most efficiently. Finally, any curved or complex path has to be approximated by a dense string of waypoints, which is jerky, expensive, and still cannot track a moving center.

The remedy is a layered controller that controls attitude as an independent objective, commands acceleration and velocity rather than bare positions, and reasons explicitly about the craft's real actuation capability. The rest of this document builds that controller.

---

## 2. System architecture and the central design choice

The root cause of every limitation above is the choice of interface. If the boundary between planning and execution is a position, or a stream of them, then every behavior must be expressed as a path, and the executing controller — handed only "go here next" — cannot reason about dynamics, velocity, orientation, or efficiency.

The design choice that unlocks everything else is to make the contract between layers a desired acceleration together with an attitude mask, never a waypoint. The mask, defined in section 8, is how guidance states which rotational freedoms it cares about and which it will let the controller use. Once that is the seam, every behavior collapses to a function that emits an acceleration vector and a mask, the executing layers reason about the full rigid-body dynamics, and following a list of waypoints becomes merely the simplest possible behavior rather than the foundation of the system.

The pipeline runs from sensing down to actuation each tick. Navigation estimates the craft's own state and tracks external contacts, producing both a clean self-state and a world model. Guidance, the currently active behavior, emits a desired world acceleration, an attitude mask, and an estimate of time remaining in the maneuver. Collision avoidance then warps that acceleration against the world model, which keeps every layer below it free of any knowledge of obstacles. Control allocation takes the safe acceleration and the mask and resolves attitude and per-axis thrust by a fixed priority. The control loops finally drive the gyros and thrusters to hit those setpoints, and a passive observer running alongside them learns the craft's rotational inertia from the motion it produces.

![The guidance-navigation-control pipeline. Avoidance and allocation highlighted.](img/d1.png){ width=62% }

The document follows this stack from the bottom up. Sections 6 through 8 cover control allocation; section 9, the control loops and the inertia observer; section 10, navigation; section 11, guidance and the masks it emits; and section 12, the timing rule that lets guidance defer an arrival constraint while the craft optimizes.

---

## 3. Notation, frames, and state

Work in a world frame $\mathcal{W}$ and a body frame $\mathcal{B}$, related by an attitude $R \in SO(3)$ whose columns are the body axes expressed in world coordinates, so that $\mathbf{x}_{\mathcal{W}} = R\,\mathbf{x}_{\mathcal{B}}$ and $\mathbf{x}_{\mathcal{B}} = R^{\top}\mathbf{x}_{\mathcal{W}}$. The craft's state is its position $\mathbf{p}\in\mathbb{R}^3$, world-frame velocity $\mathbf{v}=\dot{\mathbf{p}}$, and body-frame angular velocity $\boldsymbol{\omega}\in\mathbb{R}^3$. Its parameters are mass $m$, the diagonal effective rotational inertia $J=\operatorname{diag}(J_x,J_y,J_z)$ (per section 9.1 the engine's gyro model is diagonal, so three numbers suffice), and the local gravitational acceleration $\mathbf{g}\in\mathbb{R}^3$ in world coordinates. The operator $[\,\cdot\,]_\times$ denotes the skew-symmetric cross-product matrix.

Three conventions are fixed, and they were checked against the game source so that the implementation's frames and signs line up with what the API actually reports. First, $R$ is the rotation part of the controller's `WorldMatrix`, which is the body-to-world transform. Second, gravity from `GetNaturalGravity` is a world-frame acceleration vector pointing down, and it is folded into the force demand of section 5 — so to merely hover, $\mathbf F^\star=-m\mathbf g$, thrust opposing gravity, with the correct sign. Third, and most easily gotten wrong: the API reports angular velocity in the **world** frame, not the body frame. Every use of $\boldsymbol\omega$ in this document is body-frame, so the very first thing the controller does with the measured angular velocity is convert it,

$$
\boldsymbol\omega_{\mathcal B} = R^{\top}\,\boldsymbol\omega_{\mathcal W}.
$$

Skipping this conversion produces a controller that looks correct and rotates the wrong way.

---

## 4. Plant model

The rigid-body dynamics, with $\boldsymbol\omega$ in the body frame, are

$$
\dot{\mathbf{p}} = \mathbf{v}, \qquad
m\,\dot{\mathbf{v}} = R\,\mathbf{f} + m\,\mathbf{g},\qquad
\dot{R} = R\,[\boldsymbol{\omega}]_\times, \qquad
J\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times(J\boldsymbol{\omega}) = \boldsymbol{\tau},
$$

where $\mathbf{f}\in\mathbb{R}^3$ is the resultant body-frame thrust force and $\boldsymbol{\tau}$ the body torque from the gyros. The translational and rotational equations are driven by separate actuators — thrusters for $\mathbf f$, gyros for $\boldsymbol\tau$ — and it is exactly this independence that makes the next complication both possible and necessary.

---

## 5. Separating orientation from position

The stock autopilot welds heading to travel, so that where the ship goes dictates where it points. But translation and rotation come from independent actuators, and binding them together discards a genuine degree of freedom: most visibly, the freedom to arrive somewhere while facing a chosen direction.

The approach is to treat orientation and position as two control objectives advanced simultaneously by independent loops. Translation is driven toward a position or velocity target while attitude is driven toward its own target, so that a command such as "go to point $\mathbf P$ and be oriented like $R_{\mathrm{arr}}$ on arrival" becomes two targets converging together, with alignment achieved for free. This separation holds perfectly for a craft that can thrust in any direction. It breaks down for a craft that cannot — one that must point its nose in order to push — which reintroduces a coupling between the two loops. That coupling is not a contradiction but a dependency, resolved by the priority scheme of section 8. Reaching it first requires an honest model of the thrust a craft can actually produce, the subject of section 6.

---

## 6. Modeling real thrust capability

It is tempting to summarize a ship by a single maximum-thrust number, but that is a fiction. Thrust in Space Engineers is the sum of discrete thrusters bolted to grid faces, so capability is direction-dependent and asymmetric: a craft typically has a powerful main drive, weak retro-thrust, and modest lateral thrust, and an underactuated craft may have no thrust at all on some axes. A controller that reasons with a single scalar limit will command forces the craft physically cannot produce.

The honest model is the actual set of achievable body-frame forces. Because the thrusters align with body axes, that set is an axis-aligned, generally asymmetric, possibly degenerate box,

$$
\mathcal{F} = [-T_x^{-},\,T_x^{+}]\times[-T_y^{-},\,T_y^{+}]\times[-T_z^{-},\,T_z^{+}] \subset \mathbb{R}^3 ,
$$

in which $T_i^{\pm}\ge 0$ is the maximum thrust along $\pm\mathbf e_i$, so six cached numbers describe the craft completely. To test and scale demands against this box uniformly, use its gauge, or Minkowski functional, which captures asymmetry, one-sided thrust, and missing axes at once:

$$
\gamma_{\mathcal F}(\mathbf{f}) = \max_i \rho_i(f_i),
\qquad
\rho_i(f_i) =
\begin{cases}
f_i / T_i^{+}, & f_i \ge 0,\\[4pt]
-f_i / T_i^{-}, & f_i < 0,
\end{cases}
$$

with the conventions $c/0 = +\infty$ for $c>0$ and $0/0 = 0$. A force is feasible exactly when $\gamma_{\mathcal F}(\mathbf f)\le 1$. A missing thruster makes $\rho_i=+\infty$ for that sign, so infeasibility is reported automatically without any special case, and it is this property that lets one allocator drive a six-faced brick, a forward-and-vertical gunship, and a forward-only munition with the same code.

---

## 7. Force demand and feasibility

When guidance, after avoidance, emits a desired world acceleration $\mathbf{a}_{\mathrm{cmd}}$, setting $\dot{\mathbf{v}} = \mathbf{a}_{\mathrm{cmd}}$ in the plant gives the force the craft must produce,

$$
\mathbf{F}^{\star} := m(\mathbf{a}_{\mathrm{cmd}} - \mathbf{g}) \quad\text{(world)}, \qquad
\mathbf{f}^{\star}(R) := R^{\top}\mathbf{F}^{\star} \quad\text{(body)} ,
$$

where gravity cancellation is already folded into $\mathbf F^\star$. The demand is realizable at attitude $R$ exactly when $\gamma_{\mathcal F}(\mathbf f^\star(R)) \le 1$.

---

## 8. Allocating attitude: the mask, the optimizer, and priority

The previous section settles what force a craft must produce. This section settles the orientation it should hold to produce it. Two pressures make orientation something to be *decided* rather than merely tracked. An underactuated craft — a forward-only design — can push in only one direction, so to accelerate a given way it must first rotate to point that way; this is the coupling deferred from section 5. And even a fully capable craft wastes performance if it holds an arbitrary attitude and forces a large demand through its weak side thrusters while the powerful main drive sits idle. So the question this section answers is: of all the orientations the craft could hold, which one should the controller pick?

### The attitude mask: bound and free axes

Guidance does not always care about the craft's full orientation. A vessel cruising between waypoints may not care how it is rolled or which way its nose points, so long as it gets there; a gunship may care intensely about where its weapon points but not at all about rotation around the weapon's line. The mask captures this. Rather than handing the controller a single desired orientation, guidance declares, for each of the three rotational degrees of freedom, whether it is **bound** or **free**:

- a **bound** axis is one guidance demands a specific value for — "the nose must point here," "this face must stay up";
- a **free** axis is one guidance does not constrain — the controller may set it to whatever it likes.

The common cases follow directly. A craft holding a sensor or weapon on a target binds the two degrees of freedom that aim the relevant body axis and leaves the third — rotation about that aiming axis, i.e. roll about the line of sight — free. A craft that does not care how it is oriented binds nothing, leaving all three free. A craft that must arrive in a precise docking pose binds all three. The natural way to express a binding is as a pointing constraint — keep body axis $\mathbf b$ aligned with a world direction — which fixes the two degrees of freedom orthogonal to $\mathbf b$ and leaves rotation about $\mathbf b$ free; a strictly per-grid-axis mask is the special case where $\mathbf b$ is a grid axis. Guidance may also change the mask over time, deferring an arrival constraint until late in a maneuver; that is the subject of section 12.

### Why there is something to optimize

A free axis is latitude, and latitude is worth spending. Because the thrust capability is asymmetric (section 6), how much force the craft can produce toward the demand depends on its orientation: aligning the main drive with the demand can yield five to ten times the force available through a lateral thruster. So whenever the mask leaves an axis free, the controller uses that freedom to orient the craft so it can push as hard as possible along the demand. That search is the **thrust optimizer**, and it is the payoff for treating attitude as a resource rather than a fixed target. To state its objective precisely, define the most force the craft can produce along the unit demand direction $\hat{\mathbf d}=\mathbf F^\star/\lVert\mathbf F^\star\rVert$ at a given orientation $R$ — the reciprocal of the gauge:

$$
s_{\max}(R,\hat{\mathbf d}) = \frac{1}{\gamma_{\mathcal F}(R^\top \hat{\mathbf d})}
= \min_i \frac{\beta_i(R^\top\hat{\mathbf d})}{\big|(R^\top\hat{\mathbf d})_i\big|},
\qquad \beta_i(u)=\begin{cases}T_i^+,&u_i\ge 0\\ T_i^-,&u_i<0.\end{cases}
$$

The optimizer maximizes $s_{\max}$ over the free degrees of freedom $\theta$, holding the bound ones fixed:

$$
R^{\star} = \operatorname*{arg\,max}_{\theta\ \text{free}} \; s_{\max}\big(R(\theta),\,\hat{\mathbf d}\big).
$$

When no axes are free the optimizer has nothing to search and does not run; when all three are free it searches the full orientation.

### Resolving conflicts: a strict priority

Feasibility, the mask, and the optimizer can all pull on the same degree of freedom, so they are ordered by a strict priority rather than blended.

**Priority one is feasibility** — the craft must be able to produce the commanded acceleration at all. The number of rotational degrees of freedom this forces follows from the capability model: letting the support axes be those with any thrust, $\mathcal A=\{i:T_i^++T_i^->0\}$, spanning a subspace of dimension $r$, feasibility forces $n_{\mathrm{forced}} = 3 - r$ degrees of freedom.

| capability | $r$ | $n_{\mathrm{forced}}$ | what feasibility forces |
|---|---|---|---|
| all six faces | 3 | 0 | nothing; any direction producible at the current attitude |
| forward plus up and down | 2 | 1 | one rotation to bring the demand into the thrust plane |
| forward only | 1 | 2 | aim the nose along the demand |

**Priority two is the mask** — among orientations that satisfy feasibility, hold the bound axes. A bound axis beats the optimizer but yields to feasibility, and that ordering is what resolves the underactuated conflict. If a missile binds pitch and yaw to aim a sensor, but feasibility needs those same two degrees of freedom to point the nose along the demand, feasibility wins and the bind is dropped: the craft would rather move at all than hold an orientation it cannot move from.

**Priority three is the optimizer** — over whatever degrees of freedom remain free after the first two, maximize $s_{\max}$ as defined above.

### A worked example

Suppose an omnidirectional gunship is orbiting a target. Guidance asks for the orbit's acceleration $\mathbf a_{\mathrm{cmd}}$ (the centripetal-plus-tangential demand) while keeping the ship's forward-mounted weapon trained on the target. The weapon-pointing binds two axes — the pitch and yaw that aim the nose — and leaves roll about the nose free. Walk the priorities. Feasibility is vacuous, because an omnidirectional craft can thrust in any direction at any orientation, so it forces nothing. The mask then pins pitch and yaw to the aim. That leaves exactly one free degree of freedom: the roll angle $\phi$ about the line of sight. The optimizer sweeps $\phi$ for the orientation that maximizes $s_{\max}(R(\phi),\hat{\mathbf d})$. If the gunship's port-and-starboard thrusters are twice as strong as its dorsal-and-ventral ones, the optimizer rolls the hull so the strong lateral axis lines up with the in-plane part of $\mathbf a_{\mathrm{cmd}}$ — nearly doubling the achievable acceleration there — and it does so without moving the weapon a hair off target, because rolling about the aiming axis does not disturb the aim. One free axis, one real gain, no cost to the bound objective: that is the whole point of treating attitude as a masked, prioritized resource.

### Thrust saturation

A craft can be asked for more force than its thrusters can deliver — a heavy ship told to accelerate hard, or any craft pushed along a weak axis. This is **thrust saturation**, and it is distinct from the **gyro torque saturation** that appears in the inertia observer (section 9.1): thrust saturation is the translational actuators running out of force, gyro saturation is the rotational actuators running out of torque. They are unrelated mechanisms that happen to share the word.

When a force demand is unrealizable, something must be given up, and this autopilot gives up magnitude, never direction. It produces the most force it can along the requested vector rather than bending the heading to claw back magnitude, because for transit and intercept a force pointed the wrong way corrupts the trajectory worse than a force that is merely too small. Formally, with the scale factor $\sigma=\min\!\big(1,\,1/\gamma_{\mathcal F}(\mathbf f^\star(R))\big)$,

$$
\mathbf f_{\mathrm{ach}}(R)=\sigma\,\mathbf f^\star(R), \qquad
\mathbf a_{\mathrm{ach}}(R)=\tfrac1m R\,\mathbf f_{\mathrm{ach}}(R)+\mathbf g,
$$

which shrinks the force along $\hat{\mathbf d}$ while leaving its direction exactly on the requested vector. The heading the controller delivers is always the heading guidance asked for; only the magnitude is sacrificed when the craft is overpowered.

The figure summarizes the whole resolution: the demand and mask enter, the three priorities run in order, and an attitude setpoint and a thrust command leave.

![Control allocation resolves attitude by a fixed lexicographic priority.](img/d2.png){ width=66% }

---

## 9. The control loops and the inertia observer

The attitude loop works in terms of rotation error rather than Euler angles, which suffer gimbal degeneracies and wrap-around. Having converted the measured angular velocity into the body frame as $\boldsymbol\omega=R^\top\boldsymbol\omega_{\mathcal W}$ (section 3), it forms the error quaternion $\mathbf q_{\mathrm{err}}=\mathbf q_{\mathrm{sp}}\otimes\mathbf q_{\mathrm{cur}}^{-1}$, extracts the small-angle error vector $\mathbf e=2\,\operatorname{sign}(q_{\mathrm{err},w})\,\mathbf q_{\mathrm{err},xyz}$, and commands an angular velocity $\boldsymbol\omega_{\mathrm{cmd}}=-K_p(R^\top\mathbf e)-K_d\,\boldsymbol\omega$. That command is then mapped into each gyro's own local frame before its pitch, yaw, and roll are written. This transform and its per-axis sign conventions are the single most common point of failure in Space Engineers autopilots and must be unit-tested for each gyro's mounting.

The translation output realizes $\mathbf f_{\mathrm{ach}}$ at the craft's current attitude rather than the setpoint, splitting each body-frame component onto the appropriate one-sided thruster group through thrust overrides. Realizing the force at the current attitude means a craft in mid-slew still produces the best thrust available from where its nose currently points, rather than waiting idle until it finishes turning.

### 9.1 The inertia observer

The timing rule of section 12 needs an effective angular acceleration $\alpha_{\max}=\tau_{\max}/J$ per axis. Rather than treat the rotational inertia as a fixed parameter to be looked up — it cannot be, because a Programmable Block cannot read the grid's inertia tensor, cannot enumerate armor or other non-terminal blocks, and watches the mass shift in real time as cargo moves — the controller identifies it online from the motion it is already producing. The natural home for that is the control tier itself: a passive observer attached to the attitude loop, running every control tick. It consumes only what the loop already has on hand — the angular-velocity target it just commanded, the measured body-frame angular velocity (differenced into an angular acceleration), and the cached maximum gyro torque. It never commands a maneuver of its own; identification rides along on ordinary slews. Higher tiers read its published per-axis estimate; they do not compute it.

What makes this possible is the structure of the engine's gyro model, confirmed against the game source. The gyro override is a rate controller that sets the applied torque to the desired angular acceleration times the inertia, then clamps the result to the gyro torque ceiling, and it uses only the diagonal of the inverse inertia tensor — so the per-axis relation is $\tau_i = J_i\,\alpha_i$, decoupled, with no full tensor to reconstruct. Two regimes follow. While the demand stays within the torque ceiling, the inertia pre-multiply cancels and the craft reaches the commanded rate in a fixed number of frames independent of inertia: the inertia is invisible, but the slew time does not depend on it either, so nothing is lost. Once the torque saturates — gyro saturation, the rotational counterpart to the thrust saturation of section 8 — the relation collapses to $\alpha_i=\tau_{\max}/J_i$, and the inertia becomes directly observable in precisely the regime where slew time depends on it. The observer therefore reads its estimate straight off saturated ticks,

$$
\hat J_i = \frac{\tau_{\max}}{|\alpha_i|}\quad\text{(axis $i$, while saturated).}
$$

Equivalently it is reading the achieved saturated angular acceleration directly, which is the very quantity section 12 needs — so the exact value of $\tau_{\max}$ does not have to be accurate; it cancels between this readout and the slew estimate, serving only as the threshold that detects saturation, where an approximate value is fine.

The one genuine subtlety is which excitation reveals which axis. A slew dominated by a single axis (or one starting from rest, where the gyroscopic coupling term is negligible) yields that axis's inertia cleanly. A balanced simultaneous multi-axis slew does not: applying equal demand to pitch and yaw drives the torque in one fixed direction and produces identical angular acceleration on both axes regardless of their individual inertias, so only the combined magnitude $\sqrt{J_x^2+J_y^2}$ is observable, never the split. Separating the per-axis inertias therefore requires directionally-diverse excitation, which an autopilot's varied reorientations supply naturally over a few differently-directed slews. The observer accordingly updates an axis only when that axis dominates the angular acceleration, smoothing the readout and holding an axis otherwise rather than corrupting it with rank-deficient data. The estimator stays purely passive: it never perturbs a slew to refresh itself, because a stale axis becomes observable again the moment it is next slewed, and that is exactly when its value is needed.

Each axis is seeded at construction with a closed-form upper bound on its inertia from total mass and the bounding-box half-extents, $\hat J_i \le m(d_1^2+d_2^2)$ for the two largest half-extents, which needs nothing the API withholds and is never an under-estimate. This bound is both the cold-start value and the permanent fallback for any axis that has gone stale — after a structural change, or simply not recently slewed. The scheme is safe by construction because its only consumer is the section 12 timing rule: that rule needs an upper bound on slew time, an over-estimate merely triggers the arrival bind early (the safe direction), and the attitude loop's stability never depends on the value at all, since the engine closes the torque loop internally. A fixed-mass craft converges once and holds; a craft that loads cargo or takes damage drifts off and is re-identified by its next few diverse slews, with the bound covering the interim. Section 14 records the measurements: the bound runs roughly two-and-a-half to three-and-a-third times the true inertia for typical craft, while the online estimate reaches the true value within a tenth to a fifth of a second of an axis's first hard slew and then holds it to a few percent.

---

## 10. Navigation from imperfect sensing

The craft's own state is exact and essentially free from the API, but external contacts are neither. Detections from camera raycasts, sensors, or contacts shared over the intergrid link arrive noisily and intermittently and carry no velocity at all. Differencing raw positions to recover velocity amplifies that noise into something unusable, which matters because several guidance laws, intercept most of all, depend on a derivative of contact state.

The approach is to pass every tracked contact through a state estimator that produces smoothed position and velocity together: an alpha-beta filter for the common case, cheap enough to be unremarkable against the instruction budget, or a small Kalman filter where dropouts must be handled in a principled way. The world model is then a flat list of contacts, each a sphere with a center, radius, and velocity, and for the handful a single craft tracks a flat list outperforms any spatial index on cost. Collision avoidance consumes this model and warps the commanded acceleration, so that nothing below it ever needs to know an obstacle exists.

---

## 11. Guidance: continuous laws and the mask they emit

Consider flying a circle around a target by interpolating waypoints. Many points must be dropped around the ring, the controller stops and restarts at each, the path is a faceted polygon, denser points cost more, and the construction still cannot follow a moving center. The deeper trouble is that a waypoint is a bare position encoding no velocity or acceleration, so the controller never learns how fast or which way it should be moving as it passes through. Interpolating points is a poor reconstruction of information the geometry already contains exactly.

The approach is to express each maneuver as a continuous law that emits the desired acceleration directly, encoding the geometry in closed form, paired with the attitude mask of section 8. Every behavior is a function $\mathcal B$ mapping the craft's state, the target's state, the behavior's parameters, and time to a triple $(\mathbf a_{\mathrm{cmd}},\,\mathcal M_{\mathrm{att}},\,\tau_{\mathrm{go}})$ of commanded acceleration, attitude mask, and time-to-go. The mask is not fixed for a behavior; it may release and re-bind axes over time per section 12. Four laws form the core, each with its characteristic masking discipline.

The go-to law replaces a position controller, which overshoots, with a kinematic velocity profile that respects the braking limit so the craft arrives stopped. With distance $d=\lVert\mathbf P-\mathbf p\rVert$ along the unit heading $\hat{\mathbf d}$, the desired velocity is $\mathbf v_{\mathrm{des}}=\min(v_{\mathrm{cruise}},\sqrt{2a_{\mathrm{brake}}d})\,\hat{\mathbf d}$, optionally offset by a moving target's velocity, and the command is $\mathbf a_{\mathrm{cmd}}=\operatorname{clamp}(K_v(\mathbf v_{\mathrm{des}}-\mathbf v),\,a_{\max})$. This behavior exploits the release rule most fully: while $\tau_{\mathrm{go}}$ is large it leaves all axes free for thrust optimization across the cruise, and latches the arrival orientation only near the end.

The orbit law decomposes the desired velocity into a radial part that holds the radius and a tangential part that travels, with a centripetal feedforward so the controller does not fight its own curve. About a center $\mathbf C$ with plane normal $\mathbf n$, target radius $R_{\mathrm{orb}}$, and tangential speed $s$, writing $\boldsymbol\rho=\mathbf p-\mathbf C$ with magnitude $\rho$ and unit radial $\hat{\mathbf r}$ and unit tangent $\hat{\mathbf t}=(\mathbf n\times\hat{\mathbf r})/\lVert\mathbf n\times\hat{\mathbf r}\rVert$, the desired velocity is $\mathbf v_{\mathrm{des}}=-K_r(\rho-R_{\mathrm{orb}})\hat{\mathbf r}+s\,\hat{\mathbf t}$ and the command is $\mathbf a_{\mathrm{cmd}}=K_v(\mathbf v_{\mathrm{des}}-\mathbf v)-(s^2/R_{\mathrm{orb}})\hat{\mathbf r}$. An orbiting craft holding a weapon on the center binds the two pointing axes and leaves roll free — the one-free-axis optimization case worked through in section 8.

The intercept law is proportional navigation, commanding a lateral acceleration proportional to the rotation rate of the line of sight so as to produce a collision course without chasing the target's present position. With relative position $\mathbf r$ and relative velocity $\mathbf v_{\mathrm{rel}}$, the line-of-sight rate is $\boldsymbol\Omega=(\mathbf r\times\mathbf v_{\mathrm{rel}})/(\mathbf r\cdot\mathbf r)$ and the closing speed is $V_c=-(\mathbf r\cdot\mathbf v_{\mathrm{rel}})/\lVert\mathbf r\rVert$, giving $\mathbf a_{\mathrm{cmd}}=N\,V_c\,(\boldsymbol\Omega\times\hat{\mathbf r})$ with $N$ between three and five, augmented by $\tfrac{N}{2}\mathbf a_t$ against a maneuvering target, and $\tau_{\mathrm{go}}\approx\lVert\mathbf r\rVert/V_c$. For a forward-only munition the nose is already pinned by feasibility, so there are no free axes and the section 8 optimizer is inert; the binding constraint is whether the attitude loop can track a rapidly slewing demand. Because the law lives in $\boldsymbol\Omega$, a derivative of the filtered estimates of section 10, this is where navigation quality matters most.

Patrol sequences go-to, orbit, and loiter segments, chasing a carrot point a fixed lookahead along the path where smooth corners are wanted rather than stopping at each node. Sequencing is a finite-state machine in which each state binds a behavior and its parameters and transitions fire on guards such as arrival, target acquisition or loss, a timeout, or a fuel or health threshold.

![Example behaviour state machine for maneuver sequencing.](img/d3.png){ width=100% }

When several objectives are active at once it is better to arbitrate by priority than to blend. Summing the acceleration vectors of competing behaviors can create spurious equilibria, leaving a craft wedged motionless between a goal's pull and an obstacle's push, whereas letting the highest-priority behavior own the command while lower ones use only the residual capability has no such dead-ends. Collision avoidance is deliberately kept out of this scheme as the dedicated layer of section 2, so that it can never be out-prioritized. Formation flight, target assignment, and multi-craft coordination belong to the fleet protocol above this layer and are out of scope.

---

## 12. Deferring a constraint: when guidance binds the arrival axis

A free axis is what lets the controller optimize thrust (section 8), but some constraints guidance will eventually need are not gone — they are merely *not yet active*. A craft cruising to a waypoint five minutes away, able to slew to its arrival pose in five seconds, has no reason to hold that pose for the intervening time. It can leave all three axes free and let the controller optimize thrust throughout the cruise, then re-impose the arrival attitude only in the final seconds. The orientation constraint is real but dormant, and something must decide when to wake it. That decision belongs to guidance, the only layer that knows whether and when a constraint can be released, and it is made by comparing the time the maneuver has left against the time a reorientation would take.

The time remaining, $\tau_{\mathrm{go}}$, is the value each behavior already publishes (section 11) — intercept's range over closing speed, go-to's braking-profile arrival time, an open-ended orbit's effectively unbounded horizon. Because collision avoidance can lengthen the real path after $\tau_{\mathrm{go}}$ is computed, it is treated only as a lower bound on time available, which keeps the decision conservative: a detour can only add time.

The slew time comes from the rotational kinematics, using the effective maximum angular acceleration $\alpha_{\max}=\tau_{\max}/J$ that the inertia observer (section 9.1) publishes per axis. With a geodesic slew angle $\Delta\theta$ and rate limit $\omega_{\max}$,

$$
t_{\mathrm{slew}}=\begin{cases}
2\sqrt{\Delta\theta/\alpha_{\max}}, & \Delta\theta\le\omega_{\max}^2/\alpha_{\max}\quad(\text{triangular profile}),\\[6pt]
\dfrac{\Delta\theta}{\omega_{\max}}+\dfrac{\omega_{\max}}{\alpha_{\max}}, & \text{otherwise (trapezoidal profile)}.
\end{cases}
$$

Guidance binds the arrival axes when

$$
\tau_{\mathrm{go}} \;\le\; \beta\,t_{\mathrm{slew}}^{\mathrm{worst}} + t_{\mathrm{margin}},
$$

with safety factor $\beta>1$ and an absolute buffer $t_{\mathrm{margin}}$. Two refinements keep this from misfiring. First, $t_{\mathrm{slew}}^{\mathrm{worst}}$ is computed for the largest slew the craft could face — the worst-case angle and the most pessimistic $\alpha_{\max}$ — rather than for the current attitude, because while the axes are free the optimizer is rotating the craft and the true slew distance to the arrival pose is wandering. Triggering early against a worst-case estimate is safe; triggering late blows the arrival. Second, the bind is **latching**: once guidance commits to the arrival window it does not release the axes again even if a momentary estimate suggests there is time. The conservatism of the inertia observer's fallback bound is, in this one place, a feature rather than a cost. This timing rule is the only place in the system where $t_{\mathrm{slew}}$, and therefore the inertia estimate, is needed at all.

---

## 13. Limitations and scope

Two boundaries are worth stating plainly. First, the rigid-body model underlying both the gyro response and the inertia observer assumes the craft is a single body. Rotors, hinges, and pistons create separate physics bodies joined by constraints, and a craft with significant articulated mass — a large rotating turret, an extended piston arm — violates that assumption; the inertia estimate degrades gracefully rather than failing, but it is least trustworthy on such craft. In practice most autonomous craft either have no subgrids or carry subgrid mass small enough that the effect is negligible, so this is a documented limitation rather than a defect to engineer around.

Second, the collision-avoidance layer of section 2 is specified here only at the architectural level — it consumes the world model and warps the commanded acceleration. The concrete avoidance law (a potential field for static obstacles, a velocity-obstacle formulation for moving ones) is deferred; the seam it occupies is fixed, but its internals are future work.

---

## 14. Experiments: choosing and validating the inertia observer

*This section records how the inertia estimator was selected and can be skimmed; the autopilot mechanism above does not depend on it.* To choose an estimator without a live game, a minimal simulator was built directly from the gyro source: a rate controller that pre-multiplies the desired angular acceleration by the inertia and clamps to the gyro ceiling, with full rigid-body integration. Its core is small enough to show:

```python
DT = 1/60.0
def gyro_torque(omega, omega_target, J_diag, tau_max, ramp):
    desired_accel  = (omega_target - omega) * (60.0 / ramp)   # rate controller
    desired_torque = desired_accel * J_diag                   # engine multiplies by inertia
    n = norm(desired_torque)
    if n > tau_max:                                           # clamp to gyro ceiling
        return desired_torque * (tau_max / n), True           # saturated
    return desired_torque, False

def step(s, f_body, omega_target):                            # s = craft state
    torque, saturated = gyro_torque(s.omega, omega_target, J_diag, tau_max, ramp)
    alpha   = Jinv @ (torque - cross(s.omega, J @ s.omega))   # Euler's equation
    s.omega += alpha * DT
    s.q      = normalize(s.q + 0.5 * quat_mul(s.q, [0, *s.omega]) * DT)
    accel    = (quat_to_R(s.q) @ clip(f_body, box)) / mass + gravity
    s.v += accel * DT;  s.p += s.v * DT
```

Validation confirmed the simulator behaves: forward thrust produces exactly $F/m$ acceleration, each rate command rotates only its own axis, and — the two diagnostics that matter — a sixteenfold change in inertia leaves the unsaturated slew time unchanged (the controller cancels inertia), while a saturated slew recovers the inertia to rounding via $\tau_{\max}/\alpha$.

The chosen estimator, running on a benchmark of slews with a mid-run cargo load, behaves as the left figure shows: each axis sits at the conservative bound until first slewed, then snaps to truth within a tenth to a fifth of a second and holds it; when the cargo is loaded the truth steps up and each axis re-identifies on its next slew.

![](img/conv.png){ width=86% }

Several more elaborate estimators were tried and, perhaps surprisingly, all did worse than the simple saturated-readout approach: a per-axis inversion of Euler's equation (unstable, because both the coupling term and the torque direction depend on the unknown inertia), a windowed least-squares on the saturation magnitude constraint (under-determined, sliding to spurious solutions), and gates based on low angular rate or on which axis leads the acceleration (noisier, no better). The reason is a genuine observability limit rather than a tuning failure, shown at right: a balanced pitch-plus-yaw slew produces identical angular acceleration on both axes whatever their true inertias, so the measurement reveals only the combined $\sqrt{J_x^2+J_y^2}$ and can never separate them. Per-axis inertia is recoverable only from single-axis-dominant or directionally-diverse excitation — which is exactly what the chosen estimator waits for, and exactly why the clever simultaneous-decomposition schemes cannot beat it.

![](img/methods.png){ width=98% }

---

## Appendix A — Symbol to SE API

| symbol | source |
|---|---|
| $R$ | rotation part of `IMyShipController.WorldMatrix` (body-to-world) |
| $\mathbf{v}$ | `GetShipVelocities().LinearVelocity` (world frame) |
| $\boldsymbol{\omega}$ | `GetShipVelocities().AngularVelocity` — **world frame**; convert $\boldsymbol\omega_{\mathcal B}=R^\top\boldsymbol\omega_{\mathcal W}$ before use |
| $m$ | `CalculateShipMass().PhysicalMass` |
| $\mathbf{g}$ | `GetNaturalGravity()` (world-frame acceleration, points down) |
| $T_i^{\pm}$ | sum of `IMyThrust.MaxEffectiveThrust` per body direction, cached |
| $\tau_{\max}$ | sum of gyro maximum torque; only a saturation threshold, so approximate is fine |
| $J_i,\ \alpha_{\max}$ | identified online by the inertia observer (section 9.1); seeded by the bounding-box bound |
| contact $\mathbf p,\mathbf v$ | camera `Raycast`, `IMyShipSensor`, intergrid reports, then filtered (section 10) |

## Appendix B — Programmable Block budget notes

The per-run limits, verified against the game source, are fifty thousand executed instructions and ten thousand method calls, both instrumented counts rather than measures of CPU time. The autopilot's mathematics is cheap against this; what consumes the budget is block enumeration and string handling. The discipline is therefore to cache every block reference once on construction or recompile, never re-fetch inside the per-tick loop, and keep custom-data parsing and display-string building off the hot path. The control loops can comfortably target the full sixty-hertz update rate, while guidance and world-model upkeep can run at a tenth of that if the budget is tight.

## Appendix C — Claims verified against the game source

The following facts have been confirmed against the archived game source at `E:\dev\repos\SpaceEngineers`:

- `GetShipVelocities().AngularVelocity` is reported in the world frame — confirmed. The implementation in `MyShipController.GetShipVelocities()` passes `myPhysicsComponentBase.AngularVelocity` directly into the return struct without any frame conversion. The separate `AngularVelocityLocal` property in `MyPhysicsComponentBase` is the body-frame counterpart, confirming the base property is world-space.
- The gyro override uses only the diagonal of the inverse inertia tensor — confirmed. `MyGridGyroSystem.UpdateOverriddenGyros()` explicitly extracts diagonal components only: `new Vector3(inverseInertiaTensor.M11, inverseInertiaTensor.M22, inverseInertiaTensor.M33)` — the off-diagonal terms are discarded.

Outstanding (not yet verified in source):

- Per-block mass: `IMySlimBlock.Mass` exists and exposes block mass to scripts, which may contradict the recollection that per-block mass is not exposed. Requires clarification.
- Stored gas is massless: `MyGasTank` source does not contain `FloatVolume` or gas-mass properties in the archived source — requires in-game verification.
