# Plan — Turret absolute-encoder filtering + vernier-decode robustness

Status: **IMPLEMENTED** (JVM suite green, 142 tests). This doc is kept as the design rationale
(noise math + rejected alternatives). The shipped code lives in `control/FadingMemoryFilter.kt` and
`hardware/subsystem/Turret.kt`; on-robot tuning is tracked in `TODO.md` §3 "Turret angle filtering"
and the resolution is noted in `BUGS.md` B2.

---

## 0. TL;DR

The turret reads two noisy absolute analog encoders (12t + 13t idlers on the 137t gear) and
vernier-decodes them into one absolute angle. Noise does **not** meaningfully corrupt the *fine*
angle — the geometry divides encoder noise by ~11.4 — but it **can flip the `round()` that picks the
revolution**, which snaps the output by a full **±31.53°** (one 12t seam). That is the real hazard,
and it is worst exactly at the seams that recur every 31.53° of turret travel.

So "add a filter" is necessary but not sufficient. The plan has **two independent parts**:

1. **A dt-aware smoothing filter** (`control/FadingMemoryFilter.kt`, a g-h / alpha-beta filter) on
   the fused turret angle. Gives a clean position **and** a clean velocity (so the currently-disabled
   turret `kD` damping term becomes usable), robust to fuzzy *and* spiky noise, with an outlier gate.
2. **Revolution-continuity locking** in the decode: after an initial vernier acquisition, pick the
   12t revolution nearest the last known angle instead of re-deriving it from the noise-amplified
   coarse difference every loop. This is physically justified (the turret cannot move >~3.5°/loop, so
   a 31.5° jump between loops is *always* noise) and makes rev-selection essentially noise-immune.
   The memoryless vernier is retained for first acquisition and for re-acquisition after loop stalls.

Everything lands as: one new tested `control/` primitive, surgical `Turret` changes that respect the
read/update/write discipline, new companion tunables with safe placeholders, expanded unit tests, an
upgraded debug OpMode for on-robot tuning, and `TODO.md`/`BUGS.md` updates.

---

## 1. Problem analysis — how noise flows through `fuseAbsoluteAngle`

Let σ = per-encoder raw angular noise (deg) on each idler signal (assume 12t and 13t independent).
Ratios: r12 = 137/12 = 11.4167, r13 = 137/13 = 10.5385, Δr = 0.87821. Seam = 360·12/137 = **31.53°**.

Decode (current `Turret.fuseAbsoluteAngle`):
```
phaseDiff  = normalizeAngle(enc12 - enc13)           # in [-180,180]
coarse     = phaseDiff / Δr
rev        = round( (coarse*r12 - enc12) / 360 )      # == round( (13*phaseDiff - enc12)/360 )
result     = (enc12 + 360*rev) / r12
```

Noise propagation (verified numerically):

| Quantity | Noise (×σ) | Meaning |
|---|---|---|
| Fine angle (given correct rev) = enc12/r12 | **0.088·σ** | position readout is inherently quiet — geometry attenuates |
| `round()` argument = (13·phaseDiff − enc12)/360 | **0.049·σ** | rev decision amplifies phase noise ~13× before rounding |
| Output jump on one wrong `round()` | **31.53°** | catastrophic discontinuity — the failure mode that matters |

Consequences:
- **Mid-tooth** (far from a seam): safe until ~**3.4° of raw noise** (3σ inside the ±0.5 round band).
  Existing TODO says "~6–7°"; 3.4° is the more conservative 3σ figure — same order.
- **At a seam** (every 31.53°): the round argument sits on a `.5` boundary, so *any* noise gives a
  ~50/50 flip → the fused angle **dithers ±31.53° loop-to-loop**. A low-pass placed *after* the fuse
  cannot fix this; it only smears a 31.5° step over time. This is why we also need part 2.
- Physical bound: at MAX_VELOCITY 700°/s and a ~5 ms loop, the turret moves ≤ **3.5°/loop** — far less
  than one seam. So any inter-loop jump near 31.5° is provably noise, never real motion.

---

## 2. Design decisions (and rejected alternatives)

**D1 — Filter the *fused* angle, not the raw idler angles.**
The fused angle is slow (≤3.5°/loop), never wraps within the turret's mechanical range, and carries
only 0.088·σ of noise once the rev is right — so a plain scalar filter on it is simple and correct.
*Rejected:* filtering the raw idlers. They spin ~11× faster and **wrap at 0/360°**, forcing circular
(cos/sin) filtering for a marginal gain over the already-attenuated fine angle. Not worth the
complexity. (We could revisit if hardware shows the fine angle itself is unacceptably noisy.)

**D2 — Solve rev-selection with continuity locking + vernier acquisition, not just filtering.**
Filtering can't undo a wrong `round()`. Continuity (pick the 12t rev nearest the last angle) is
immune to enc12 noise up to enormous levels (a mis-pick needs ~180° of enc12 error) and is physically
justified. The memoryless vernier remains the source of truth for the *first* lock and for
re-acquisition after a loop stall (large dt), where continuity's prior is stale.
*Rejected:* hysteresis on the integer rev only. Continuity subsumes it and also handles stalls.

**D3 — g-h / alpha-beta (constant-velocity) filter, parameterized by a time constant τ.**
Gives filtered **position + velocity** from one intuitive knob (seconds), dt-aware so it is
loop-rate-invariant, and is a steady-state Kalman for a constant-velocity model — without asking the
user to supply process/measurement covariances they cannot estimate.
*Rejected:* full Kalman (needs Q/R we can't characterize; overkill for 1-D). Plain 1st-order low-pass
(no velocity output → can't enable kD; differentiating it re-injects noise).

**D4 — dt-aware gains via a fading-memory discount factor.** Per loop: `θ = exp(-dt/τ)`,
`g = 1 − θ²`, `h = (1 − θ)²` (Brookner fading-memory g-h). τ→0 = snap to measurement, τ large = heavy
smoothing. Single tunable, stable, dt-invariant.

**D5 — Outlier gate inside the filter.** Reject a sample whose innovation exceeds a gate (deg), since
real per-loop motion is ≤3.5°. Self-heals: after N consecutive rejects it forces acceptance so a
genuine step can never wedge the estimate. Handles *spiky* noise without a separate median stage.

**D6 — Respect read/update/write.** `read()` latches raw hardware only (fixes the current minor
impurity where `fuse` runs in `read()`); all decode + filtering is pure computation in `update()`
(it needs `dt`, which `update()` already owns).

**D7 — Placeholder-safe + on-robot-tunable + TODO'd**, exactly like the existing encoder offsets and
shooter constants. No behavior is silently changed (e.g. kD stays 0.0 by default; enabling it becomes
a documented on-robot step now that velocity is trustworthy).

---

## 3. New primitive — `control/FadingMemoryFilter.kt`

Pure, hardware-free, JVM-tested — same shelf as `TrapezoidalProfile`/`ProjectileSolver`. Stateful
(holds position/velocity), so the `Turret` owns one long-lived instance (unlike `TrapezoidalProfile`,
which is stateless and rebuilt each loop). All tunables stay in `Turret`'s companion and are passed
into `update()` each call, so the filter carries **no config of its own** (keeps the single-source
convention).

Illustrative sketch (NOT final code):
```kotlin
package org.firstinspires.ftc.teamcode.control

import kotlin.math.abs
import kotlin.math.exp

/**
 * First-order fading-memory (g-h / alpha-beta) tracking filter over a continuous scalar. Models a
 * constant-velocity target, so each [update] yields a smoothed [position] AND [velocity] estimate
 * from one intuitive knob -- the smoothing time constant tau (seconds). It is a steady-state Kalman
 * filter for a constant-velocity process, without requiring hand-tuned covariances.
 *
 * dt-aware: the discount factor theta = exp(-dt/tau) is recomputed each call, so behaviour is
 * loop-rate invariant. An optional innovation gate rejects outlier samples (sensor spikes); after
 * [maxConsecutiveRejects] rejects it force-accepts so a genuine step can never wedge the estimate.
 *
 * Input must NOT wrap (feed a continuous, unwrapped signal). The turret's fused angle qualifies --
 * it stays within the mechanical travel range and never crosses a 0/360 seam.
 */
class FadingMemoryFilter {
    var position = 0.0; private set
    var velocity = 0.0; private set
    private var initialized = false
    private var consecutiveRejects = 0

    /** Seed the estimate (used on first lock and after a re-acquire); zeroes velocity. */
    fun reset(position: Double) {
        this.position = position
        this.velocity = 0.0
        this.initialized = true
        this.consecutiveRejects = 0
    }

    /**
     * Fuse one [measurement] taken [dt] seconds after the previous update.
     * @param tau smoothing time constant (s); larger = smoother + laggier.
     * @param spikeGate reject |innovation| beyond this (same unit as measurement); Double.MAX = off.
     */
    fun update(dt: Double, measurement: Double, tau: Double,
               spikeGate: Double = Double.MAX_VALUE, maxConsecutiveRejects: Int = 5) {
        if (!initialized) { reset(measurement); return }
        if (dt <= 0.0) return               // hold; caller guards against bad dt too

        val posPred = position + velocity * dt
        val velPred = velocity
        val innovation = measurement - posPred

        if (abs(innovation) > spikeGate && consecutiveRejects < maxConsecutiveRejects) {
            position = posPred; velocity = velPred          // outlier: coast on the model
            consecutiveRejects++
            return
        }
        consecutiveRejects = 0

        val theta = exp(-dt / tau)
        val g = 1.0 - theta * theta
        val h = (1.0 - theta) * (1.0 - theta)
        position = posPred + g * innovation
        velocity = velPred + (h / dt) * innovation
    }
}
```

---

## 4. `Turret.kt` changes

### 4a. `read()` — latch raw hardware only (pure read)
```kotlin
override fun read() {
    motorImpliedAngle = (motor.currentPosition / TICKS_PER_TURRET_REV) * 360.0
    rawVoltage12 = encoder12.voltage    // new fields
    rawVoltage13 = encoder13.voltage
}
```
(deg-conversion + fuse + filter move to `update()`, which owns `dt`.)

### 4b. `update()` — decode + lock + filter at the top, before the profile/PID
New fields: `rawVoltage12/13`, `locked: Boolean`, `private val angleFilter = FadingMemoryFilter()`.
Delete the finite-difference `measuredVelocity`/`lastAngle` bookkeeping; velocity now comes from the
filter.

```kotlin
// aim first (unchanged): sets targetAngle from follower pose, needs no currentAngle
if (aimAtGoal) { ... face(...) ... }

val enc12Deg = wrapTo360(voltageToDegrees(rawVoltage12) - ENCODER_12T_ZERO_OFFSET_DEG)
val enc13Deg = wrapTo360(voltageToDegrees(rawVoltage13) - ENCODER_13T_ZERO_OFFSET_DEG)

val now = timeSource.seconds()
var dt = now - lastTime
lastTime = now
if (dt <= 0.0 || dt > MAX_DT) dt = 0.0     // stall/first-frame -> treat as re-acquire

if (!locked || dt == 0.0) {
    // (Re)acquire: memoryless vernier is correct even if the turret was back-driven while disabled
    // or across a loop stall. Seed the filter here so we never smooth across the gap.
    currentAngle = fuseAbsoluteAngle(enc12Deg, enc13Deg)
    angleFilter.reset(currentAngle)
    locked = true
} else {
    // Track: pick the 12t revolution nearest the last angle (noise-immune), then smooth.
    val measured = selectRevolutionByContinuity(enc12Deg, angleFilter.position)
    angleFilter.update(dt, measured, ANGLE_FILTER_TAU, ANGLE_FILTER_SPIKE_GATE)
    currentAngle = angleFilter.position
}
measuredVelocity = angleFilter.velocity

// ... existing profileInitialized seeding + TrapezoidalProfile + FF/PID law unchanged below ...
```

### 4c. New decode helper — continuity revolution pick
Same algebra as the fuse's last step, but the rev is chosen from the prior angle, not the coarse
difference:
```kotlin
private fun selectRevolutionByContinuity(enc12Deg: Double, priorAngle: Double): Double {
    val rev = round((priorAngle * ENCODER_12T_GEAR_RATIO - enc12Deg) / 360.0)
    return (enc12Deg + 360.0 * rev) / ENCODER_12T_GEAR_RATIO
}
```
`fuseAbsoluteAngle` is **unchanged** (still the acquisition path + still what `TurretTest`'s algebra
tests exercise).

### 4d. `reset()` — add `locked = false` (and `angleFilter` re-seeds on first `update`).

### 4e. Companion tunables (safe placeholders, on-robot-tunable, TODO'd)
```kotlin
// Absolute-angle smoothing (FadingMemoryFilter). Larger tau = smoother but laggier; the turret
// goal-locks a MOVING target, so keep tau small. Gate rejects noise spikes (real motion <= ~3.5°/loop).
var ANGLE_FILTER_TAU = 0.02          // s  (~8 Hz; mild smoothing, ~one-loop lag) -- TUNE ON ROBOT
var ANGLE_FILTER_SPIKE_GATE = 10.0   // deg innovation reject threshold          -- TUNE ON ROBOT
```
`MAX_DT` (already 0.1 s) doubles as the acquire/track boundary — no new constant.
`kD` stays `0.0` but its comment changes from "encoder is noisy" to "velocity now filtered — enable
+ tune on robot"; enabling it becomes a TODO step, not a silent change.

---

## 5. Debug OpMode + telemetry

**`TurretEncoderDebug.kt`** (`opmode/debug`): currently calls only `turret.read()` to populate the
fused angle — that breaks once the fuse/filter move to `update()`. Change to `turret.read();
turret.update()` (it already never calls `write()`; the open-loop jog writes `motor.power` directly,
so the computed `motorPower` is simply ignored — safe). Add telemetry so the filter is tunable live:
- raw fused (memoryless) angle vs. **filtered** `currentAngle` — watch the jitter shrink,
- `measuredVelocity` (deg/s) — should sit near 0 at rest, ramp smoothly on jog,
- `locked` state and a re-acquire indicator,
- the two new tunables (`ANGLE_FILTER_TAU`, `ANGLE_FILTER_SPIKE_GATE`).

Optional (nice-to-have): annotate `Turret` `@Configurable` so τ/gate/kD are dashboard-live-tunable
(it currently is not). Deferred unless you want live tuning — the existing gains aren't `@Configurable`
either, so this would be a small convention change, called out but not assumed.

`AnalogEncoderReadout.kt` (raw per-encoder wiring check): **unchanged**.

---

## 6. Tests (`./gradlew :TeamCode:testDebugUnitTest`)

### New — `control/FadingMemoryFilterTest.kt`
- Constant input → position converges to it, velocity → 0.
- Constant-velocity ramp → velocity estimate → true slope; position lag bounded.
- White-noise input (seeded RNG) → filtered std < raw std by the expected factor.
- Single large spike → rejected (position barely moves); sustained offset → force-accept after N.
- dt-invariance: two different dt schedules at the same τ give ~matching time-domain response.
- `reset` seeds position and zeroes velocity.

### Update — `TurretTest.kt`
- **Decode algebra tests** (`crtDecode_*`): `currentAngle` now populates in `update()`, not `read()`.
  Adapt the `applyTurretAngle` helper to `read(); update()` **without advancing the clock** (dt==0 →
  acquire path → `currentAngle == fuseAbsoluteAngle` exactly). Intent preserved: still pure,
  zero-noise vernier algebra with `decode(forward(theta)) == theta`.
- **New robustness tests** (advance the clock, small dt):
  - `tracking_survivesNoiseAtSeam`: sweep across a seam with injected per-loop voltage noise; assert
    the output never jumps ~31.5° (continuity holds where the old memoryless round would dither).
  - `tracking_rejectsSpike`: one-loop voltage spike → `currentAngle` barely moves.
  - `largeDtReacquires`: after dt>MAX_DT with the turret moved >1 seam, output re-acquires to the true
    (vernier) angle instead of tracking a wrong rev.
  - `velocityEstimateTracksConstantRate`: constant-rate angle ramp → `measuredVelocity` → true rate
    (justifies enabling kD).
- Existing control-law tests still pass — the profile/FF/PID block is untouched; only the source of
  `currentAngle`/`measuredVelocity` changed.

`FakeAnalogInput` (settable voltage) + `FakeTimeSource` (hand-advanced clock) already exist — no new
fakes.

---

## 7. `TODO.md` / `BUGS.md` updates (Hard Rule: update TODO on setup-affecting changes)

- Resolve/convert the **B2/B3** "decode noise margin" item (TODO ~241): the outlier/rate guard it
  asked for now exists — reframe as "tune `ANGLE_FILTER_TAU` / `ANGLE_FILTER_SPIKE_GATE`."
- Add to the turret PID section (TODO ~162): with velocity now filtered, **enable + tune `kD`**.
- Add a filter-tuning entry under the turret encoder section with `file:line` refs and pointing at the
  upgraded **Turret Encoder Debug** as the tool that produces the values (matches the existing
  "debug OpMode → constant" pattern).

---

## 8. On-robot tuning procedure (goes in TODO)

1. Calibrate the zero offsets + gear-mesh sign first (existing §3 gate) — filtering a mis-decoded
   angle is meaningless.
2. Run **Turret Encoder Debug**, turret at rest: read *filtered* jitter. Raise `ANGLE_FILTER_TAU`
   until the position is steady; stop before jog tracking feels laggy.
3. Jog the turret: `measuredVelocity` should be smooth. Set `ANGLE_FILTER_SPIKE_GATE` just above the
   worst clean per-loop delta so it clips only gross spikes.
4. With velocity clean, enable `kD` and raise it until motion is critically damped (no overshoot ring).
5. Sweep the full travel including seams: confirm the fused angle never steps 31.5° at speed.

---

## 9. Risks / mitigations

| Risk | Mitigation |
|---|---|
| Continuity locks onto a wrong rev after the turret is back-driven while disabled | `reset()` clears `locked`; first `update()` re-acquires via memoryless vernier |
| Loop stall (dt large) while slewing → stale prior picks wrong rev | dt>MAX_DT forces re-acquire via vernier (memoryless, correct after any move) |
| enc13 dead/unplugged | Only used at acquisition; tracking rides enc12. Still a hardware fault — caught by `AnalogEncoderReadout`. Note in debug OpMode. |
| Filter lag hurts shoot-on-the-move aim | τ small (≈ one loop) by default; tunable; velocity feed-through (g-h predicts) limits lag |
| Over-aggressive spike gate clips real motion | Real motion ≤3.5°/loop ≪ default 10° gate; self-heal after N rejects; tunable |
| Behavior change slips silently | kD default unchanged (0.0); all new knobs are placeholders + TODO'd; algebra tests pin the decode |

---

## 10. File-by-file change list

| File | Change |
|---|---|
| `control/FadingMemoryFilter.kt` | **new** — g-h/alpha-beta filter primitive |
| `test/control/FadingMemoryFilterTest.kt` | **new** — unit tests for the primitive |
| `hardware/subsystem/Turret.kt` | `read()` latches raw voltages; `update()` decode+lock+filter; `selectRevolutionByContinuity`; `locked` field + `angleFilter`; drop finite-diff velocity; new companion tunables; kD comment |
| `test/.../TurretTest.kt` | adapt decode-algebra helper to `read()+update()`; add noise/seam/spike/reacquire/velocity tests |
| `opmode/debug/TurretEncoderDebug.kt` | `read()`→`read()+update()`; add filtered-vs-raw + velocity + lock telemetry |
| `TODO.md` | reframe B2/B3 item; add filter-tuning + enable-kD steps |
| `BUGS.md` | mark B2/B3 addressed (guard implemented; tune on robot) |

Not touched: `AnalogEncoderReadout.kt`, `fuseAbsoluteAngle` algebra, the profile/FF/PID law,
`Robot.kt` wiring, `Names.kt`.
```
