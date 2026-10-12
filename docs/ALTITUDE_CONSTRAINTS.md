# Commanded altitude constraints and frozen historical evaluations

The published 6,000-encounter evaluation and 29-case mutation gate belong to
[the original reliability-review source](https://github.com/LUOaini1213/evtol-daa-sim/tree/0e8566360e02e7d2c5afb45018578ea80c05f13a).
Their JSON, CSV, figures and mutation evidence remain unchanged. They do **not**
evaluate the current altitude-constrained controller. No new Monte-Carlo run,
aggregate improvement, render or flight-safety certification is claimed here.

## The bounded correction

A legal altitude target did not ensure a legal trajectory. With the existing
1 s Euler controller, initial commanded altitude 190 m, zero vertical rate and
target 150 m, the original controller reached **147.45 m**. The planner could
select this target with `free=true` and `limited=false`; it checked target height
and aircraft separation but did not prevent its own altitude-capture overshoot.
The symmetric ceiling case also overshot 600 m.

Prediction and execution now call the same bounded vertical controller. Euler
position still advances using the velocity at the **start** of the step. The
next vertical velocity respects the existing rate and acceleration limits and
the distance required to stop before either altitude boundary. With rate `q`,
decrement `delta = a_z * dt` and `n` positive-rate steps, that distance is:

`dt * (n*q - delta*n*(n-1)/2)`

For the configured 5 m/s rate, 2.45 m/s² acceleration and 1 s step, braking needs
`5 + 2.55 + 0.1 = 7.65 m`, rather than the continuous-time `q²/(2*a_z)` estimate.
The bounded controller can capture the legal 150/600 m targets without overshoot,
instantaneous position changes or extra acceleration. Existing feasible interior
commands retain their original motion. `limited`/`floor_limited` now also report
intervention by this controller, in addition to target exclusion.

The zero-altitude candidate and the current command no longer bypass target
eligibility. Initial altitude and the held-velocity step are checked, including
the response-delay interval. A state that cannot stop within the band under the
configured acceleration raises `ValueError`; the simulator does not fabricate
a recovery by moving the aircraft. The 1e-9 numerical budget is round-off only.

## The physical boundary of this guarantee

The band constrains **commanded, nominal Euler altitude**, the state controlled
by the capture law. The original OU gust sequence remains an unbounded external
disturbance on truth position. Neither `own_pos`, the gust distribution nor the
LoWC/NMAC truth checks are clipped. A gust can therefore move true altitude out
of the band; this change does not certify disturbed trajectories or real flight.
A regression explicitly leaves a 10 m/s vertical disturbance visible in truth,
while commanded altitude stays 300 m. Future disturbance prediction and aircraft
flight-control validation remain outside this software correction.

## Verification and original-source audit

Install the pinned `requirements.txt`, then run:

```sh
python -m pytest -q
python scripts/check_archived_results.py
python scripts/make_readme.py --check
```

`tests/test_altitude_band.py` covers the original floor/ceiling examples, actual
`plan` and `closed_loop` calls, the hand-computed braking boundary, rates,
acceleration, dt, delayed/current commands, unchanged feasible controls and
infeasible states. These are deterministic local controls, not a new encounter
distribution or a claim about the archived 6,000 cases.

`docs/historical_evaluation.json` binds all eight historical artifacts to their
fixed original commit. The audit checks the evaluation's nine original runtime
source hashes, the mutation gate's 26 source/test/config hashes and all 90 ZIP
evidence members. The two original runs used different mixed LF/CRLF checkout
bytes. Their per-file policies reconstruct those **recorded raw bytes** from the
original Git commit; the historical hashes are not rewritten. Current sources
are compared separately, ignoring checkout line endings, so a passing historical
audit cannot be mistaken for an evaluation of the new controller. CI fetches
the original history and runs this audit separately from current-engine tests.
The earlier first-run mutation report is byte-preserved only; its original
runtime is not recertified by this audit. The local mutation copier excludes
private virtual environments while retaining tested source directories.

### Local validation receipt (not a completed required CI run)

[The current receipt](altitude_constraints_review.json) records the source
fingerprints and actual validation boundaries. The required-mode local suite
had 286 passing cases and one setup error: this Windows host has no Visual
Studio `vswhere`, required by the existing forced-MSVC Unicode-path test. That
gate remains enabled. The final 42 controller and nine archive regressions
passed after the extra target-eligibility test; the 22 mutation-runner controls
also passed. These overlapping runs are not advertised as a full-suite pass.

The eight identical public `closed_loop` controls against the original source
produced six assertion failures (the dt=1 floor/ceiling trajectories with
0/5/10 s delays) and two passes (already feasible dt=0.5 controls), with no setup
errors. The archived runtime hashes were independently matched before this
comparison. Current deterministic boundary captures stay at 150/600 m.

An optional GCC Unicode/special-character path build also failed locally; it
does not replace the protected MSVC check or ordinary Python/C++ agreement.
The existing 29 product-mutation specifications were adapted to the new target
floor expression, but no new 29-case product gate was run. The archived 29/29
continues to describe only its fixed original source.
