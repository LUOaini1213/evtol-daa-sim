# Commanded altitude constraints

This note describes the correction that bounds the commanded altitude trajectory
of the avoidance controller (merged in PR #1, commit `1c9dd54`). The README results
are a fresh 6,000-encounter run of the controller with this correction. The
previous run, made before it, is kept in [historical/](historical/README.md).

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

## Verification

```sh
python -m pytest -q
python scripts/make_readme.py --check
python scripts/check_archived_results.py   # needs full Git history
```

`tests/test_altitude_band.py` covers the original floor/ceiling examples, actual
`plan` and `closed_loop` calls, the hand-computed braking boundary, rates,
acceleration, dt, delayed/current commands, unchanged feasible controls and
infeasible states. Run against the pre-correction source, eight of these
`closed_loop` controls gave six assertion failures (the dt=1 floor/ceiling
trajectories with 0/5/10 s delays) and two passes (controls that were already
feasible at dt=0.5).

CI runs the full suite with the C++ core required on Ubuntu and Windows; the
merge commit of this correction passed
([run 38206288076](https://github.com/LUOaini1213/evtol-daa-sim/actions/runs/38206288076)),
as did the following commit
([run 38419229475](https://github.com/LUOaini1213/evtol-daa-sim/actions/runs/38419229475)).
The local receipt written before CI existed for this change is kept as
[historical/altitude_constraints_review.json](historical/altitude_constraints_review.json).
How the historical files are audited is described in [PROVENANCE.md](PROVENANCE.md).
