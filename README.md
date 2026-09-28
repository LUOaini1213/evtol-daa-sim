# eVTOL detect-and-avoid alerting and avoidance simulation

A personal project built in 2026 by Wenjie Luo. It compares detect-and-avoid (DAA)
alerting logics and avoidance manoeuvres for an eVTOL in low-altitude airspace, using
synthetic Monte-Carlo encounters. It is not certified, not derived from any real system,
and none of the encounter data comes from real traffic.

What is in the repository:

- `daa/` - Python simulator: encounter generator, surveillance and tracker model, three
  alerting logics with three alert tiers, four manoeuvre policies, metrics.
- `cpp/daa_core.hpp` - a small C++17 version of the CPA / well-clear core and the tiered
  alert function, with a command-line front end (`cpp/daa_cli.cpp`). The tests feed the same
  inputs to both implementations and require identical outputs.
- `scripts/run_eval.py` - the full evaluation (a few minutes on a laptop CPU; the measured
  run time is printed with the results), writing `results/` and `docs/*.png`.
- `scripts/make_readme.py` - renders the results section below from `results/summary.json`;
  `--check` fails if this README no longer matches the results files.
- `scripts/mutation_check.py` - plants deliberate bugs one at a time and checks that the test
  suite catches each one.

## Method

**Encounters.** Each encounter pairs the ownship (a level eVTOL heading along +x) with one
intruder: another eVTOL, a small multirotor drone or a light general-aviation (GA) aircraft.
The geometry class is head-on, crossing (30-150 deg), overtaking or vertical (intruder
climbing or descending through the ownship altitude). Each encounter is specified at the
closest point of approach (CPA) of the undisturbed straight-line paths - time to CPA
80-120 s, horizontal miss distance 0-1500 m, vertical offset +-300 m - and then propagated
back to t = 0. Encounters that start inside the well-clear volume, lose well clear in less
than 60 s, close slower than 5 m/s, or put the intruder outside 30-1200 m altitude are
rejected and counted. 20% of intruders make an unannounced 30-90 deg turn 10-60 s before
the nominal CPA, which a straight-line predictor cannot anticipate.

**Disturbances.** A steady uniform wind moves both aircraft equally and cancels out of the
relative motion, so it is not simulated. Gusts are modelled as an independent
Ornstein-Uhlenbeck velocity disturbance on each aircraft (time constant 20 s).

**Surveillance.** Ownship state is known exactly. Cooperative intruders (eVTOL, GA) report
position and velocity with noise; drones are non-cooperative and report position only.
Reports are lost according to a two-state (Gilbert) burst model. An alpha-beta tracker per
axis produces the intruder track; its gains come from the steady-state Kalman relations for
the constant-velocity model (checked against a Riccati iteration in the tests). Three
conditions: ideal (track = truth), nominal, degraded (3x noise, about 20% report loss).

**Alerting logics.** All three use three tiers - preventive, corrective, warning - and the
alert level is the highest tier whose condition holds.

| logic | condition for a tier |
|---|---|
| CPA thresholds | intruder already inside the cylinder, or horizontal time to CPA in (0, T] with horizontal miss distance <= Dh and vertical separation at that time <= Dv |
| WCV | predicted well-clear violation (DAIDALUS `WCV_TAUMOD` test, TTHR = 0 s) at any time in [0, T] |
| WCV + tau_mod 20 s | as WCV with modified-tau threshold TTHR = 20 s |

The horizontal and vertical thresholds and alert times are the same for all three logics,
so the comparison isolates the logic rather than the numbers. With equal thresholds, a CPA
alert implies a WCV alert (at t = tcpa <= T the intruder is inside both bands; a test checks
this on 20,000 random states), so WCV can only add alerts: it trades nuisance alerts for
earlier ones, and the threshold sweeps compare the logics at equal nuisance rates. The well-clear violation test
is computed analytically as an interval of time (horizontal interval from the modified-tau
quadratic, intersected with the vertical interval), following the DAIDALUS source.
Optionally the alert level is filtered: a level is raised only when present in 2 of the last
4 cycles and held until it has been absent for 5 cycles ("filtered" below; "raw" is
unfiltered).

**Avoidance.** On the first corrective alert the planner predicts the ownship path for 90 s
for each candidate manoeuvre (after a 5 s response delay, using the ownship turn-rate,
climb-rate and acceleration limits) against the intruder track extrapolated in a straight
line, and picks the cheapest candidate whose predicted separation stays outside 1.2x the
well-clear distances; if none does, the one with the largest predicted separation.

| policy | candidates |
|---|---|
| none | baseline, no manoeuvre |
| heading | heading change +-10 ... 90 deg; on ties the right turn is preferred (14 CFR 91.113 head-on and overtaking rules) |
| vertical | altitude change +-40 ... 200 m, only targets inside the 150-600 m band |
| combined | heading {0, +-20, +-40, +-60, +-90} deg x altitude {0, +-80, +-160} m |

On a warning alert every non-baseline policy escalates once to an emergency manoeuvre:
heading x altitude x {current speed, slow to 20 m/s}, choosing the largest predicted
separation regardless of cost. While a corrective or warning alert persists, the
manoeuvre in force is re-checked every 5 s against the latest track and replaced (after the
same response delay) if it is no longer predicted clear; section 5 of the results compares
this with a one-shot manoeuvre. The intruder never manoeuvres in response.

**Metrics.** The hazard is loss of well clear (LoWC) of the true, unmitigated trajectories,
checked in continuous time between 1 s samples.

- missed-or-late alert: a hazardous encounter whose corrective alert episode that is active
  at (or started most recently before) LoWC began less than 25 s before LoWC, or never. A
  flickering alert therefore counts from its latest restart; the table also shows the
  lenient version that credits the first alert.
- nuisance alert: a non-hazardous encounter that received any corrective or warning alert.
- lead time: LoWC onset minus start of that alert episode.
- with avoidance: LoWC and NMAC rates, LoWC risk ratio against no manoeuvre (paired
  bootstrap), share of encounters manoeuvred, share manoeuvred when no manoeuvre was
  needed, heading/altitude change, emergency escalations, and LoWC induced in encounters
  that were not hazardous without avoidance.
- system operating characteristic: the corrective alert time T (0-120 s) and, separately, a
  scale on the thresholds (0.5-2.0) are swept to trace missed-or-late against nuisance.

All strategies are evaluated on the same encounters, tracks and gust sequences, so
differences are paired (exact McNemar test and paired bootstrap).

## Sources and assumptions

Verified sources (fetched 2026-09-28):

- **[DAIDALUS]** NASA DAIDALUS v2.0.4a, `Configurations/DO_365B_no_SUM.conf` and
  `Java/src/gov/nasa/larcfm/ACCoRD/WCV_TAUMOD.java`, `WCV_tvar.java`,
  https://github.com/nasa/daidalus at commit `1c7b58e5525d5cca91cbbf4a8b7337a488243769`.
  DAIDALUS is NASA's open-source reference implementation of the RTCA DO-365 DAA alerting
  logic. The DO-365 document itself was not read; values below are as encoded in DAIDALUS.
- **[14 CFR 91.113]** US right-of-way rules, eCFR: aircraft approaching head-on each alter
  course to the right; an overtaking aircraft alters course to the right.
- **[Directory]** J. Meszaros, "Chinese eVTOL Aircraft Directory, Part 1", evtol.news
  (Vertical Flight Society), 2 May 2025,
  https://evtol.news/news/chinese-evtol-aircraft-directory-part-1 - quoted cruise speeds of
  passenger eVTOL designs between 200 and 300 km/h.

| parameter | value | source |
|---|---|---|
| well-clear volume (hazard) | DTHR 2200 ft (670.56 m), ZTHR 450 ft (137.16 m), TTHR 0 s | DAIDALUS `DWC_Non_Coop`; applying it to eVTOL traffic is an assumption |
| NMAC volume | 500 ft (152.4 m) horizontal, 100 ft (30.48 m) vertical | DAIDALUS `horizontal_nmac`, `vertical_nmac` |
| corrective / warning alert time | 55 s / 25 s | DAIDALUS `DWC_Non_Coop` alert_2 / alert_3 |
| preventive tier | ZTHR 700 ft (213.36 m), 55 s | ZTHR from DAIDALUS `DWC_Phase_I` alert_1; combining it with the Non-Coop DTHR is an assumption |
| alert filter | 2 of 4 cycles to raise, hold 5 cycles | M, N and 5 s from DAIDALUS (`alerting_m`, `alerting_n`, `hysteresis_time`); the filter itself is a simplification, not DAIDALUS's algorithm |
| modified-tau threshold of "WCV + tau_mod" | TTHR 20 s | assumption (DAIDALUS Phase I uses 35 s with a larger DTHR) |
| CPA logic thresholds | same as WCV | assumption (chosen for a like-for-like comparison) |
| required alert lead time | 25 s | assumption |
| response delay | 5 s | assumption |
| planner lookahead / buffer / re-check period | 90 s / 1.2x / 5 s | assumption |
| altitude band | 150-600 m | assumption |
| altitude-capture gain | 0.5 1/s | assumption |
| eVTOL speed | 55-70 m/s | range chosen inside the 200-300 km/h cruise speeds in [Directory]; the range itself is an assumption |
| eVTOL turn rate | g tan(30 deg)/V (4.6-5.9 deg/s over 55-70 m/s) | bank limit 30 deg is an assumption |
| eVTOL climb/descent, vertical accel., longitudinal accel. | 5 m/s, 0.25 g, 2 m/s^2 | assumption (0.25 g and 2 m/s^2 equal the DAIDALUS kinematic defaults, which are not eVTOL data) |
| drone / GA speed, climb | 8-20 m/s, 3 m/s / 45-65 m/s, 4 m/s | assumption |
| gusts | OU, sigma 1.0 / 2.0 / 1.0 m/s horizontal (eVTOL / drone / GA), tau 20 s | assumption |
| sensor noise | cooperative 10 m / 8 m / 0.5 m/s; non-cooperative 25 m / 20 m; degraded x3 | assumption |
| report loss | nominal P(lose)=0.02, P(recover)=0.5; degraded 0.05, 0.2 | assumption |
| encounter distribution | see Method | assumption, not fitted to traffic data |

Every parameter is in `config/default.json` with its source string.

## Results

<!-- BEGIN GENERATED RESULTS (scripts/make_readme.py) -->
All numbers below are produced by `scripts/run_eval.py` (seed 20260928, 6000 encounters) and rendered by `scripts/make_readme.py`. Brackets are 95% Wilson intervals for rates and 95% paired-bootstrap intervals for differences and ratios.

Encounter set: 6000 accepted of 12064 sampled (rejected: 175 slow closure, 230 loss too early, 1995 intruder altitude out of range). Geometry: head-on 1739, crossing 1705, overtaking 1437, vertical 1119. Intruder: eVTOL 2000, drone 2054, GA 1946; 1235 intruders make an unannounced turn. Without avoidance, 1249/6000 = 20.8% [19.8, 21.9] of encounters lose well clear and 66/6000 = 1.1% [0.9, 1.4] reach NMAC. Surveillance report loss: nominal 3.8%, degraded 19.6%. Evaluation run time: 261 s (Python 3.11.5, numpy 1.24.3).

### 1. Alerting performance (open loop, no avoidance), nominal surveillance, filtered alerts

| strategy | missed or late (lead < 25 s) | nuisance | median lead (s) | 10th pct lead (s) | alert episodes per hazardous enc. |
|---|---|---|---|---|---|
| CPA thresholds | 20.8% [18.7, 23.2] | 11.8% [10.9, 12.7] | 42.9 | 13.7 | 1.13 |
| WCV | 12.6% [10.8, 14.5] | 16.4% [15.4, 17.5] | 51.9 | 21.5 | 1.17 |
| WCV + tau_mod 20 s | 9.8% [8.3, 11.6] | 21.8% [20.7, 23.0] | 62.8 | 25.9 | 1.21 |

Threshold sweeps at matched nuisance: lowest missed-or-late rate among sweep points whose nuisance rate is at most the given cap (figure below shows all points).

| nuisance cap | swept parameter | CPA thresholds | WCV | WCV + tau_mod 20 s |
|---|---|---|---|---|
| <= 10% | alert time T | 23.2% [21.0, 25.6] at 50 s | 14.2% [12.3, 16.2] at 35 s | 13.2% [11.4, 15.2] at 25 s |
| <= 10% | threshold scale | 29.0% [26.5, 31.6] at 0.9x | 32.1% [29.6, 34.7] at 0.8x | 34.1% [31.5, 36.8] at 0.7x |
| <= 15% | alert time T | 18.8% [16.7, 21.1] at 60 s | 12.8% [11.1, 14.8] at 50 s | 10.8% [9.2, 12.7] at 35 s |
| <= 15% | threshold scale | 20.8% [18.7, 23.2] at 1x | 22.2% [20.0, 24.6] at 0.9x | 18.4% [16.4, 20.7] at 0.9x |
| <= 20% | alert time T | 16.5% [14.5, 18.7] at 75 s | 12.5% [10.8, 14.4] at 60 s | 10.0% [8.5, 11.8] at 45 s |
| <= 20% | threshold scale | 14.7% [12.9, 16.8] at 1.1x | 12.6% [10.8, 14.5] at 1x | 18.4% [16.4, 20.7] at 0.9x |

### 2. Surveillance quality and alert filtering

| surveillance | strategy | missed/late raw | missed/late filtered | nuisance raw | nuisance filtered | missed/late if first alert counted |
|---|---|---|---|---|---|---|
| ideal | CPA thresholds | 22.7% [20.5, 25.1] | 19.2% [17.1, 21.5] | 9.3% [8.5, 10.2] | 8.0% [7.2, 8.8] | 12.2% [10.5, 14.2] |
| ideal | WCV | 15.1% [13.3, 17.2] | 11.1% [9.5, 13.0] | 12.4% [11.5, 13.4] | 11.0% [10.2, 12.0] | 4.9% [3.8, 6.2] |
| ideal | WCV + tau_mod 20 s | 12.7% [10.9, 14.6] | 9.6% [8.1, 11.4] | 15.8% [14.8, 16.9] | 14.1% [13.2, 15.1] | 3.7% [2.8, 4.9] |
| nominal | CPA thresholds | 33.1% [30.6, 35.8] | 20.8% [18.7, 23.2] | 15.5% [14.5, 16.5] | 11.8% [10.9, 12.7] | 11.9% [10.2, 13.8] |
| nominal | WCV | 24.3% [22.0, 26.8] | 12.6% [10.8, 14.5] | 20.3% [19.2, 21.5] | 16.4% [15.4, 17.5] | 5.1% [4.0, 6.5] |
| nominal | WCV + tau_mod 20 s | 19.1% [17.0, 21.3] | 9.8% [8.3, 11.6] | 26.0% [24.8, 27.3] | 21.8% [20.7, 23.0] | 3.6% [2.7, 4.8] |
| degraded | CPA thresholds | 63.9% [61.2, 66.5] | 41.9% [39.2, 44.6] | 28.2% [27.0, 29.5] | 21.0% [19.9, 22.2] | 31.9% [29.4, 34.6] |
| degraded | WCV | 54.0% [51.2, 56.7] | 34.3% [31.7, 36.9] | 33.8% [32.5, 35.2] | 26.7% [25.5, 28.0] | 25.4% [23.0, 27.9] |
| degraded | WCV + tau_mod 20 s | 41.6% [38.9, 44.4] | 30.4% [27.9, 33.0] | 39.3% [37.9, 40.7] | 31.4% [30.1, 32.8] | 23.1% [20.9, 25.6] |

### 3. Paired comparisons on the same encounters (filtered alerts)

| surveillance | a - b | metric | n | difference | discordant a-only / b-only | McNemar exact p |
|---|---|---|---|---|---|---|
| nominal | WCV - CPA thresholds | missed_or_late | 1249 | -8.2 pts [-9.8, -6.6] | 7 / 110 | 6.4e-25 |
| nominal | WCV - CPA thresholds | nuisance | 4751 | +4.7 pts [+4.0, +5.2] | 221 / 0 | 5.9e-67 |
| nominal | WCV + tau_mod 20 s - WCV | missed_or_late | 1249 | -2.7 pts [-3.7, -1.9] | 0 / 34 | 1.2e-10 |
| nominal | WCV + tau_mod 20 s - WCV | nuisance | 4751 | +5.4 pts [+4.8, +6.1] | 256 / 0 | 1.7e-77 |
| nominal | WCV + tau_mod 20 s - CPA thresholds | missed_or_late | 1249 | -11.0 pts [-12.8, -9.1] | 4 / 141 | 8.1e-37 |
| nominal | WCV + tau_mod 20 s - CPA thresholds | nuisance | 4751 | +10.0 pts [+9.2, +10.9] | 477 / 0 | 5.1e-144 |
| degraded | WCV - CPA thresholds | missed_or_late | 1249 | -7.6 pts [-9.4, -5.8] | 19 / 114 | 1.1e-17 |
| degraded | WCV - CPA thresholds | nuisance | 4751 | +5.7 pts [+5.0, +6.3] | 269 / 0 | 2.1e-81 |
| degraded | WCV + tau_mod 20 s - WCV | missed_or_late | 1249 | -3.8 pts [-5.0, -2.7] | 1 / 49 | 9.1e-14 |
| degraded | WCV + tau_mod 20 s - WCV | nuisance | 4751 | +4.7 pts [+4.1, +5.3] | 224 / 0 | 7.4e-68 |
| degraded | WCV + tau_mod 20 s - CPA thresholds | missed_or_late | 1249 | -11.4 pts [-13.3, -9.5] | 13 / 156 | 2.7e-32 |
| degraded | WCV + tau_mod 20 s - CPA thresholds | nuisance | 4751 | +10.4 pts [+9.5, +11.2] | 493 / 0 | 7.8e-149 |

### 4. Closed loop: avoidance manoeuvres

| surveillance | alerting + policy | loss of well clear | NMAC count (rate) | LoWC risk ratio | manoeuvred | manoeuvred when not needed | mean max heading change (deg) | mean max altitude change (m) | emergency escalation | induced LoWC |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | no manoeuvre | 20.8% [19.8, 21.9] | 66 (1.10% [0.87, 1.40]) | 1 | 0.0% | 0.0% | - | - | 0.0% | 0 |
| nominal | CPA thresholds + heading | 1.4% [1.1, 1.7] | 1 (0.02% [0.00, 0.09]) | 0.067 [0.054, 0.080] | 29.8% | 11.6% | 20.3 | 2.0 | 3.0% | 2 |
| nominal | CPA thresholds + vertical | 3.5% [3.1, 4.0] | 1 (0.02% [0.00, 0.09]) | 0.170 [0.150, 0.190] | 29.4% | 11.2% | 5.9 | 122.3 | 6.3% | 2 |
| nominal | CPA thresholds + combined | 1.2% [1.0, 1.5] | 1 (0.02% [0.00, 0.09]) | 0.059 [0.047, 0.071] | 29.8% | 11.6% | 23.2 | 5.0 | 2.6% | 3 |
| nominal | WCV + heading | 0.7% [0.5, 0.9] | 0 (0.00% [0.00, 0.06]) | 0.032 [0.023, 0.042] | 33.7% | 16.4% | 18.5 | 2.1 | 3.0% | 1 |
| nominal | WCV + vertical | 1.1% [0.9, 1.4] | 0 (0.00% [0.00, 0.06]) | 0.052 [0.040, 0.064] | 33.4% | 16.0% | 6.9 | 119.3 | 7.4% | 1 |
| nominal | WCV + combined | 0.5% [0.4, 0.8] | 0 (0.00% [0.00, 0.06]) | 0.026 [0.017, 0.035] | 33.7% | 16.4% | 22.3 | 4.0 | 2.6% | 2 |
| nominal | WCV + tau_mod 20 s + heading | 0.6% [0.4, 0.8] | 0 (0.00% [0.00, 0.06]) | 0.029 [0.020, 0.038] | 37.8% | 21.5% | 16.9 | 1.4 | 5.6% | 2 |
| nominal | WCV + tau_mod 20 s + vertical | 0.5% [0.4, 0.8] | 0 (0.00% [0.00, 0.06]) | 0.026 [0.017, 0.035] | 37.6% | 21.3% | 7.4 | 108.2 | 11.9% | 2 |
| nominal | WCV + tau_mod 20 s + combined | 0.4% [0.3, 0.7] | 0 (0.00% [0.00, 0.06]) | 0.022 [0.014, 0.030] | 37.8% | 21.5% | 22.2 | 2.1 | 4.8% | 5 |
| degraded | no manoeuvre | 20.8% [19.8, 21.9] | 66 (1.10% [0.87, 1.40]) | 1 | 0.0% | 0.0% | - | - | 0.0% | 0 |
| degraded | CPA thresholds + heading | 3.7% [3.3, 4.2] | 5 (0.08% [0.04, 0.19]) | 0.178 [0.157, 0.199] | 36.0% | 20.7% | 25.1 | 6.2 | 8.7% | 10 |
| degraded | CPA thresholds + vertical | 6.6% [6.0, 7.2] | 6 (0.10% [0.05, 0.22]) | 0.315 [0.289, 0.341] | 35.6% | 20.2% | 15.5 | 105.2 | 14.7% | 20 |
| degraded | CPA thresholds + combined | 3.6% [3.1, 4.1] | 5 (0.08% [0.04, 0.19]) | 0.171 [0.152, 0.193] | 36.0% | 20.7% | 27.8 | 9.9 | 8.5% | 12 |
| degraded | WCV + heading | 3.3% [2.9, 3.8] | 4 (0.07% [0.03, 0.17]) | 0.158 [0.139, 0.178] | 40.9% | 26.5% | 22.7 | 7.6 | 11.2% | 11 |
| degraded | WCV + vertical | 3.7% [3.2, 4.2] | 5 (0.08% [0.04, 0.19]) | 0.176 [0.156, 0.199] | 40.7% | 26.2% | 14.7 | 100.8 | 18.4% | 11 |
| degraded | WCV + combined | 3.2% [2.8, 3.7] | 4 (0.07% [0.03, 0.17]) | 0.153 [0.135, 0.174] | 40.9% | 26.5% | 26.1 | 9.0 | 11.0% | 14 |
| degraded | WCV + tau_mod 20 s + heading | 3.0% [2.6, 3.5] | 3 (0.05% [0.02, 0.15]) | 0.145 [0.127, 0.164] | 44.5% | 30.8% | 21.2 | 7.0 | 16.4% | 11 |
| degraded | WCV + tau_mod 20 s + vertical | 3.2% [2.8, 3.7] | 3 (0.05% [0.02, 0.15]) | 0.156 [0.136, 0.176] | 44.3% | 30.6% | 14.8 | 87.6 | 25.6% | 12 |
| degraded | WCV + tau_mod 20 s + combined | 2.9% [2.5, 3.4] | 3 (0.05% [0.02, 0.15]) | 0.141 [0.123, 0.161] | 44.5% | 30.8% | 25.3 | 7.4 | 16.0% | 13 |

Paired differences in loss-of-well-clear rate:

| surveillance | a - b | difference | discordant a-only / b-only | McNemar exact p |
|---|---|---|---|---|
| nominal | wcv/heading - cpa/heading | -0.73 pts [-0.97, -0.52] | 1 / 45 | 1.3e-12 |
| nominal | wcv/vertical - wcv/heading | +0.42 pts [+0.18, +0.65] | 37 / 12 | 0.00047 |
| nominal | wcv/combined - wcv/heading | -0.13 pts [-0.23, -0.03] | 1 / 9 | 0.021 |
| nominal | wcv_tau/heading - wcv/heading | -0.07 pts [-0.18, +0.05] | 4 / 8 | 0.39 |
| degraded | wcv/heading - cpa/heading | -0.42 pts [-0.65, -0.18] | 12 / 37 | 0.00047 |
| degraded | wcv/vertical - wcv/heading | +0.38 pts [+0.15, +0.62] | 39 / 16 | 0.0027 |
| degraded | wcv/combined - wcv/heading | -0.10 pts [-0.22, +0.02] | 4 / 10 | 0.18 |
| degraded | wcv_tau/heading - wcv/heading | -0.27 pts [-0.48, -0.07] | 13 / 29 | 0.02 |

### 5. Re-planning versus a one-shot manoeuvre

| surveillance | alerting + policy | LoWC one-shot | LoWC with re-planning | difference | discordant re-plan-only / one-shot-only | McNemar exact p |
|---|---|---|---|---|---|---|
| nominal | WCV + heading | 0.9% [0.7, 1.2] | 0.7% [0.5, 0.9] | -0.25 pts [-0.38, -0.13] | 0 / 15 | 6.1e-05 |
| nominal | WCV + vertical | 1.6% [1.3, 1.9] | 1.1% [0.9, 1.4] | -0.47 pts [-0.73, -0.18] | 20 / 48 | 0.00091 |
| nominal | WCV + tau_mod 20 s + heading | 1.4% [1.1, 1.7] | 0.6% [0.4, 0.8] | -0.80 pts [-1.03, -0.58] | 0 / 48 | 7.1e-15 |
| nominal | WCV + tau_mod 20 s + vertical | 1.8% [1.4, 2.1] | 0.5% [0.4, 0.8] | -1.22 pts [-1.53, -0.92] | 7 / 80 | 8.3e-17 |
| degraded | WCV + heading | 4.0% [3.5, 4.5] | 3.3% [2.9, 3.8] | -0.67 pts [-0.88, -0.47] | 1 / 41 | 2e-11 |
| degraded | WCV + vertical | 4.8% [4.3, 5.4] | 3.7% [3.2, 4.2] | -1.12 pts [-1.43, -0.80] | 15 / 82 | 2.4e-12 |
| degraded | WCV + tau_mod 20 s + heading | 4.7% [4.2, 5.3] | 3.0% [2.6, 3.5] | -1.68 pts [-2.02, -1.35] | 1 / 102 | 2.1e-29 |
| degraded | WCV + tau_mod 20 s + vertical | 5.3% [4.8, 5.9] | 3.2% [2.8, 3.7] | -2.08 pts [-2.45, -1.70] | 7 / 132 | 5.2e-31 |

### 6. Pilot/automation response delay (WCV alerting, nominal surveillance)

| policy | delay 0 s: LoWC | delay 5 s: LoWC | delay 10 s: LoWC | delay 15 s: LoWC | delay 20 s: LoWC |
|---|---|---|---|---|---|
| heading | 0.2% [0.2, 0.4] | 0.7% [0.5, 0.9] | 1.2% [1.0, 1.5] | 1.7% [1.4, 2.1] | 3.6% [3.2, 4.1] |
| vertical | 0.3% [0.2, 0.5] | 1.1% [0.9, 1.4] | 1.6% [1.3, 2.0] | 2.6% [2.2, 3.0] | 8.1% [7.4, 8.8] |

### 7. Where it fails

Missed-or-late alerts (open loop, nominal, filtered), counted by mechanism. Labels overlap.

| strategy | total | no_alert_before_loss | alert_but_late | tcpa_beyond_T_at_required_time | vertical_outside_Dv_at_cpa | intruder_turned | drone_intruder |
|---|---|---|---|---|---|---|---|
| CPA thresholds | 260 | 17 | 243 | 60 | 73 | 84 | 96 |
| WCV | 157 | 2 | 155 | 11 | 40 | 75 | 83 |
| WCV + tau_mod 20 s | 123 | 2 | 121 | 10 | 25 | 66 | 59 |

Missed-or-late rate by encounter class (open loop, nominal, filtered):

| class | CPA thresholds | WCV | WCV + tau_mod 20 s |
|---|---|---|---|
| geometry=head_on | 10.2% [7.4, 13.9] (n=334) | 9.3% [6.6, 12.9] (n=334) | 7.5% [5.1, 10.8] (n=334) |
| geometry=crossing | 12.3% [9.4, 15.9] (n=391) | 11.3% [8.5, 14.8] (n=391) | 8.7% [6.3, 11.9] (n=391) |
| geometry=overtaking | 43.0% [37.1, 49.1] (n=258) | 18.6% [14.3, 23.8] (n=258) | 15.5% [11.6, 20.4] (n=258) |
| geometry=vertical | 25.2% [20.3, 30.7] (n=266) | 12.8% [9.3, 17.3] (n=266) | 9.0% [6.1, 13.1] (n=266) |
| intruder=evtol | 20.5% [16.7, 24.9] (n=370) | 7.8% [5.5, 11.0] (n=370) | 6.5% [4.4, 9.5] (n=370) |
| intruder=drone | 20.3% [16.9, 24.1] (n=474) | 17.5% [14.4, 21.2] (n=474) | 12.4% [9.8, 15.7] (n=474) |
| intruder=ga | 21.7% [18.0, 26.0] (n=405) | 11.1% [8.4, 14.5] (n=405) | 9.9% [7.3, 13.2] (n=405) |
| intruder_turn=straight | 17.1% [14.9, 19.5] (n=1031) | 8.0% [6.5, 9.8] (n=1031) | 5.5% [4.3, 7.1] (n=1031) |
| intruder_turn=turning | 38.5% [32.3, 45.1] (n=218) | 34.4% [28.4, 40.9] (n=218) | 30.3% [24.6, 36.7] (n=218) |

Residual losses of well clear with avoidance, labelled (labels overlap; `results/fail_cases.csv` lists every case with its encounter index):

| surveillance | alerting + policy | n_lowc | no_or_late_alert | no_conflict_free_candidate | intruder_turned | floor_or_ceiling_limited | drone_intruder | unmitigated_not_hazardous | none_of_first_four |
|---|---|---|---|---|---|---|---|---|---|
| nominal | CPA thresholds + heading | 84 | 60 | 79 | 53 | 4 | 16 | 2 | 0 |
| nominal | CPA thresholds + vertical | 212 | 90 | 203 | 63 | 102 | 75 | 2 | 4 |
| nominal | CPA thresholds + combined | 74 | 58 | 68 | 46 | 7 | 14 | 3 | 1 |
| nominal | WCV + heading | 40 | 25 | 39 | 39 | 3 | 5 | 1 | 0 |
| nominal | WCV + vertical | 65 | 24 | 64 | 32 | 32 | 32 | 1 | 0 |
| nominal | WCV + combined | 32 | 25 | 31 | 31 | 2 | 4 | 2 | 0 |
| nominal | WCV + tau_mod 20 s + heading | 36 | 22 | 34 | 36 | 2 | 4 | 2 | 0 |
| nominal | WCV + tau_mod 20 s + vertical | 32 | 22 | 30 | 27 | 7 | 10 | 2 | 0 |
| nominal | WCV + tau_mod 20 s + combined | 27 | 22 | 26 | 27 | 1 | 3 | 5 | 0 |
| degraded | CPA thresholds + heading | 222 | 190 | 170 | 75 | 9 | 173 | 10 | 2 |
| degraded | CPA thresholds + vertical | 393 | 243 | 343 | 101 | 114 | 239 | 20 | 5 |
| degraded | CPA thresholds + combined | 214 | 188 | 166 | 73 | 11 | 168 | 12 | 0 |
| degraded | WCV + heading | 197 | 165 | 145 | 68 | 7 | 165 | 11 | 3 |
| degraded | WCV + vertical | 220 | 164 | 170 | 66 | 35 | 182 | 11 | 7 |
| degraded | WCV + combined | 191 | 163 | 139 | 65 | 8 | 163 | 14 | 2 |
| degraded | WCV + tau_mod 20 s + heading | 181 | 152 | 133 | 59 | 8 | 155 | 11 | 6 |
| degraded | WCV + tau_mod 20 s + vertical | 195 | 152 | 148 | 64 | 22 | 163 | 12 | 7 |
| degraded | WCV + tau_mod 20 s + combined | 176 | 152 | 127 | 54 | 8 | 155 | 13 | 6 |

### 8. Test-suite mutation check

`scripts/mutation_check.py` planted 25 deliberate bugs, one at a time, in fresh copies of the repository; the test suite failed on 25 of 25. The unmutated copy passed first (58 passed in 19.14s).

The first run caught 24 of 25; the survivor (C++ CPA time boundary < instead of <=) showed that no test hit an exact threshold boundary, so boundary cases (tcpa == T, hmd == Dh, |vmd| == Dv) were added to the Python and C++ tests before the run below.

| # | file | planted bug | result |
|---|---|---|---|
| 0 | `daa/geometry.py` | scalar tcpa sign flipped | caught |
| 1 | `daa/geometry.py` | vectorised CPA not clamped to t >= 0 | caught |
| 2 | `daa/geometry.py` | scalar circle exit uses wrong root | caught |
| 3 | `daa/geometry.py` | vectorised tau_mod coefficient sign | caught |
| 4 | `daa/geometry.py` | vectorised vertical interval bound | caught |
| 5 | `daa/geometry.py` | lookahead time ignored | caught |
| 6 | `daa/geometry.py` | CPA time threshold doubled (vectorised) | caught |
| 7 | `daa/geometry.py` | loss onset takes last segment | caught |
| 8 | `daa/alerting.py` | tier index lost | caught |
| 9 | `daa/alerting.py` | M-of-N off by one | caught |
| 10 | `daa/alerting.py` | hold time off by one | caught |
| 11 | `daa/metrics.py` | Wilson half-width term dropped | caught |
| 12 | `daa/metrics.py` | required lead boundary | caught |
| 13 | `daa/metrics.py` | alerts after onset counted | caught |
| 14 | `daa/sim.py` | turn rate limit doubled | caught |
| 15 | `daa/sim.py` | vertical acceleration unbounded | caught |
| 16 | `daa/sim.py` | left-turn preference | caught |
| 17 | `daa/sim.py` | altitude floor ignored | caught |
| 18 | `daa/sim.py` | response delay ignored | caught |
| 19 | `daa/sensors.py` | alpha-beta gain formula | caught |
| 20 | `daa/sensors.py` | dropout recovery inverted | caught |
| 21 | `daa/encounters.py` | encounter back-propagation sign | caught |
| 22 | `cpp/daa_core.hpp` | C++ circle exit perturbed by 1e-9 | caught |
| 23 | `cpp/daa_core.hpp` | C++ tier off by one | caught |
| 24 | `cpp/daa_core.hpp` | C++ CPA time boundary < instead of <= | caught |

![SOC curve](docs/soc_curve.png)

![Lead time histogram](docs/lead_time_hist.png)
<!-- END GENERATED RESULTS -->

## Findings

These statements refer to the tables above and hold for this synthetic setup only.

1. At the default thresholds, both WCV logics give fewer missed-or-late alerts than the CPA
   logic on the same encounters, and more nuisance alerts (tables 1 and 3). Sweeping the
   alert time, the WCV logics have the lower missed-or-late rate at every nuisance cap in the
   matched-nuisance table; scaling the distance thresholds down to a 10% nuisance cap, the
   CPA logic is not worse (intervals overlap). The WCV advantage here is mostly about alert
   timing, not a uniformly better trade-off.
2. The CPA logic fails mainly in overtaking and vertical-converging encounters (section 7):
   in slow overtakes the time to CPA stays above the alert time while well clear is about to
   be lost, and in vertical geometries the vertical separation at the horizontal CPA is not
   the separation at the time of loss.
3. For both WCV logics, encounters in which the intruder makes an unannounced turn have the
   highest missed-or-late rate of any class, several times the rate with straight-flying
   intruders; every logic here extrapolates straight lines.
4. The alert metric matters. Raw alerts flicker under noise and report loss; crediting the
   first alert makes every logic look much better than counting from the alert episode that
   is actually running when well clear is lost (table 2). The 2-of-4 raise / 5-cycle hold
   filter lowered both missed-or-late and nuisance rates for every logic in all three
   surveillance conditions.
5. Every avoidance policy cut loss of well clear to a small fraction of the no-manoeuvre
   rate. Vertical-only avoidance was worse than heading-only in both surveillance conditions
   (paired test), and in many of its residual failures the 150-600 m altitude band changed
   the planner's decision. Combined heading and altitude was at least as good as heading only.
6. With a one-shot manoeuvre, the earliest-alerting logic (WCV + tau_mod) had more residual
   losses than WCV in all four pairs shown in section 5; re-checking the manoeuvre every 5 s
   reduced residual losses in all eight configurations tested. Earlier alerts only paid off
   once the manoeuvre was re-checked against newer tracks.
7. Residual losses grow with response delay, most steeply between 15 and 20 s, and vertical
   avoidance is more sensitive to delay than heading avoidance (section 6).
8. Under degraded surveillance most residual losses involve non-cooperative drones, which
   have the noisiest tracks.

## Limitations

- Synthetic encounters. The mix of geometries, miss distances and intruder types is chosen,
  not measured, and every rate above depends on it. The rates are useful for comparing
  logics on the same encounters, not as estimates of real-world risk.
- Rejection sampling (intruder altitude limits) removes many steep vertical encounters, so
  the vertical class is the smallest and has fewer climbing than descending intruders.
- Loss of well clear without avoidance defines "hazardous"; an encounter that stays just
  outside the volume counts as non-hazardous, so a well-timed alert on it counts as nuisance.
- Performance, noise, gust and dropout values are assumptions (see table). No aircraft or
  sensor data was used.
- Point-mass kinematics with first-order limits; no aerodynamics, no flight control system,
  no transition-phase behaviour of lift+cruise eVTOLs.
- Pairwise encounters only. Terrain is represented only by an altitude floor; there are no
  weather cells, buildings or multi-intruder conflicts.
- Intruders never coordinate or respond. Real cooperative traffic may manoeuvre too.
- Alert filtering and the planner are my simplifications. They are inspired by DAIDALUS
  but are not DAIDALUS, and DO-365 compliance was not assessed.
- Human factors are only represented by a fixed response delay and by lead-time and alert
  flicker metrics; no pilot-in-the-loop study was done.

## Reproduce

```
pip install -r requirements.txt
python -m pytest -q                      # needs g++, clang++ or MSVC for the C++ agreement tests
python scripts/run_eval.py               # writes results/ and docs/
python scripts/make_readme.py            # refresh the results section
python scripts/make_readme.py --check    # fails if README and results disagree
python scripts/mutation_check.py         # optional, about 10 minutes
```

The `.github/workflows/ci.yml` workflow builds the C++ with g++ on Ubuntu, runs the tests with
`DAA_REQUIRE_CPP=1` (so the agreement tests cannot be skipped) and runs the README check.
