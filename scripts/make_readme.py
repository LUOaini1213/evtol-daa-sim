"""Render the results section of README.md from results/summary.json.

    python scripts/make_readme.py          # rewrite the generated block in README.md
    python scripts/make_readme.py --check  # exit 1 if README.md differs from the results
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BEGIN = "<!-- BEGIN GENERATED RESULTS (scripts/make_readme.py) -->"
END = "<!-- END GENERATED RESULTS -->"
NAMES = {"cpa": "CPA thresholds", "wcv": "WCV", "wcv_tau": "WCV + tau_mod 20 s"}


def pct(r):
    return f"{100 * r['p']:.1f}% [{100 * r['lo']:.1f}, {100 * r['hi']:.1f}]"


def pct2(r):
    return f"{r['k']} ({100 * r['p']:.2f}% [{100 * r['lo']:.2f}, {100 * r['hi']:.2f}])"


def cnt(r):
    return f"{r['k']}/{r['n']}"


def num(x, fmt="{:.1f}"):
    return "-" if x is None else fmt.format(x)


def table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def render_mutation(mut):
    L = ["### 8. Historical test-suite mutation check", ""]
    L.append(f"`scripts/mutation_check.py` planted {mut['total']} deliberate bugs, one at a time, in fresh "
             f"copies of the repository; tests detected {mut['caught']} of {mut['total']}. "
             f"The unmutated copy passed first ({mut['baseline']['summary']}).")
    if mut.get("cpp_required"):
        L.append("Every case must compile the C++ core before pytest. Collection, setup, compiler, "
                 "timeout and skipped-test outcomes invalidate the run rather than count as detections. "
                 "The report retains per-case failure evidence and full logs/JUnit files.")
    L.append("")
    first = mut.get("_first_run")
    if first:
        surv = [m["description"] for m in first["mutants"] if not m["caught"]]
        L.append(f"The archived initial implementation's first run caught {first['caught']} of {first['total']}; "
                 f"the survivor ({'; '.join(surv) or 'none'}) led to exact-threshold tests. "
                 "That file is historical evidence, not a rerun of the current source or current mutation harness.")
        L.append("")
    rows = [[m["index"], f"`{m['file']}`", m["description"], "caught" if m["caught"] else "SURVIVED"]
            for m in mut["mutants"]]
    L.append(table(["#", "file", "planted bug", "result"], rows))
    return "\n".join(L)


def render(res):
    m = res["meta"]
    L = []
    L.append("These are preserved historical evaluations of [source commit "
             "`0e8566360e02e7d2c5afb45018578ea80c05f13a`](https://github.com/LUOaini1213/evtol-daa-sim/tree/0e8566360e02e7d2c5afb45018578ea80c05f13a). "
             "They do not evaluate the current altitude-constrained controller. No new 6,000-encounter evaluation "
             "or full mutation gate is claimed for that controller; see [the local regression scope](docs/ALTITUDE_CONSTRAINTS.md).")
    L.append("")
    L.append(f"Historical numbers below were produced by that version of `scripts/run_eval.py` (seed {m['seed']}, "
             f"{m['n']} encounters) and rendered by `scripts/make_readme.py`. Brackets are 95% "
             "Wilson intervals for rates and 95% paired-bootstrap intervals for differences and ratios.")
    L.append("")
    gc = m["geometry_counts"]
    ic = m["intruder_counts"]
    L.append(f"Encounter set: {m['n']} accepted of {m['generation']['tried']} sampled "
             f"(rejected: {m['generation']['rejected']['slow_closure']} slow closure, "
             f"{m['generation']['rejected']['early_or_initial_loss']} loss too early, "
             f"{m['generation']['rejected']['intruder_altitude']} intruder altitude out of range). "
             f"Geometry: head-on {gc['head_on']}, crossing {gc['crossing']}, overtaking {gc['overtaking']}, "
             f"vertical {gc['vertical']}. Intruder: eVTOL {ic['evtol']}, drone {ic['drone']}, GA {ic['ga']}; "
             f"{m['intruder_turn']} intruders make an unannounced turn. Without avoidance, "
             f"{cnt(m['hazardous'])} = {pct(m['hazardous'])} of encounters lose well clear and "
             f"{cnt(m['nmac_unmitigated'])} = {pct(m['nmac_unmitigated'])} reach NMAC. "
             f"Surveillance report loss: nominal {100 * m['report_loss_fraction']['nominal']:.1f}%, "
             f"degraded {100 * m['report_loss_fraction']['degraded']:.1f}%. "
             f"Simulation and analysis time, excluding exports/plots: {m['runtime_s']:.0f} s "
             f"(Python {m['python']}, numpy {m['numpy']}).")
    L.append("")

    ol = {(r["condition"], r["strategy"], r["filter"]): r for r in res["open_loop"]}
    L.append("### 1. Alerting performance (open loop, no avoidance), nominal surveillance, filtered alerts")
    L.append("")
    rows = []
    for st in NAMES:
        r = ol[("nominal", st, "filtered")]
        rows.append([NAMES[st], pct(r["missed_or_late"]), pct(r["nuisance"]), num(r["lead"]["median"]),
                     num(r["lead"]["p10"]), f"{r['episodes_mean']:.2f}"])
    L.append(table(["strategy", "missed or late (lead < 25 s)", "nuisance", "median lead (s)",
                    "10th pct lead (s)", "alert episodes per hazardous enc."], rows))
    L.append("")
    L.append("Threshold sweeps at matched nuisance: lowest missed-or-late rate among sweep points whose "
             "nuisance rate is at most the given cap (figure below shows all points).")
    L.append("")
    rows = []
    for cap in (0.10, 0.15, 0.20):
        for kind, label in (("time", "alert time T"), ("scale", "threshold scale")):
            row = [f"<= {100 * cap:.0f}%", label]
            for st in NAMES:
                pts = [p for p in res["sweep"] if p["kind"] == kind and p["strategy"] == st and p["nuisance"]["p"] <= cap]
                if pts:
                    b = min(pts, key=lambda p: (p["missed_or_late"]["p"], p["nuisance"]["p"]))
                    unit = " s" if kind == "time" else "x"
                    row.append(f"{pct(b['missed_or_late'])} at {b['x']:g}{unit}")
                else:
                    row.append("-")
            rows.append(row)
    L.append(table(["nuisance cap", "swept parameter"] + [NAMES[s] for s in NAMES], rows))
    L.append("")
    L.append("### 2. Surveillance quality and alert filtering")
    L.append("")
    rows = []
    for c in ("ideal", "nominal", "degraded"):
        for st in NAMES:
            a, b = ol[(c, st, "raw")], ol[(c, st, "filtered")]
            rows.append([c, NAMES[st], pct(a["missed_or_late"]), pct(b["missed_or_late"]),
                         pct(a["nuisance"]), pct(b["nuisance"]), pct(b["missed_first_alert"])])
    L.append(table(["surveillance", "strategy", "missed/late raw", "missed/late filtered", "nuisance raw",
                    "nuisance filtered", "missed/late if first alert counted"], rows))
    L.append("")
    L.append("### 3. Paired comparisons on the same encounters (filtered alerts)")
    L.append("")
    rows = []
    for p in res["open_loop_paired"]:
        d = p["diff"]
        rows.append([p["condition"], f"{NAMES[p['a']]} - {NAMES[p['b']]}", p["metric"], p["n"],
                     f"{100 * d['diff']:+.1f} pts [{100 * d['lo']:+.1f}, {100 * d['hi']:+.1f}]",
                     f"{p['mcnemar']['b']} / {p['mcnemar']['c']}", f"{p['mcnemar']['p']:.2g}"])
    L.append(table(["surveillance", "a - b", "metric", "n", "difference", "discordant a-only / b-only",
                    "McNemar exact p"], rows))
    L.append("")
    L.append("### 4. Closed loop: avoidance manoeuvres")
    L.append("")
    rows = []
    for r in res["closed_loop"]:
        name = "no manoeuvre" if r["policy"] == "none" else f"{NAMES[r['strategy']]} + {r['policy']}"
        rr = r["lowc_rr"]
        rows.append([r["condition"], name, pct(r["lowc"]), pct2(r["nmac"]),
                     "1" if r["policy"] == "none" else f"{rr['ratio']:.3f} [{rr['lo']:.3f}, {rr['hi']:.3f}]",
                     f"{100 * r['manoeuvre']['p']:.1f}%", f"{100 * r['nuisance_manoeuvre']['p']:.1f}%",
                     num(r["mean_dpsi_deg"]), num(r["mean_dz_m"]), f"{100 * r['emergency']['p']:.1f}%",
                     r["induced_lowc"]])
    L.append(table(["surveillance", "alerting + policy", "loss of well clear", "NMAC count (rate)", "LoWC risk ratio",
                    "manoeuvred", "manoeuvred when not needed", "mean max heading change (deg)",
                    "mean max altitude change (m)", "emergency escalation", "induced LoWC"], rows))
    L.append("")
    rows = []
    for p in res["closed_loop_paired"]:
        d = p["diff"]
        rows.append([p["condition"], f"{p['a']} - {p['b']}", f"{100 * d['diff']:+.2f} pts [{100 * d['lo']:+.2f}, {100 * d['hi']:+.2f}]",
                     f"{p['mcnemar']['b']} / {p['mcnemar']['c']}", f"{p['mcnemar']['p']:.2g}"])
    L.append("Paired differences in loss-of-well-clear rate:")
    L.append("")
    L.append(table(["surveillance", "a - b", "difference", "discordant a-only / b-only", "McNemar exact p"], rows))
    L.append("")
    L.append("### 5. Re-planning versus a one-shot manoeuvre")
    L.append("")
    rows = []
    for a in res["replan_ablation"]:
        d = a["diff"]
        rows.append([a["condition"], f"{NAMES[a['strategy']]} + {a['policy']}", pct(a["one_shot"]), pct(a["replan"]),
                     f"{100 * d['diff']:+.2f} pts [{100 * d['lo']:+.2f}, {100 * d['hi']:+.2f}]",
                     f"{a['mcnemar']['b']} / {a['mcnemar']['c']}", f"{a['mcnemar']['p']:.2g}"])
    L.append(table(["surveillance", "alerting + policy", "LoWC one-shot", "LoWC with re-planning",
                    "difference", "discordant re-plan-only / one-shot-only", "McNemar exact p"], rows))
    L.append("")
    L.append("### 6. Pilot/automation response delay (WCV alerting, nominal surveillance)")
    L.append("")
    delays = sorted({d["delay"] for d in res["delay_sensitivity"]})
    rows = []
    for pol in ("heading", "vertical"):
        rows.append([pol] + [pct(d["lowc"]) for dl in delays for d in res["delay_sensitivity"]
                             if d["policy"] == pol and d["delay"] == dl])
    L.append(table(["policy"] + [f"delay {d:.0f} s: LoWC" for d in delays], rows))
    L.append("")
    L.append("### 7. Where it fails")
    L.append("")
    L.append("Missed-or-late alerts (open loop, nominal, filtered), counted by mechanism. Labels overlap.")
    L.append("")
    mech = res["open_loop_mechanisms"]
    keys = ["total", "no_alert_before_loss", "alert_but_late", "tcpa_beyond_T_at_required_time",
            "vertical_outside_Dv_at_cpa", "intruder_turned", "drone_intruder"]
    L.append(table(["strategy"] + keys, [[NAMES[s]] + [mech[s][k] for k in keys] for s in NAMES]))
    L.append("")
    L.append("Missed-or-late rate by encounter class (open loop, nominal, filtered):")
    L.append("")
    rows = []
    br = res["open_loop_breakdown"]
    labels = [(b["by"], b["value"]) for b in br if b["strategy"] == "cpa"]
    for by, val in labels:
        row = [f"{by}={val}"]
        for st in NAMES:
            b = next(x for x in br if x["strategy"] == st and x["by"] == by and x["value"] == val)
            row.append(f"{pct(b['missed_or_late'])} (n={b['missed_or_late']['n']})")
        rows.append(row)
    L.append(table(["class"] + [NAMES[s] for s in NAMES], rows))
    L.append("")
    L.append("Residual losses of well clear with avoidance, labelled (labels overlap; "
             "`results/fail_cases.csv` lists every case with its encounter index):")
    L.append("")
    fk = ["n_lowc", "no_or_late_alert", "no_conflict_free_candidate", "intruder_turned",
          "floor_or_ceiling_limited", "drone_intruder", "unmitigated_not_hazardous", "none_of_first_four"]
    rows = [[f["condition"], f"{NAMES[f['strategy']]} + {f['policy']}"] + [f[k] for k in fk]
            for f in res["closed_loop_failures"]]
    L.append(table(["surveillance", "alerting + policy"] + fk, rows))
    L.append("")
    if res.get("_mutation"):
        L.append(render_mutation(res["_mutation"]))
        L.append("")
    L.append("![SOC curve](docs/soc_curve.png)")
    L.append("")
    L.append("![Lead time histogram](docs/lead_time_hist.png)")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--results", default=str(ROOT / "results" / "summary.json"))
    ap.add_argument("--readme", default=str(ROOT / "README.md"))
    args = ap.parse_args(argv)
    res = json.loads(Path(args.results).read_text(encoding="utf-8"))
    mut = Path(args.results).with_name("mutation_report.json")
    if mut.exists():
        res["_mutation"] = json.loads(mut.read_text(encoding="utf-8"))
        first = Path(args.results).with_name("mutation_report_first_run.json")
        if first.exists():
            res["_mutation"]["_first_run"] = json.loads(first.read_text(encoding="utf-8"))
    readme = Path(args.readme).read_text(encoding="utf-8")
    if BEGIN not in readme or END not in readme:
        print("README markers missing", file=sys.stderr)
        return 1
    head, rest = readme.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    new = head + BEGIN + "\n" + render(res) + "\n" + END + tail
    if args.check:
        if new != readme:
            print("README.md results are out of date with results/summary.json "
                  "(run: python scripts/make_readme.py)", file=sys.stderr)
            return 1
        print("README.md matches results/summary.json")
        return 0
    Path(args.readme).write_text(new, encoding="utf-8", newline="\n")
    print("README.md updated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
