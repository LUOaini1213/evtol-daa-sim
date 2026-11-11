"""Render the summary and results sections of README.md from results/.

    python scripts/make_readme.py          # rewrite the generated blocks in README.md
    python scripts/make_readme.py --check  # exit 1 if README.md, the results files and the
                                           # current source disagree (see check_provenance)

--check also re-tests every claim in the hand-written Findings (check_findings), so a
re-run that changes an ordering fails CI instead of leaving stale prose.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BEGIN = "<!-- BEGIN GENERATED RESULTS (scripts/make_readme.py) -->"
END = "<!-- END GENERATED RESULTS -->"
TLDR_BEGIN = "<!-- BEGIN GENERATED SUMMARY (scripts/make_readme.py) -->"
TLDR_END = "<!-- END GENERATED SUMMARY -->"
HISTORICAL_SUMMARY = "docs/historical/0e85663/summary.json"
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
    L = ["### 8. Test-suite mutation check", ""]
    L.append(f"`scripts/mutation_check.py` planted {mut['total']} deliberate bugs, one at a time, in fresh "
             f"copies of the repository; tests detected {mut['caught']} of {mut['total']}. "
             f"The unmutated copy passed first ({mut['baseline']['summary']}).")
    if mut.get("cpp_required"):
        L.append("Every case must compile the C++ core before pytest. Collection, setup, compiler, "
                 "timeout and skipped-test outcomes invalidate the run rather than count as detections. "
                 "The report retains per-case failure evidence; the logs and JUnit files are in "
                 "[`results/mutation_evidence.zip`](results/mutation_evidence.zip), with local paths replaced "
                 "by placeholders.")
    L.append("")
    rows = [[m["index"], f"`{m['file']}`", m["description"], "caught" if m["caught"] else "SURVIVED"]
            for m in mut["mutants"]]
    L.append(table(["#", "file", "planted bug", "result"], rows))
    return "\n".join(L)


def render(res):
    m = res["meta"]
    L = []
    L.append(f"Numbers below were produced by `scripts/run_eval.py` on the current code (seed {m['seed']}, "
             f"{m['n']} encounters) and rendered by `scripts/make_readme.py`. `results/summary.json` records the "
             "SHA-256 of every source file the run used, and `make_readme.py --check` (run in CI) fails if any of "
             "them has changed since. Brackets are 95% Wilson intervals for rates and 95% paired-bootstrap "
             "intervals for differences and ratios. Results of the earlier controller (source `0e85663`) are kept "
             "in [docs/historical/](docs/historical/README.md).")
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


def _rate(res, cond, strategy, metric):
    return next(r for r in res["open_loop"] if r["condition"] == cond and r["strategy"] == strategy
                and r["filter"] == "filtered")[metric]


def _closed(res, cond, strategy, policy):
    return next(r for r in res["closed_loop"] if r["condition"] == cond and r["policy"] == policy
                and (policy == "none" or r["strategy"] == strategy))


def _paired(res, cond, a, b, metric):
    return next(p for p in res["open_loop_paired"] if p["condition"] == cond and p["a"] == a
                and p["b"] == b and p["metric"] == metric)


def _class(res, strategy, by, value):
    return next(b for b in res["open_loop_breakdown"] if b["strategy"] == strategy and b["by"] == by
                and b["value"] == value)["missed_or_late"]


def render_tldr(res):
    m = res["meta"]
    ml = _paired(res, "nominal", "wcv", "cpa", "missed_or_late")["diff"]
    nu = _paired(res, "nominal", "wcv", "cpa", "nuisance")["diff"]
    none = _closed(res, "nominal", None, "none")["lowc"]
    best = min((r for r in res["closed_loop"] if r["condition"] == "nominal" and r["policy"] != "none"),
               key=lambda r: r["lowc"]["p"])
    turn, straight = (_class(res, "wcv", "intruder_turn", v) for v in ("turning", "straight"))
    delay = {(d["policy"], d["delay"]): d["lowc"] for d in res["delay_sensitivity"]}
    mut = res.get("_mutation")
    L = [f"**Built:** a Monte-Carlo detect-and-avoid simulator for an eVTOL: {m['n']} synthetic encounters, "
         "three alerting logics (CPA thresholds, DAIDALUS-style well-clear, well-clear + modified tau) x four "
         "avoidance policies, paired McNemar/bootstrap statistics, and a C++17 alerting core that must match "
         "the Python output exactly."
         + (f" A mutation gate plants {mut['total']} bugs; the tests catch {mut['caught']}." if mut else ""),
         "",
         "**Found** (nominal surveillance, filtered alerts, this synthetic setup only):",
         "",
         f"- Well-clear (WCV) alerting cuts missed-or-late alerts by {-100 * ml['diff']:.1f} points "
         f"[{-100 * ml['hi']:.1f}, {-100 * ml['lo']:.1f}] versus CPA thresholds, at the cost of "
         f"{100 * nu['diff']:+.1f} points of nuisance alerts.",
         f"- With avoidance, loss of well clear falls from {100 * none['p']:.1f}% (no manoeuvre) to "
         f"{100 * best['lowc']['p']:.1f}% at best ({NAMES[best['strategy']]} + {best['policy']}).",
         f"- Unannounced intruder turns are the main failure mode: WCV misses or is late on "
         f"{100 * turn['p']:.1f}% of hazardous turning encounters versus {100 * straight['p']:.1f}% of straight ones.",
         f"- Response delay matters: WCV + heading loses well clear in {100 * delay[('heading', 5.0)]['p']:.1f}% "
         f"of encounters at 5 s and {100 * delay[('heading', 20.0)]['p']:.1f}% at 20 s.",
         "",
         "**Limits:** encounters are synthetic and not fitted to traffic data; point-mass kinematics; "
         "single intruder that never manoeuvres in response; gusts are not bounded; this is not DAIDALUS and "
         "DO-365 compliance was not assessed."]
    return "\n".join(L)


def cited_numbers(res):
    """The numbers the hand-written Findings quote, formatted exactly as they appear."""
    def best_scale(st):
        pts = [p for p in res["sweep"] if p["kind"] == "scale" and p["strategy"] == st and p["nuisance"]["p"] <= 0.10]
        return f"{100 * min(p['missed_or_late']['p'] for p in pts):.1f}%"

    def lowc(c, st, pol):
        return str(_closed(res, c, st, pol)["lowc"]["k"])

    def cl_p(c, a, b):
        return f"p={next(x for x in res['closed_loop_paired'] if x['condition'] == c and x['a'] == a and x['b'] == b)['mcnemar']['p']:.3f}"

    abl = next(a for a in res["replan_ablation"] if a["condition"] == "degraded" and a["strategy"] == "wcv"
               and a["policy"] == "vertical")
    geo = {v: _class(res, "cpa", "geometry", v) for v in ("head_on", "crossing", "overtaking", "vertical")}
    old = json.loads((ROOT / HISTORICAL_SUMMARY).read_text(encoding="utf-8"))
    return {
        "F1 CPA scale point": best_scale("cpa"), "F1 WCV scale point": best_scale("wcv"),
        "F1 WCV+tau scale point": best_scale("wcv_tau"),
        "F2 overtaking rate": f"{100 * geo['overtaking']['p']:.1f}%", "F2 vertical rate": f"{100 * geo['vertical']['p']:.1f}%",
        "F2 vertical count": f"contributes {geo['vertical']['k']} cases",
        "F2 head-on/crossing counts": f"versus {geo['head_on']['k']} head-on and {geo['crossing']['k']} crossing",
        "F5 nominal counts": f"{lowc('nominal', 'wcv', 'vertical')} versus {lowc('nominal', 'wcv', 'heading')} under nominal",
        "F5 degraded counts": f"{lowc('degraded', 'wcv', 'vertical')} versus {lowc('degraded', 'wcv', 'heading')} under degraded",
        "F5 p values": f"McNemar {cl_p('nominal', 'wcv/vertical', 'wcv/heading')} and "
                       f"{cl_p('degraded', 'wcv/vertical', 'wcv/heading')[2:]}",
        "F6 counts": f"from {abl['one_shot']['k']} to {abl['replan']['k']}, with p={abl['mcnemar']['p']:.3f}",
        "F9 changed loss counts": " and ".join(
            f"{NAMES[x['strategy']]} + {x['policy']} ({x['lowc']['k']} to {y['lowc']['k']})"
            for x, y in zip(old["closed_loop"], res["closed_loop"]) if x["lowc"]["k"] != y["lowc"]["k"]),
        "F9 open loop identical": "open-loop alerting tables are identical" if all(
            old[k] == res[k] for k in ("open_loop", "open_loop_paired", "sweep")) else "(open loop changed)",
    }


def check_findings(res, readme=None):
    """Each hand-written Finding in README.md, as a test on results/summary.json."""
    problems = []
    if readme is not None:
        section = readme.split("## Findings", 1)[-1].split("\n## ", 1)[0]
        flat = " ".join(section.split())
        for label, text in cited_numbers(res).items():
            if text not in flat:
                problems.append(f"Findings must quote {label} as '{text}'")

    def claim(ok, text):
        if not ok:
            problems.append(text)

    S = list(NAMES)
    for c in ("nominal", "degraded"):
        cpa_ml = _rate(res, c, "cpa", "missed_or_late")["p"]
        cpa_nu = _rate(res, c, "cpa", "nuisance")["p"]
        for st in ("wcv", "wcv_tau"):
            claim(_rate(res, c, st, "missed_or_late")["p"] < cpa_ml, f"F1 {c}: {st} missed-or-late below CPA")
            claim(_rate(res, c, st, "nuisance")["p"] > cpa_nu, f"F1 {c}: {st} nuisance above CPA")
    for cap in (0.10, 0.15, 0.20):
        best = {}
        for st in S:
            pts = [p for p in res["sweep"] if p["kind"] == "time" and p["strategy"] == st and p["nuisance"]["p"] <= cap]
            best[st] = min(p["missed_or_late"]["p"] for p in pts) if pts else 1.0
        claim(best["wcv"] < best["cpa"] and best["wcv_tau"] < best["cpa"],
              f"F1 time sweep cap {cap}: WCV logics below CPA")
    scale10 = {}
    for st in S:
        pts = [p for p in res["sweep"] if p["kind"] == "scale" and p["strategy"] == st and p["nuisance"]["p"] <= 0.10]
        scale10[st] = min(p["missed_or_late"]["p"] for p in pts) if pts else 1.0
    claim(scale10["cpa"] < min(scale10["wcv"], scale10["wcv_tau"]), "F1 scale sweep 10% cap: CPA lowest")
    geo = {v: _class(res, "cpa", "geometry", v) for v in ("head_on", "crossing", "overtaking", "vertical")}
    claim(max(geo, key=lambda v: geo[v]["p"]) == "overtaking", "F2: CPA worst class is overtaking")
    for st in ("wcv", "wcv_tau"):
        rates = [b["missed_or_late"]["p"] for b in res["open_loop_breakdown"] if b["strategy"] == st]
        turn = _class(res, st, "intruder_turn", "turning")["p"]
        claim(turn == max(rates), f"F3: turning intruders are the worst class for {st}")
        claim(turn >= 2 * _class(res, st, "intruder_turn", "straight")["p"], f"F3: {st} turning >= 2x straight")
    for c in ("ideal", "nominal", "degraded"):
        for st in S:
            raw = next(r for r in res["open_loop"] if r["condition"] == c and r["strategy"] == st and r["filter"] == "raw")
            filt = next(r for r in res["open_loop"] if r["condition"] == c and r["strategy"] == st and r["filter"] == "filtered")
            claim(filt["missed_or_late"]["p"] < raw["missed_or_late"]["p"] and filt["nuisance"]["p"] < raw["nuisance"]["p"],
                  f"F4 {c} {st}: filter lowers both rates")
            claim(filt["missed_first_alert"]["p"] < filt["missed_or_late"]["p"], f"F4 {c} {st}: first-alert credit lower")
    for c in ("nominal", "degraded"):
        base = _closed(res, c, None, "none")["lowc"]["p"]
        for r in res["closed_loop"]:
            if r["condition"] == c and r["policy"] != "none":
                claim(r["lowc"]["p"] < base, f"F5 {c} {r['strategy']}/{r['policy']}: below no-manoeuvre")
        claim(_closed(res, c, "wcv", "combined")["lowc"]["k"] < _closed(res, c, "wcv", "heading")["lowc"]["k"],
              f"F5 {c}: WCV combined below WCV heading")
    for a in res["replan_ablation"]:
        claim(a["replan"]["k"] <= a["one_shot"]["k"], f"F6 {a['condition']} {a['strategy']}/{a['policy']}: re-plan not worse")
    for pol in ("heading", "vertical"):
        d = sorted((x["delay"], x["lowc"]["p"]) for x in res["delay_sensitivity"] if x["policy"] == pol)
        claim(all(b[1] >= a[1] for a, b in zip(d, d[1:])), f"F7 {pol}: LoWC non-decreasing in delay")
        steps = [b[1] - a[1] for a, b in zip(d, d[1:])]
        claim(steps[-1] == max(steps), f"F7 {pol}: steepest step is the last one")
    v20 = next(x for x in res["delay_sensitivity"] if x["policy"] == "vertical" and x["delay"] == 20.0)["lowc"]["p"]
    h20 = next(x for x in res["delay_sensitivity"] if x["policy"] == "heading" and x["delay"] == 20.0)["lowc"]["p"]
    claim(v20 > h20, "F7: vertical worse than heading at 20 s")
    for f in res["closed_loop_failures"]:
        if f["condition"] == "degraded":
            claim(2 * f["drone_intruder"] > f["n_lowc"], f"F8 {f['strategy']}/{f['policy']}: drones are the majority")
    return problems


def _load(name):
    spec = importlib.util.spec_from_file_location(name + "_for_readme", ROOT / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_provenance(res, results_dir):
    """Results must describe the current source: same file hashes, same evidence bytes."""
    problems = []
    run_eval = _load("run_eval")
    if res["meta"]["runtime_source_sha256"] != run_eval.runtime_source_sha256():
        problems.append("results/summary.json was produced by different source files than the current "
                        "checkout (rerun: python scripts/run_eval.py)")
    mut = res.get("_mutation")
    if mut is None:
        return problems + ["results/mutation_report.json is missing"]
    if mut.get("status") != "complete" or not mut.get("complete_full_matrix"):
        problems.append("mutation report is not a complete full run")
    if mut.get("source_test_config_sha256") != _load("mutation_check").source_hashes(ROOT):
        problems.append("results/mutation_report.json was produced by different sources/tests than the "
                        "current checkout (rerun: python scripts/mutation_check.py)")
    archive = Path(results_dir) / mut.get("evidence_archive", "")
    if not archive.is_file() or hashlib.sha256(archive.read_bytes()).hexdigest() != mut.get("evidence_archive_sha256"):
        problems.append("mutation evidence archive missing or changed")
    else:
        with zipfile.ZipFile(archive) as zipped:
            got = {n: hashlib.sha256(zipped.read(n)).hexdigest() for n in zipped.namelist()}
        if got != mut.get("evidence_member_sha256"):
            problems.append("mutation evidence members differ from the report")
    return problems


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
    readme = Path(args.readme).read_text(encoding="utf-8")
    new = readme
    for begin, end, body in ((TLDR_BEGIN, TLDR_END, render_tldr(res)), (BEGIN, END, render(res))):
        if begin not in new or end not in new:
            print("README markers missing: " + begin, file=sys.stderr)
            return 1
        head, rest = new.split(begin, 1)
        _, tail = rest.split(end, 1)
        new = head + begin + "\n" + body + "\n" + end + tail
    if args.check:
        problems = check_findings(res, new) + check_provenance(res, Path(args.results).parent)
        if new != readme:
            problems.insert(0, "README.md generated blocks are out of date with results/ "
                               "(run: python scripts/make_readme.py)")
        if problems:
            for problem in problems:
                print("FAIL " + problem, file=sys.stderr)
            return 1
        print("README.md matches results/; Findings hold; results match the current source")
        return 0
    Path(args.readme).write_text(new, encoding="utf-8", newline="\n")
    print("README.md updated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
