"""Full Monte-Carlo evaluation. Writes results/*.json|csv and docs/*.png.

    python scripts/run_eval.py            # full run (config n_encounters)
    python scripts/run_eval.py --n 500    # quick run (does not overwrite results/ unless --out given)
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from daa import load_config  # noqa: E402
from daa import alerting as al  # noqa: E402
from daa import encounters as E  # noqa: E402
from daa import geometry as g  # noqa: E402
from daa import metrics as M  # noqa: E402
from daa import sensors as S  # noqa: E402
from daa import sim  # noqa: E402

CONDITIONS = ("ideal", "nominal", "degraded")


def runtime_source_sha256():
    """Deterministic relative-path map of the exact source/config bytes used by a run."""
    paths = [*(ROOT / "daa").glob("*.py"), ROOT / "config/default.json", ROOT / "scripts/run_eval.py"]
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths, key=lambda p: p.relative_to(ROOT).as_posix())}


def verify_runtime_sources(expected):
    if runtime_source_sha256() != expected:
        raise RuntimeError("runtime sources changed during evaluation; no results published")


def _f(x):
    return None if x is None or (isinstance(x, float) and not np.isfinite(x)) else float(x)


def filt_cfg(cfg):
    p = cfg["alerting"]["persistence_m_of_n"]
    return dict(M=p["M"], N=p["N"], H=5)


def single_tier(cfg, strategy, T=None, scale=1.0):
    """Corrective tier only (for sweeps)."""
    t = al.tier_params(cfg, strategy)[1].copy()
    if T is not None:
        t["T"] = T
    t["D"] *= scale
    t["Z"] *= scale
    return [t]


def lead_stats(lead):
    x = lead[np.isfinite(lead)]
    if x.size == 0:
        return dict(median=None, p10=None, p90=None, n=0)
    return dict(median=float(np.median(x)), p10=float(np.quantile(x, 0.1)),
                p90=float(np.quantile(x, 0.9)), n=int(x.size))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=None)
    ap.add_argument("--out", default=None, help="results dir (default results/, or a scratch dir when --n given)")
    ap.add_argument("--docs", default=None)
    args = ap.parse_args(argv)

    t_start = time.time()
    source_hashes = runtime_source_sha256()
    cfg = load_config()
    try:
        n = E.validate_encounter_count(cfg["encounter"]["n_encounters"] if args.n is None else args.n)
    except ValueError as exc:
        ap.error(str(exc))
    out = Path(args.out) if args.out else (ROOT / "results" if args.n is None else ROOT / "build" / "quick")
    docs = Path(args.docs) if args.docs else (ROOT / "docs" if args.out is None and args.n is None else out)
    out.mkdir(parents=True, exist_ok=True)
    docs.mkdir(parents=True, exist_ok=True)
    seed = cfg["encounter"]["seed"]
    t_req = cfg["alerting"]["required_lead_time"]["value"]
    nb = cfg["stats"]["bootstrap_resamples"]
    FILT = filt_cfg(cfg)

    enc = E.generate(cfg, n=n, seed=seed)
    dt = enc.dt
    haz, onset, nmac0 = E.truth_outcome(enc.own_pos, enc.int_pos, dt, cfg)
    tracks = {c: S.make_track(enc, cfg, c, seed + 1 + i) for i, c in enumerate(CONDITIONS)}
    res = dict(meta=dict(n=n, seed=seed, dt=dt, horizon=float(enc.t[-1]), generation=enc.stats,
                         runtime_source_sha256=source_hashes,
                         hazardous=M.rate(haz), nmac_unmitigated=M.rate(nmac0),
                         geometry_counts={gname: int((enc.geometry == i).sum()) for i, gname in enumerate(E.GEOMETRIES)},
                         intruder_counts={x: int((enc.intruder == i).sum()) for i, x in enumerate(E.INTRUDERS)},
                         intruder_turn=int(enc.params["intruder_turn"].sum()),
                         filter=FILT, required_lead_time=t_req,
                         python=platform.python_version(), numpy=np.__version__))
    drop = {c: float(1 - tracks[c].received.mean()) for c in CONDITIONS}
    res["meta"]["report_loss_fraction"] = drop

    # ---------------------------------------------------------- open loop
    rows, outcomes = [], {}
    for c in CONDITIONS:
        for st in al.STRATEGIES:
            tiers = al.tier_params(cfg, st)
            raw = sim.open_loop_levels(enc, tracks[c], st, tiers)
            for fname, lv in (("raw", raw), ("filtered", al.filter_levels(raw, **FILT))):
                o = M.alert_outcomes(lv, haz, onset, dt, t_req)
                eps = al.episodes_before(lv, onset, dt)
                first = al.first_time_at_or_above(lv, 2, dt)
                lead_first = onset - first
                miss_first = haz & (np.isnan(lead_first) | (lead_first < t_req))
                outcomes[(c, st, fname)] = dict(o, lead_first=lead_first, levels=lv)
                rows.append(dict(condition=c, strategy=st, filter=fname,
                                 missed_or_late=M.rate(o["missed_or_late"], haz),
                                 nuisance=M.rate(o["nuisance"], ~haz),
                                 missed_first_alert=M.rate(miss_first, haz),
                                 lead=lead_stats(o["lead"][haz]),
                                 episodes_mean=float(eps[haz].mean()),
                                 multi_episode=M.rate(eps > 1, haz)))
    res["open_loop"] = rows

    # paired comparisons (same encounters, same tracks)
    pairs = []
    for c in ("nominal", "degraded"):
        for a, b in (("wcv", "cpa"), ("wcv_tau", "wcv"), ("wcv_tau", "cpa")):
            for metric, subset in (("missed_or_late", haz), ("nuisance", ~haz)):
                x = outcomes[(c, a, "filtered")][metric][subset]
                y = outcomes[(c, b, "filtered")][metric][subset]
                pairs.append(dict(condition=c, a=a, b=b, metric=metric, n=int(subset.sum()),
                                  mcnemar=M.mcnemar_exact(x, y),
                                  diff=M.paired_bootstrap_diff(x, y, nb, seed + 11)))
    res["open_loop_paired"] = pairs

    # breakdowns (nominal, filtered)
    brk = []
    for st in al.STRATEGIES:
        o = outcomes[("nominal", st, "filtered")]
        for key, labels, arr in (("geometry", E.GEOMETRIES, enc.geometry),
                                 ("intruder", E.INTRUDERS, enc.intruder),
                                 ("intruder_turn", ("straight", "turning"), enc.params["intruder_turn"].astype(int))):
            for i, lab in enumerate(labels):
                sel = arr == i
                brk.append(dict(strategy=st, by=key, value=lab,
                                missed_or_late=M.rate(o["missed_or_late"], haz & sel),
                                nuisance=M.rate(o["nuisance"], ~haz & sel)))
    res["open_loop_breakdown"] = brk

    # CPA-specific failure mechanisms among its missed/late cases (nominal, filtered)
    hz = cfg["hazard"]["well_clear"]
    k_on = np.where(haz, np.floor(np.nan_to_num(onset) / dt).astype(int), 0)
    r = np.arange(n)
    s_on = enc.int_pos[r, k_on] - enc.own_pos[r, k_on]
    v_on = enc.int_vel[r, k_on] - enc.own_vel[r, k_on]
    k_a = np.where(haz, np.maximum(k_on - int(round(t_req / dt)), 0), 0)
    s_a = enc.int_pos[r, k_a] - enc.own_pos[r, k_a]
    v_a = enc.int_vel[r, k_a] - enc.own_vel[r, k_a]
    tc_a, _, vmd_a = g.cpa(s_a, v_a)
    T_c = al.tier_params(cfg, "cpa")[1]["T"]
    mech = {}
    for st in al.STRATEGIES:
        miss = outcomes[("nominal", st, "filtered")]["missed_or_late"]
        lead = outcomes[("nominal", st, "filtered")]["lead"]
        m = dict(total=int(miss.sum()),
                 no_alert_before_loss=int((miss & np.isnan(lead)).sum()),
                 alert_but_late=int((miss & np.isfinite(lead)).sum()),
                 tcpa_beyond_T_at_required_time=int((miss & (tc_a > T_c)).sum()),
                 vertical_outside_Dv_at_cpa=int((miss & (np.abs(vmd_a) > hz["ZTHR"])).sum()),
                 intruder_turned=int((miss & enc.params["intruder_turn"]).sum()),
                 drone_intruder=int((miss & (enc.intruder == 1)).sum()))
        mech[st] = m
    res["open_loop_mechanisms"] = mech

    # ------------------------------------------------------------- sweeps
    sweep = []
    for st in al.STRATEGIES:
        for T in np.arange(0.0, 121.0, 5.0):
            lv = al.filter_levels(sim.open_loop_levels(enc, tracks["nominal"], st, single_tier(cfg, st, T=T)) * 2, **FILT)
            o = M.alert_outcomes(lv, haz, onset, dt, t_req)
            sweep.append(dict(kind="time", strategy=st, x=float(T),
                              missed_or_late=M.rate(o["missed_or_late"], haz), nuisance=M.rate(o["nuisance"], ~haz)))
        for k in np.round(np.arange(0.5, 2.01, 0.1), 2):
            lv = al.filter_levels(sim.open_loop_levels(enc, tracks["nominal"], st, single_tier(cfg, st, scale=k)) * 2, **FILT)
            o = M.alert_outcomes(lv, haz, onset, dt, t_req)
            sweep.append(dict(kind="scale", strategy=st, x=float(k),
                              missed_or_late=M.rate(o["missed_or_late"], haz), nuisance=M.rate(o["nuisance"], ~haz)))
    res["sweep"] = sweep

    # --------------------------------------------------------- closed loop
    cl_rows, cl = [], {}
    for c in ("nominal", "degraded"):
        base = sim.closed_loop(enc, tracks[c], "wcv", al.tier_params(cfg, "wcv"), "none", cfg, filt=FILT)
        cl[(c, "none", "none")] = base
        for st in al.STRATEGIES:
            for pol in ("heading", "vertical", "combined"):
                cl[(c, st, pol)] = sim.closed_loop(enc, tracks[c], st, al.tier_params(cfg, st), pol, cfg, filt=FILT)
        for key, rr in cl.items():
            if key[0] != c:
                continue
            man = rr.manoeuvred
            cl_rows.append(dict(
                condition=c, strategy=key[1], policy=key[2],
                lowc=M.rate(rr.lowc), nmac=M.rate(rr.nmac),
                lowc_rr=M.paired_bootstrap_ratio(rr.lowc, base.lowc, nb, seed + 21),
                nmac_rr=M.paired_bootstrap_ratio(rr.nmac, base.nmac, nb, seed + 22),
                manoeuvre=M.rate(man), nuisance_manoeuvre=M.rate(man, ~haz),
                induced_lowc=int((rr.lowc & ~haz).sum()),
                emergency=M.rate(rr.emergency), mean_replans=float(rr.n_replans.mean()),
                mean_dpsi_deg=_f(rr.max_dpsi_deg[man].mean()) if man.any() else None,
                mean_dz_m=_f(rr.max_dz_m[man].mean()) if man.any() else None,
                floor_limited=int(rr.floor_limited.sum())))
    res["closed_loop"] = cl_rows

    cl_pairs = []
    for c in ("nominal", "degraded"):
        for a, b in ((("wcv", "heading"), ("cpa", "heading")), (("wcv", "vertical"), ("wcv", "heading")),
                     (("wcv", "combined"), ("wcv", "heading")), (("wcv_tau", "heading"), ("wcv", "heading"))):
            x, y = cl[(c,) + a].lowc, cl[(c,) + b].lowc
            cl_pairs.append(dict(condition=c, a="/".join(a), b="/".join(b), metric="lowc", n=n,
                                 mcnemar=M.mcnemar_exact(x, y), diff=M.paired_bootstrap_diff(x, y, nb, seed + 31)))
    res["closed_loop_paired"] = cl_pairs

    # one-shot manoeuvre (no re-planning) vs default re-planning
    ab = []
    for c in ("nominal", "degraded"):
        for st in ("wcv", "wcv_tau"):
            for pol in ("heading", "vertical"):
                one = sim.closed_loop(enc, tracks[c], st, al.tier_params(cfg, st), pol, cfg, filt=FILT, replan=False)
                rp = cl[(c, st, pol)]
                ab.append(dict(condition=c, strategy=st, policy=pol, one_shot=M.rate(one.lowc), replan=M.rate(rp.lowc),
                               diff=M.paired_bootstrap_diff(rp.lowc, one.lowc, nb, seed + 41),
                               mcnemar=M.mcnemar_exact(rp.lowc, one.lowc)))
    res["replan_ablation"] = ab

    # response-delay sensitivity (wcv, nominal)
    dl = []
    for pol in ("heading", "vertical"):
        for d in (0.0, 5.0, 10.0, 15.0, 20.0):
            rr = sim.closed_loop(enc, tracks["nominal"], "wcv", al.tier_params(cfg, "wcv"), pol, cfg,
                                 filt=FILT, response_delay=d)
            dl.append(dict(policy=pol, delay=d, lowc=M.rate(rr.lowc), nmac=M.rate(rr.nmac)))
    res["delay_sensitivity"] = dl

    # ---------------------------------------------------- fail-case analysis
    fail_rows, fail_summary = [], []
    rec_nom = {c: tracks[c].received for c in ("nominal", "degraded")}
    for (c, st, pol), rr in cl.items():
        if pol == "none":
            continue
        idx = np.flatnonzero(rr.lowc)
        late = np.isnan(rr.first_plan_t) | ~(onset - rr.first_plan_t >= t_req)
        labels = dict(
            no_or_late_alert=late & haz,
            no_conflict_free_candidate=~rr.plan_free,
            intruder_turned=enc.params["intruder_turn"],
            floor_or_ceiling_limited=rr.floor_limited,
            drone_intruder=enc.intruder == 1,
            unmitigated_not_hazardous=~haz,
        )
        summ = dict(condition=c, strategy=st, policy=pol, n_lowc=int(idx.size))
        for lab, arr in labels.items():
            summ[lab] = int(arr[idx].sum())
        explained = np.zeros(n, bool)
        for lab in ("no_or_late_alert", "no_conflict_free_candidate", "intruder_turned", "floor_or_ceiling_limited"):
            explained |= labels[lab]
        summ["none_of_first_four"] = int((~explained[idx]).sum())
        fail_summary.append(summ)
        for i in idx:
            k0 = int(np.nan_to_num(onset[i], nan=rr.onset[i]) // dt)
            lo = max(0, k0 - 30)
            fail_rows.append(dict(
                condition=c, strategy=st, policy=pol, encounter=int(i),
                geometry=E.GEOMETRIES[enc.geometry[i]], intruder=E.INTRUDERS[enc.intruder[i]],
                intruder_turn=bool(enc.params["intruder_turn"][i]),
                h0=round(float(enc.params["h0"][i]), 1), hmd0=round(float(enc.params["hmd0"][i]), 1),
                vmd0=round(float(enc.params["vmd0"][i]), 1), t_cpa0=round(float(enc.params["t_cpa0"][i]), 1),
                unmitigated_onset=_f(onset[i]), mitigated_onset=_f(rr.onset[i]),
                first_plan_t=_f(rr.first_plan_t[i]), plan_free=bool(rr.plan_free[i]),
                floor_limited=bool(rr.floor_limited[i]), emergency=bool(rr.emergency[i]),
                nmac=bool(rr.nmac[i]),
                report_loss_30s_before=round(float(1 - rec_nom[c][i, lo:k0 + 1].mean()), 3),
                labels=";".join(k for k, a in labels.items() if a[i])))
    res["closed_loop_failures"] = fail_summary

    # ------------------------------------------------------------- outputs
    verify_runtime_sources(source_hashes)
    res["meta"]["runtime_s"] = round(time.time() - t_start, 1)
    with open(out / "summary.json", "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(out / "fail_cases.csv", "w", newline="", encoding="utf-8") as f:
        if fail_rows:
            w = csv.DictWriter(f, fieldnames=list(fail_rows[0].keys()))
            w.writeheader()
            w.writerows(fail_rows)
    with open(out / "sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["kind", "strategy", "x", "missed_or_late", "ml_lo", "ml_hi", "nuisance", "nu_lo", "nu_hi"])
        for s in sweep:
            a, b = s["missed_or_late"], s["nuisance"]
            w.writerow([s["kind"], s["strategy"], s["x"], a["p"], a["lo"], a["hi"], b["p"], b["lo"], b["hi"]])
    make_figures(res, outcomes, haz, docs)
    print(f"done in {time.time() - t_start:.1f} s -> {out}")
    return res


def make_figures(res, outcomes, haz, docs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"cpa": "#1f77b4", "wcv": "#d62728", "wcv_tau": "#2ca02c"}
    names = {"cpa": "CPA thresholds", "wcv": "WCV (TTHR 0 s)", "wcv_tau": "WCV + tau_mod (TTHR 20 s)"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, kind, title in ((axes[0], "time", "sweep: corrective alerting time 0-120 s"),
                            (axes[1], "scale", "sweep: threshold scale 0.5-2.0 (T = 55 s)")):
        for st in colors:
            pts = [s for s in res["sweep"] if s["kind"] == kind and s["strategy"] == st]
            x = np.array([p["nuisance"]["p"] for p in pts])
            y = np.array([p["missed_or_late"]["p"] for p in pts])
            xe = np.array([[p["nuisance"]["p"] - p["nuisance"]["lo"], p["nuisance"]["hi"] - p["nuisance"]["p"]] for p in pts]).T
            ye = np.array([[p["missed_or_late"]["p"] - p["missed_or_late"]["lo"], p["missed_or_late"]["hi"] - p["missed_or_late"]["p"]] for p in pts]).T
            ax.errorbar(x, y, xerr=xe, yerr=ye, color=colors[st], lw=1.4, marker="o", ms=3,
                        elinewidth=0.6, capsize=0, label=names[st])
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("nuisance-alert rate (non-hazardous encounters)")
        ax.grid(alpha=0.3)
        xmax = max(p["nuisance"]["hi"] for p in res["sweep"] if p["kind"] == kind)
        ax.set_xlim(0, min(1.0, 1.1 * xmax + 0.02))
        ax.set_ylim(0, 1.02)
    axes[0].set_ylabel("missed-or-late alert rate (hazardous encounters)")
    axes[0].legend(fontsize=8, loc="upper right")
    fig.suptitle("System operating characteristic, nominal surveillance, filtered alerts, 95% Wilson intervals", fontsize=10)
    fig.tight_layout()
    fig.savefig(docs / "soc_curve.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    bins = np.arange(-2.5, 130, 5)
    for st in colors:
        lead = outcomes[("nominal", st, "filtered")]["lead"][haz]
        lead = lead[np.isfinite(lead)]
        ax.hist(lead, bins=bins, histtype="step", lw=1.6, color=colors[st], label=f"{names[st]} (n={lead.size})")
    ax.axvline(res["meta"]["required_lead_time"], color="k", ls="--", lw=1)
    ax.set_xlabel("alert lead time before loss of well clear (s)")
    ax.set_ylabel("hazardous encounters")
    ax.set_title("Corrective-alert lead time, nominal surveillance, filtered alerts", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(docs / "lead_time_hist.png", dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
