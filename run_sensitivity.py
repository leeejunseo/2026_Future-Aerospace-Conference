#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
가정값 민감도 분석
====================

capture_sim.py 의 CONFIG 값 중 문헌 근거 없이 정한 값(`# TODO: 출처 확인 필요`)이
결론을 얼마나 흔드는지 정량화한다. 논문에서 "이 값은 가정이지만 결론은 견고하다" 또는
"이 값이 결론을 좌우하므로 반드시 근거가 필요하다"를 구분해 말하기 위한 근거 자료다.

방법 (One-At-a-Time, OAT)
    각 파라미터를 기준값의 low/high 배수로 바꾸고 나머지는 고정한 채 같은 시행 묶음을 돌린다.
    기준(baseline) 대비 성공률·감속 시간·충격량의 변화를 본다.
    각 조건은 같은 시드 집합을 써서 짝지어(paired) 비교한다.

출력 (기본 sensitivity/)
    sensitivity_raw.csv       시행별 원자료
    sensitivity_summary.csv   파라미터 x 수준별 지표
    sensitivity_tornado.png   기준 대비 성공률 변화 폭 (토네이도 차트)

실행
    conda activate debris-sim
    python run_sensitivity.py --repeats 8 --workers 10
    python run_sensitivity.py --plots-only
"""

from __future__ import annotations

import argparse
import csv
import multiprocessing as mp
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from run_grid_experiment import (EXPERIMENT, METHOD_COLORS, METHOD_LABELS, TARGET_LABELS,
                                 _setup_matplotlib, wilson_interval)

# =============================================================================
# 시험할 가정값과 변동 범위
# =============================================================================
# (CONFIG 경로, 표시 이름, [낮음 배수, 높음 배수], 영향을 보는 방식)
#   배수 대신 절대값을 쓰려면 ("abs", low, high) 형태로 준다.
PARAMS: List[Dict] = [
    # ---- 공통: 대상 물성
    {"path": ("target_dynamics", "restitution"), "name": "반발계수", "abs": (0.05, 0.5), "methods": ["arm", "net", "harpoon"]},
    {"path": ("target_dynamics", "lateral_friction"), "name": "표면 마찰계수", "abs": (0.2, 0.9), "methods": ["arm", "net", "harpoon"]},
    # ---- 서비스 위성
    {"path": ("servicer", "mass"), "name": "서비서 질량", "mult": (0.5, 2.0), "methods": ["arm", "net", "harpoon"]},
    # ---- 로봇팔
    {"path": ("arm", "grasp_force_limit"), "name": "그리퍼 유지력", "mult": (0.5, 2.0), "methods": ["arm"]},
    {"path": ("arm", "joint_max_torque"), "name": "관절 최대 토크", "mult": (0.5, 2.0), "methods": ["arm"]},
    {"path": ("arm", "tool_max_speed"), "name": "그리퍼 접근 속도", "mult": (0.5, 2.0), "methods": ["arm"]},
    {"path": ("arm", "grasp_closure_time"), "name": "그리퍼 닫힘 시간", "mult": (0.5, 2.0), "methods": ["arm"]},
    # ---- 그물
    {"path": ("net", "areal_density"), "name": "그물 면밀도", "mult": (0.5, 2.0), "methods": ["net"]},
    {"path": ("net", "corner_mass_multiple"), "name": "모서리 추 질량", "mult": (0.5, 2.0), "methods": ["net"]},
    {"path": ("net", "deploy_speed"), "name": "그물 발사 속도", "mult": (0.6, 1.5), "methods": ["net"]},
    {"path": ("net", "closing_ratio"), "name": "그물 조임 비율", "abs": (0.08, 0.35), "methods": ["net"]},
    {"path": ("net", "winch_max_tension"), "name": "그물 윈치 장력", "mult": (0.4, 2.5), "methods": ["net"]},
    {"path": ("net", "winch_reel_rate"), "name": "그물 되감기 속도", "mult": (0.5, 2.0), "methods": ["net"]},
    {"path": ("net", "tether_c_factor"), "name": "그물 테더 감쇠", "abs": (0.10, 0.45), "methods": ["net"]},
    # ---- 작살
    {"path": ("harpoon", "speed"), "name": "작살 발사 속도", "mult": (0.6, 1.5), "methods": ["harpoon"]},
    {"path": ("harpoon", "min_penetration_speed"), "name": "관통 임계 속도", "mult": (0.5, 2.0), "methods": ["harpoon"]},
    {"path": ("harpoon", "max_incidence_deg"), "name": "관통 허용 입사각", "abs": (30.0, 60.0), "methods": ["harpoon"]},
    {"path": ("harpoon", "anchor_break_force"), "name": "고정부 파단력", "mult": (0.4, 2.5), "methods": ["harpoon"]},
    {"path": ("harpoon", "winch_max_tension"), "name": "작살 윈치 장력", "mult": (0.4, 2.5), "methods": ["harpoon"]},
    {"path": ("harpoon", "tether_c"), "name": "작살 테더 감쇠", "mult": (0.4, 1.6), "methods": ["harpoon"]},
    # ---- 시나리오 선택 (가정값이 아니라 연구 조건. 결론에 미치는 영향이 커서 함께 정량화한다)
    #   회전축: z 는 접근축(x)에 수직 → 중형 대상의 태양전지판이 접근 평면을 쓸고 지나간다.
    #           x 는 접근축과 일치 → 파지점이 회전축 위에 온다.
    {"path": ("target_dynamics", "spin_axis"), "name": "회전축 (z→x)", "abs": ("z", "x"), "methods": ["arm", "net", "harpoon"]},
    #   세차 운동: 각속도 벡터를 회전축에서 기울여 토크-프리 세차를 일으킨다.
    {"path": ("__spin_mode__",), "name": "세차 운동", "abs": ("principal", "precession"), "methods": ["arm", "net", "harpoon"]},
    # ---- 몬테카를로 불확실성
    {"path": ("monte_carlo", "spin_axis_jitter_deg"), "name": "회전축 추정 오차", "abs": (2.0, 25.0), "methods": ["arm", "net", "harpoon"]},
    {"path": ("monte_carlo", "relative_position_error_m"), "name": "상대 위치 오차", "abs": (0.02, 0.4), "methods": ["arm", "net", "harpoon"]},
]

SENS = {
    "targets": ["small", "medium", "large"],
    "spins_deg_s": [4.0, 12.0, 24.0],   # 저·중·고 회전 대표 3수준
    "repeats": 8,                        # 조건당 반복 (기본 격자보다 적게, 조건 수가 많으므로)
    "seed_base": 5000,
    "max_time": None,                    # None 이면 capture_sim CONFIG 값 사용
}

RAW_FIELDS = ["param", "level", "value", "method", "target", "spin_deg_s", "seed",
              "success", "capture_time", "impulse", "servicer_attitude_error", "final_rel_omega_deg_s", "notes"]


# =============================================================================
# 워커
# =============================================================================
_EXP: Dict = {}


def _init_worker(exp: Dict):
    global _EXP
    _EXP = exp
    import capture_sim as cs
    cs.CONFIG["monte_carlo"].update(exp["monte_carlo"])
    cs.CONFIG["target_dynamics"].update(exp["target_dynamics"])
    if exp.get("max_time"):
        cs.CONFIG["sim"]["max_time"] = exp["max_time"]
    cs.CONFIG["sim"]["record_history"] = False


def _run_one(task: Tuple) -> Dict:
    """task = (param_name, level, path, value, method, target, spin, seed)"""
    import capture_sim as cs
    pname, level, path, value, method, target, spin, seed = task
    spin_mode = "principal"
    old = None
    if path == ("__spin_mode__",):        # 회전 모드는 CONFIG 가 아니라 run_capture 인자
        spin_mode, path = value, None
    if path is not None:
        node = cs.CONFIG
        for k in path[:-1]:
            node = node[k]
        old = node[path[-1]]
        node[path[-1]] = value
    try:
        r = cs.run_capture(method, target, spin, spin_mode, None, seed, False)
    except Exception as e:
        r = {"success": False, "notes": f"EXCEPTION: {e!r}"}
    finally:
        if path is not None:
            node = cs.CONFIG
            for k in path[:-1]:
                node = node[k]
            node[path[-1]] = old
    row = {k: r.get(k) for k in RAW_FIELDS if k not in
           ("param", "level", "value", "method", "target", "spin_deg_s", "seed")}
    row.update(param=pname, level=level, value=value, method=method, target=target,
               spin_deg_s=spin, seed=seed)
    return row


# =============================================================================
# 요약 / 그래프
# =============================================================================
def _f(v) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def summarize(rows: List[Dict]) -> List[Dict]:
    groups: Dict[Tuple[str, str, str], List[Dict]] = {}
    for r in rows:
        groups.setdefault((r["param"], r["level"], r["method"]), []).append(r)
    out = []
    for (pname, level, method), g in sorted(groups.items()):
        succ = [str(r["success"]).lower() == "true" for r in g]
        n, k = len(g), sum(succ)
        lo, hi = wilson_interval(k, n)
        ct = [_f(r["capture_time"]) for r, ok in zip(g, succ) if ok]
        out.append({"param": pname, "level": level, "method": method, "value": g[0]["value"],
                    "n": n, "successes": k, "success_rate": k / n, "ci_low": lo, "ci_high": hi,
                    # 이진 성공률은 반복 수가 적으면 신뢰구간이 매우 넓다.
                    # 잔류 상대 각속도는 연속값이라 같은 반복 수에서 훨씬 작은 분산으로 영향을 드러낸다.
                    "final_rel_omega_median_deg_s": float(np.nanmedian([_f(r["final_rel_omega_deg_s"]) for r in g])),
                    "capture_time_mean_s": float(np.nanmean(ct)) if ct else float("nan"),
                    "impulse_mean_Ns": float(np.nanmean([_f(r["impulse"]) for r in g])),
                    "attitude_error_mean_deg": float(np.nanmean([_f(r["servicer_attitude_error"]) for r in g]))})
    return out


def plot_tornado(summary: List[Dict], path: str, field: str = "success_rate",
                 label: str = "기준 대비 성공률 변화 [%p]", scale: float = 100.0, unit_fmt: str = "{:.0f}%"):
    """기준 대비 지표 변화 폭을 파라미터별 가로 막대로. 방식별 서브플롯."""
    plt = _setup_matplotlib()
    methods = sorted({r["method"] for r in summary})
    base = {r["method"]: r[field] for r in summary if r["param"] == "(기준)"}
    fig, axes = plt.subplots(1, len(methods), figsize=(4.9 * len(methods) + 1, 6.6), squeeze=False,
                             constrained_layout=True)
    for ax, m in zip(axes[0], methods):
        b = base.get(m, 0.0)
        items = {}
        for r in summary:
            if r["method"] != m or r["param"] == "(기준)":
                continue
            items.setdefault(r["param"], {})[r["level"]] = r[field]
        rows = []
        for pname, lv in items.items():
            lo = scale * (lv.get("low", b) - b)
            hi = scale * (lv.get("high", b) - b)
            rows.append((pname, lo, hi, max(abs(lo), abs(hi))))
        rows.sort(key=lambda x: x[3])
        y = np.arange(len(rows))
        c = METHOD_COLORS.get(m, "#2a78d6")
        for i, (pname, lo, hi, _) in enumerate(rows):
            ax.barh(i, lo, color=c, alpha=0.45, height=0.6, edgecolor="none")
            ax.barh(i, hi, color=c, alpha=0.95, height=0.6, edgecolor="none")
        ax.axvline(0, color="#52514e", linewidth=1)
        ax.set_yticks(y); ax.set_yticklabels([r[0] for r in rows], fontsize=9)
        ax.set_xlabel(label)
        ax.set_title(f"{METHOD_LABELS.get(m, m)}  (기준 {unit_fmt.format(scale * b / 100 if scale == 100 else b)})",
                     loc="left", fontsize=11)
        ax.grid(axis="x")
        lim = max(5, max((r[3] for r in rows), default=5) * 1.15)
        ax.set_xlim(-lim, lim)
    n = summary[0]["n"] if summary else 0
    fig.suptitle(f"가정값 민감도 (OAT, 조건당 n={n}, 진한 막대=높은 값 / 연한 막대=낮은 값)",
                 x=0.01, ha="left", fontsize=12)
    fig.savefig(path, dpi=200)
    plt.close(fig)


def write_outputs(rows: List[Dict], out_dir: str):
    if not rows:
        print("no rows")
        return
    summary = summarize(rows)
    sp = os.path.join(out_dir, "sensitivity_summary.csv")
    with open(sp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys())); w.writeheader(); w.writerows(summary)
    plot_tornado(summary, os.path.join(out_dir, "sensitivity_tornado.png"))
    plot_tornado(summary, os.path.join(out_dir, "sensitivity_tornado_rel_omega.png"),
                 field="final_rel_omega_median_deg_s",
                 label="기준 대비 잔류 상대 각속도 변화 [deg/s] (음수 = 감속이 더 잘 됨)",
                 scale=1.0, unit_fmt="{:.2f} deg/s")
    print(f"summary -> {sp}")
    # 콘솔 표: 영향 큰 순
    base = {r["method"]: r["success_rate"] for r in summary if r["param"] == "(기준)"}
    print(f"\n{'method':8s} {'parameter':18s} {'low':>8s} {'high':>8s} {'base':>8s}  변화폭[%p]")
    ranked = []
    for r in summary:
        if r["param"] == "(기준)":
            continue
        ranked.append(r)
    agg: Dict[Tuple[str, str], Dict[str, float]] = {}
    for r in ranked:
        agg.setdefault((r["method"], r["param"]), {})[r["level"]] = r["success_rate"]
    out = []
    for (m, pn), lv in agg.items():
        b = base.get(m, 0.0)
        span = 100 * abs(lv.get("high", b) - lv.get("low", b))
        out.append((span, m, pn, lv.get("low", b), lv.get("high", b), b))
    for span, m, pn, lo, hi, b in sorted(out, reverse=True):
        print(f"{m:8s} {pn:18s} {100*lo:7.1f}% {100*hi:7.1f}% {100*b:7.1f}%  {span:7.1f}")


# =============================================================================
# 실행
# =============================================================================
def build_tasks(exp: Dict) -> List[Tuple]:
    tasks = []
    seeds = [exp["seed_base"] + i for i in range(exp["repeats"])]
    methods = sorted({m for p in PARAMS for m in p["methods"]})
    # 기준 조건
    for m in methods:
        for t in exp["targets"]:
            for s in exp["spins_deg_s"]:
                for sd in seeds:
                    tasks.append(("(기준)", "base", None, None, m, t, s, sd))
    # 파라미터별 low/high
    import capture_sim as cs
    for pdef in PARAMS:
        if pdef["path"] == ("__spin_mode__",):
            baseval = "principal"
        else:
            node = cs.CONFIG
            for k in pdef["path"][:-1]:
                node = node[k]
            baseval = node[pdef["path"][-1]]
        if "abs" in pdef:
            lo, hi = pdef["abs"]
        else:
            lo, hi = baseval * pdef["mult"][0], baseval * pdef["mult"][1]  # type: ignore[operator]
        for level, val in (("low", lo), ("high", hi)):
            for m in pdef["methods"]:
                for t in exp["targets"]:
                    for s in exp["spins_deg_s"]:
                        for sd in seeds:
                            tasks.append((pdef["name"], level, pdef["path"], val, m, t, s, sd))
    return tasks


def main(argv=None):
    ap = argparse.ArgumentParser(description="가정값 민감도 분석 (OAT)")
    ap.add_argument("--repeats", type=int, default=None)
    ap.add_argument("--spins", type=float, nargs="+", default=None)
    ap.add_argument("--targets", nargs="+", default=None)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--max-time", type=float, default=None)
    ap.add_argument("--out", default="sensitivity")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--plots-only", action="store_true")
    args = ap.parse_args(argv)

    exp = dict(SENS)
    exp["monte_carlo"] = EXPERIMENT["monte_carlo"]
    exp["target_dynamics"] = EXPERIMENT["target_dynamics"]
    if args.repeats: exp["repeats"] = args.repeats
    if args.spins: exp["spins_deg_s"] = args.spins
    if args.targets: exp["targets"] = args.targets
    if args.max_time: exp["max_time"] = args.max_time

    os.makedirs(args.out, exist_ok=True)
    raw_path = os.path.join(args.out, "sensitivity_raw.csv")

    if args.plots_only:
        with open(raw_path, newline="", encoding="utf-8") as f:
            write_outputs(list(csv.DictReader(f)), args.out)
        return

    tasks = build_tasks(exp)
    done = set()
    existing: List[Dict] = []
    if args.resume and os.path.exists(raw_path):
        with open(raw_path, newline="", encoding="utf-8") as f:
            existing = list(csv.DictReader(f))
        done = {(r["param"], r["level"], r["method"], r["target"], float(r["spin_deg_s"]), int(r["seed"]))
                for r in existing}
        tasks = [t for t in tasks if (t[0], t[1], t[4], t[5], float(t[6]), t[7]) not in done]
    print(f"tasks: {len(tasks)} (skipped {len(done)}), workers: {args.workers}")
    if not tasks:
        write_outputs(existing, args.out)
        return

    new_file = not (args.resume and os.path.exists(raw_path))
    f = open(raw_path, "w" if new_file else "a", newline="", encoding="utf-8")
    w = csv.DictWriter(f, fieldnames=RAW_FIELDS)
    if new_file:
        w.writeheader()
    rows = list(existing)
    t0 = time.time(); n_done = 0; total = len(tasks)
    try:
        with mp.Pool(args.workers, initializer=_init_worker, initargs=(exp,)) as pool:
            for row in pool.imap_unordered(_run_one, tasks, chunksize=1):
                w.writerow(row); f.flush(); rows.append(row); n_done += 1
                if n_done % 25 == 0 or n_done == total:
                    el = time.time() - t0
                    print(f"[{n_done}/{total}] {el/60:.1f} min, ETA {el/n_done*(total-n_done)/60:.1f} min", flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted — --resume 으로 이어서 실행 가능")
    finally:
        f.close()
    write_outputs(rows, args.out)


if __name__ == "__main__":
    mp.freeze_support()
    main()
