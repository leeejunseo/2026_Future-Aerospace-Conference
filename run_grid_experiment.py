#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
격자 몬테카를로 실험: 회전 속도 x 질량 x 포획 방식
=====================================================

capture_sim.py 의 물리 시뮬레이션을 아래 격자로 반복 실행한다.
    회전 속도 : 0 ~ 30 deg/s, 2 deg/s 간격 (16 수준)
    질량      : small(10 kg) / medium(500 kg) / large(2000 kg)
    포획 방식 : arm / net / harpoon
    반복      : 조합당 30회, 시드를 바꿔가며 (기본 4320 회)

"실제 우주에서 하듯이" : 매 반복마다 실제 임무에서 불확실한 양을 시드 기반으로 섭동한다
(EXPERIMENT["monte_carlo"] 참고). 같은 반복 번호는 모든 조합에서 같은 시드를 쓴다
(공통 난수, common random numbers) → 방식 간 비교가 짝지어져(paired) 분산이 줄어든다.

출력 (기본 results/ 폴더)
    results_raw.csv       시행별 원자료 (중단 후 --resume 로 이어서 실행 가능)
    results_summary.csv   조합별 성공률 + Wilson 95 % 신뢰구간 + 보조 지표 평균
    heatmap_success.png   방식별 회전속도 x 질량 성공률 히트맵
    curves_success.png    질량별 회전속도-성공률 곡선 (95 % CI 밴드)

실행 예
    conda activate debris-sim
    python run_grid_experiment.py                      # 전체 격자 (수 시간 소요)
    python run_grid_experiment.py --workers 8          # 병렬 프로세스 수 지정
    python run_grid_experiment.py --resume             # 중단된 실험 이어서
    python run_grid_experiment.py --spins 0 10 20 30 --repeats 3 --out results_quick   # 빠른 확인
    python run_grid_experiment.py --plots-only --out results   # CSV 로부터 요약/그래프만 재생성
"""

from __future__ import annotations

import argparse
import csv
import math
import multiprocessing as mp
import os
import sys
import time
from typing import Dict, List, Tuple

import numpy as np

# =============================================================================
# 실험 설정
# =============================================================================
EXPERIMENT: Dict = {
    "spins_deg_s": [float(v) for v in range(0, 31, 2)],   # 0~30 deg/s, 2 deg/s 간격
    "targets": ["small", "medium", "large"],
    "methods": ["arm", "net", "harpoon"],
    "repeats": 30,                 # 조합당 반복 횟수
    "seed_base": 1000,             # 반복 i 의 시드 = seed_base + i (모든 조합 공통 → paired 비교)
    "spin_mode": "principal",      # "principal" 또는 "precession"
    # 시행당 최대 시뮬레이션 시간 [s]. None 이면 capture_sim.CONFIG["sim"]["max_time"] 을 그대로 쓴다.
    # 여기에 숫자를 적으면 capture_sim 의 값을 조용히 덮어써서, 시간 창을 늘린 사실이 실험에 반영되지 않는다.
    # 실제로 이 값이 60 으로 남아 있어 테더 방식(감속에 50~130 s 필요)이 전부 무효가 될 뻔했다.
    "max_time": None,
    "ci_level": 0.95,              # 성공률 신뢰구간 수준 (Wilson score interval)
    # ---- 실제 임무 불확실성 (시드로 섭동). capture_sim.CONFIG["monte_carlo"] 를 덮어씀
    "monte_carlo": {
        "spin_axis_jitter_deg": 10.0,        # 지상 관측 기반 회전축 추정 오차  # TODO: 출처 확인 필요
        "spin_rate_jitter_frac": 0.05,       # 회전 속도 추정 오차 5 %  # TODO: 출처 확인 필요
        "relative_position_error_m": 0.10,   # 근접 항법 횡방향 위치 오차  # TODO: 출처 확인 필요
        "relative_velocity_error_mps": 0.02,  # 잔류 상대 속도  # TODO: 출처 확인 필요
        "servicer_pointing_error_deg": 1.0,  # 서비서 지향 오차  # TODO: 출처 확인 필요
    },
    "target_dynamics": {
        "initial_attitude_jitter_deg": 10.0,  # 초기 자세 섭동 표준편차 (capture_sim 기본 5 → 10)  # TODO: 출처 확인 필요
    },
}

TARGET_LABELS = {"small": "소형 10 kg", "medium": "중형 500 kg", "large": "대형 2000 kg"}
METHOD_LABELS = {"arm": "로봇팔", "net": "그물", "harpoon": "작살"}
# 범주형 색: 고정 순서(파랑/주황/청록). dataviz 기준 팔레트, 색각이상 검증된 첫 3슬롯
METHOD_COLORS = {"arm": "#2a78d6", "net": "#eb6834", "harpoon": "#1baf7a"}
# 순차형(단일 색상, 연→진) 파랑 램프: 히트맵용
SEQUENTIAL_BLUE = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
                   "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]

# 방식별 1회 시행의 벽시계 시간은 자릿수가 다르다 (그물은 질점 81개를 매 스텝 갱신한다).
# 아직 실측이 없는 방식에 쓸 초기 추정치 [s]. 관측이 쌓이면 실측 평균으로 대체된다.
WALL_TIME_PRIOR = {"net": 360.0, "harpoon": 90.0, "arm": 25.0}

RAW_FIELDS = ["method", "target", "spin_deg_s", "seed", "rep", "success", "capture_time", "impulse",
              "servicer_attitude_error", "angular_impulse", "peak_contact_force", "max_interaction_force",
              "final_rel_omega_deg_s", "first_contact_time", "bond_intact_final", "sim_time", "wall_time_s", "notes"]


# =============================================================================
# 워커
# =============================================================================
def _apply_overrides(exp: Dict):
    import capture_sim as cs
    cs.CONFIG["monte_carlo"].update(exp["monte_carlo"])
    cs.CONFIG["target_dynamics"].update(exp["target_dynamics"])
    if exp.get("max_time"):
        cs.CONFIG["sim"]["max_time"] = exp["max_time"]
    cs.CONFIG["sim"]["record_history"] = False


_EXP: Dict = {}


def _init_worker(exp: Dict):
    global _EXP
    _EXP = exp
    _apply_overrides(exp)


def _run_one(task: Tuple[str, str, float, int, int]) -> Dict:
    import capture_sim as cs
    method, target, spin, seed, rep = task
    t0 = time.time()
    try:
        r = cs.run_capture(method, target, spin, _EXP["spin_mode"], None, seed, False)
    except Exception as e:  # 한 시행의 예외가 전체 실험을 멈추지 않도록
        r = {"success": False, "notes": f"EXCEPTION: {e!r}"}
    row = {k: r.get(k) for k in RAW_FIELDS if k not in ("method", "target", "spin_deg_s", "seed", "rep", "wall_time_s")}
    row.update(method=method, target=target, spin_deg_s=spin, seed=seed, rep=rep, wall_time_s=round(time.time() - t0, 2))
    return row


# =============================================================================
# 통계
# =============================================================================
def wilson_interval(k: int, n: int, level: float = 0.95) -> Tuple[float, float]:
    """이항 비율의 Wilson score 신뢰구간 (n 이 작거나 p 가 0/1 근처여도 안정적)"""
    if n == 0:
        return (float("nan"), float("nan"))
    z = {0.90: 1.6449, 0.95: 1.9600, 0.99: 2.5758}[level]
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def _to_float(v) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def summarize(rows: List[Dict], level: float) -> List[Dict]:
    groups: Dict[Tuple[str, str, float], List[Dict]] = {}
    for r in rows:
        groups.setdefault((r["method"], r["target"], float(r["spin_deg_s"])), []).append(r)
    out = []
    for (m, t, s), g in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2])):
        succ = [str(r["success"]).lower() == "true" for r in g]
        n, k = len(g), sum(succ)
        lo, hi = wilson_interval(k, n, level)
        ct = [_to_float(r["capture_time"]) for r, ok in zip(g, succ) if ok]
        imp = [_to_float(r["impulse"]) for r in g]
        att = [_to_float(r["servicer_attitude_error"]) for r in g]
        out.append({
            "method": m, "target": t, "spin_deg_s": s, "n": n, "successes": k,
            "success_rate": k / n, "ci_low": lo, "ci_high": hi,
            "capture_time_mean_s": float(np.nanmean(ct)) if ct else float("nan"),
            "capture_time_std_s": float(np.nanstd(ct)) if ct else float("nan"),
            "capture_time_median_s": float(np.nanmedian(ct)) if ct else float("nan"),
            # 이진 성공률만 보면 "거의 성공"과 "전혀 안 됨"이 구분되지 않는다.
            # 최종 상대 각속도(중앙값)를 함께 보고해 감속이 어디까지 진행됐는지 남긴다.
            "final_rel_omega_median_deg_s": float(np.nanmedian([_to_float(r["final_rel_omega_deg_s"]) for r in g])),
            "bond_intact_rate": float(np.mean([str(r["bond_intact_final"]).lower() == "true" for r in g])),
            "impulse_mean_Ns": float(np.nanmean(imp)), "impulse_max_Ns": float(np.nanmax(imp)),
            "servicer_attitude_error_mean_deg": float(np.nanmean(att)),
        })
    return out


# =============================================================================
# 그래프
# =============================================================================
def _setup_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for name in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams.update({"figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                         "axes.edgecolor": "#c9c8c2", "axes.labelcolor": "#0b0b0b",
                         "xtick.color": "#52514e", "ytick.color": "#52514e",
                         "axes.spines.top": False, "axes.spines.right": False,
                         "grid.color": "#e6e5e0", "grid.linewidth": 0.8, "font.size": 10})
    return plt


def plot_metric_heatmaps(summary: List[Dict], exp: Dict, path: str, field: str, title: str,
                         unit: str, vmin=None, vmax=None, log: bool = False, reverse: bool = False,
                         fmt: str = "{:.0f}"):
    """임의 지표를 방식별 회전속도 x 질량 히트맵으로. 성공률 히트맵과 같은 배치를 쓴다."""
    plt = _setup_matplotlib()
    from matplotlib.colors import LinearSegmentedColormap, LogNorm, Normalize
    ramp = SEQUENTIAL_BLUE[::-1] if reverse else SEQUENTIAL_BLUE
    cmap = LinearSegmentedColormap.from_list("seq", ramp)
    cmap = cmap.with_extremes(bad="#eceae4")
    spins = sorted({r["spin_deg_s"] for r in summary})
    targets = [t for t in exp["targets"] if any(r["target"] == t for r in summary)]
    methods = [m for m in exp["methods"] if any(r["method"] == m for r in summary)]
    table = {(r["method"], r["target"], r["spin_deg_s"]): r for r in summary}
    mats = {}
    for m in methods:
        M = np.full((len(targets), len(spins)), np.nan)
        for i, t in enumerate(targets):
            for j, s in enumerate(spins):
                r = table.get((m, t, s))
                if r is not None:
                    M[i, j] = r.get(field, np.nan)
        mats[m] = M
    allv = np.concatenate([M[np.isfinite(M)] for M in mats.values()]) if mats else np.array([0.0])
    if allv.size == 0:
        allv = np.array([0.0, 1.0])
    lo = vmin if vmin is not None else float(np.nanmin(allv))
    hi = vmax if vmax is not None else float(np.nanmax(allv))
    if log:
        lo = max(lo, 1e-3)
        norm = LogNorm(vmin=lo, vmax=max(hi, lo * 10))
    else:
        norm = Normalize(vmin=lo, vmax=hi if hi > lo else lo + 1)
    fig, axes = plt.subplots(len(methods), 1, figsize=(max(8, 0.55 * len(spins) + 2), 2.1 * len(methods) + 1.2),
                             squeeze=False, constrained_layout=True)
    im = None
    for ax, m in zip(axes[:, 0], methods):
        M = mats[m]
        im = ax.imshow(M, cmap=cmap, norm=norm, aspect="auto")
        ax.set_xticks(range(len(spins))); ax.set_xticklabels([f"{s:g}" for s in spins])
        ax.set_yticks(range(len(targets))); ax.set_yticklabels([TARGET_LABELS[t] for t in targets])
        ax.set_title(f"{METHOD_LABELS[m]} · {title}", loc="left", fontsize=11, color="#0b0b0b")
        ax.set_xlabel("회전 속도 [deg/s]")
        ax.tick_params(length=0)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.set_xticks(np.arange(-0.5, len(spins), 1), minor=True)
        ax.set_yticks(np.arange(-0.5, len(targets), 1), minor=True)
        ax.grid(which="minor", color="#fcfcfb", linewidth=2)
        ax.tick_params(which="minor", length=0)
        for i in range(len(targets)):
            for j in range(len(spins)):
                v = M[i, j]
                if np.isfinite(v):
                    frac = norm(v)
                    ax.text(j, i, fmt.format(v), ha="center", va="center", fontsize=8,
                            color="#ffffff" if frac > 0.55 else "#0b0b0b")
    cbar = fig.colorbar(im, ax=axes[:, 0].tolist(), shrink=0.8, pad=0.02)
    cbar.set_label(unit)
    cbar.outline.set_visible(False)
    n = summary[0]["n"] if summary else 0
    fig.suptitle(f"{title} (조합당 n={n}, 회색 = 해당 없음)", x=0.01, ha="left", fontsize=12)
    fig.savefig(path, dpi=200)
    plt.close(fig)


def plot_heatmaps(summary: List[Dict], exp: Dict, path: str):
    plt = _setup_matplotlib()
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("seq_blue", SEQUENTIAL_BLUE)
    spins = sorted({r["spin_deg_s"] for r in summary})
    targets = [t for t in exp["targets"] if any(r["target"] == t for r in summary)]
    methods = [m for m in exp["methods"] if any(r["method"] == m for r in summary)]
    table = {(r["method"], r["target"], r["spin_deg_s"]): r for r in summary}
    fig, axes = plt.subplots(len(methods), 1, figsize=(max(8, 0.55 * len(spins) + 2), 2.1 * len(methods) + 1.2),
                             squeeze=False, constrained_layout=True)
    for ax, m in zip(axes[:, 0], methods):
        M = np.full((len(targets), len(spins)), np.nan)
        for i, t in enumerate(targets):
            for j, s in enumerate(spins):
                r = table.get((m, t, s))
                if r:
                    M[i, j] = 100 * r["success_rate"]
        im = ax.imshow(M, cmap=cmap, vmin=0, vmax=100, aspect="auto")
        ax.set_xticks(range(len(spins))); ax.set_xticklabels([f"{s:g}" for s in spins])
        ax.set_yticks(range(len(targets))); ax.set_yticklabels([TARGET_LABELS[t] for t in targets])
        ax.set_title(f"{METHOD_LABELS[m]} 성공률 [%]", loc="left", fontsize=11, color="#0b0b0b")
        ax.set_xlabel("회전 속도 [deg/s]")
        ax.tick_params(length=0)
        for spine in ax.spines.values():
            spine.set_visible(False)
        # 셀 사이 2 px 여백 효과: 흰 격자선
        ax.set_xticks(np.arange(-0.5, len(spins), 1), minor=True)
        ax.set_yticks(np.arange(-0.5, len(targets), 1), minor=True)
        ax.grid(which="minor", color="#fcfcfb", linewidth=2)
        ax.tick_params(which="minor", length=0)
        for i in range(len(targets)):
            for j in range(len(spins)):
                v = M[i, j]
                if not np.isnan(v):
                    ax.text(j, i, f"{v:.0f}", ha="center", va="center", fontsize=8,
                            color="#ffffff" if v > 55 else "#0b0b0b")
    cbar = fig.colorbar(im, ax=axes[:, 0].tolist(), shrink=0.8, pad=0.02)
    cbar.set_label("성공률 [%]")
    cbar.outline.set_visible(False)
    n = summary[0]["n"] if summary else 0
    mode = "주축 회전" if exp.get("spin_mode") == "principal" else "세차 운동"
    axis = exp.get("target_dynamics", {}).get("spin_axis")
    if axis and axis != "z":
        mode += f", 회전축 {axis}"
    fig.suptitle(f"회전 속도 × 질량별 포획 성공률 (조합당 n={n}, {mode})", x=0.01, ha="left", fontsize=12)
    fig.savefig(path, dpi=200)
    plt.close(fig)


def plot_curves(summary: List[Dict], exp: Dict, path: str):
    plt = _setup_matplotlib()
    spins = sorted({r["spin_deg_s"] for r in summary})
    targets = [t for t in exp["targets"] if any(r["target"] == t for r in summary)]
    methods = [m for m in exp["methods"] if any(r["method"] == m for r in summary)]
    table = {(r["method"], r["target"], r["spin_deg_s"]): r for r in summary}
    fig, axes = plt.subplots(1, len(targets), figsize=(4.6 * len(targets) + 1, 4.2), sharey=True,
                             squeeze=False, constrained_layout=True)
    for ax, t in zip(axes[0], targets):
        end_labels = []  # (y, label, color) — 오른쪽 끝 직접 라벨, 겹치면 세로로 밀어냄
        for m in methods:
            xs, ys, lo, hi = [], [], [], []
            for s in spins:
                r = table.get((m, t, s))
                if r:
                    xs.append(s); ys.append(100 * r["success_rate"])
                    lo.append(100 * r["ci_low"]); hi.append(100 * r["ci_high"])
            if not xs:
                continue
            c = METHOD_COLORS[m]
            ax.fill_between(xs, lo, hi, color=c, alpha=0.12, linewidth=0)
            ax.plot(xs, ys, color=c, linewidth=2, marker="o", markersize=4, label=METHOD_LABELS[m],
                    markeredgecolor="#fcfcfb", markeredgewidth=1)
            end_labels.append([ys[-1], METHOD_LABELS[m], c, xs[-1]])
        # 직접 라벨 겹침 해소: y 순으로 정렬해 최소 간격(7 %p) 확보
        end_labels.sort(key=lambda e: e[0])
        for i in range(1, len(end_labels)):
            if end_labels[i][0] - end_labels[i - 1][0] < 7:
                end_labels[i][0] = end_labels[i - 1][0] + 7
        for y, lab, c, x in end_labels:
            ax.annotate(lab, (x, y), xytext=(6, 0), textcoords="offset points", va="center",
                        fontsize=9, color="#52514e", annotation_clip=False)
        ax.set_title(TARGET_LABELS[t], loc="left", fontsize=11)
        ax.set_xlabel("회전 속도 [deg/s]")
        ax.set_ylim(-3, 103); ax.set_xlim(min(spins) - 1, max(spins) + 4)
        ax.grid(axis="y")
    axes[0][0].set_ylabel("성공률 [%]")
    axes[0][0].set_ylim(-3, 108)
    lvl = int(100 * exp["ci_level"])
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, title=f"음영: Wilson {lvl}% 신뢰구간", frameon=False,
               loc="outside lower center", ncol=len(labels) + 1)
    n = summary[0]["n"] if summary else 0
    axis = exp.get("target_dynamics", {}).get("spin_axis")
    axis_note = f", 회전축 {axis}" if axis and axis != "z" else ""
    fig.suptitle(f"회전 속도에 따른 포획 성공률 (조합당 n={n}{axis_note})", x=0.01, ha="left", fontsize=12)
    fig.savefig(path, dpi=200)
    plt.close(fig)


# =============================================================================
# 실행 관리
# =============================================================================
def load_raw(path: str) -> List[Dict]:
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_summary(rows: List[Dict], exp: Dict, out_dir: str):
    if not rows:
        print("no rows to summarize")
        return
    summary = summarize(rows, exp["ci_level"])
    spath = os.path.join(out_dir, "results_summary.csv")
    with open(spath, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        wr.writeheader(); wr.writerows(summary)
    plot_heatmaps(summary, exp, os.path.join(out_dir, "heatmap_success.png"))
    plot_curves(summary, exp, os.path.join(out_dir, "curves_success.png"))
    # 이진 성공률만으로는 "거의 다 감속했다"와 "전혀 안 됐다"가 구분되지 않는다.
    plot_metric_heatmaps(summary, exp, os.path.join(out_dir, "heatmap_final_rel_omega.png"),
                         "final_rel_omega_median_deg_s", "잔류 상대 각속도 (중앙값)", "deg/s",
                         log=True, reverse=True, fmt="{:.1f}")
    plot_metric_heatmaps(summary, exp, os.path.join(out_dir, "heatmap_capture_time.png"),
                         "capture_time_median_s", "감속 소요 시간 (성공 시행 중앙값)", "s",
                         reverse=True, fmt="{:.0f}")
    plot_metric_heatmaps(summary, exp, os.path.join(out_dir, "heatmap_impulse.png"),
                         "impulse_mean_Ns", "최대 충격량 평균 (2차 파편 위험)", "N·s",
                         log=True, reverse=True, fmt="{:.0f}")
    print(f"summary -> {spath}")
    print(f"plots   -> {os.path.join(out_dir, 'heatmap_success.png')}, {os.path.join(out_dir, 'curves_success.png')}")
    # 콘솔 요약표
    print(f"\n{'method':8s} {'target':7s} " + " ".join(f"{s:>5g}" for s in sorted({r['spin_deg_s'] for r in summary})))
    for m in exp["methods"]:
        for t in exp["targets"]:
            vals = [r for r in summary if r["method"] == m and r["target"] == t]
            if vals:
                print(f"{m:8s} {t:7s} " + " ".join(f"{100*r['success_rate']:5.0f}" for r in sorted(vals, key=lambda r: r['spin_deg_s'])))


def main(argv=None):
    ap = argparse.ArgumentParser(description="회전속도 x 질량 x 포획방식 격자 몬테카를로 실험")
    ap.add_argument("--spins", type=float, nargs="+", default=None, help="회전 속도 목록 [deg/s] (기본 0~30, 2 간격)")
    ap.add_argument("--targets", nargs="+", default=None, choices=["small", "medium", "large"])
    ap.add_argument("--methods", nargs="+", default=None, choices=["arm", "net", "harpoon"])
    ap.add_argument("--repeats", type=int, default=None)
    ap.add_argument("--seed-base", type=int, default=None)
    ap.add_argument("--spin-mode", choices=["principal", "precession"], default=None)
    ap.add_argument("--spin-axis", choices=["x", "y", "z", "major", "minor"], default=None,
                    help="대상 동체 좌표의 주축 회전축 (기본 z: 접근축 x 에 수직). 서비서는 -x 쪽에서 접근한다")
    ap.add_argument("--max-time", type=float, default=None)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", default="results", help="출력 폴더")
    ap.add_argument("--resume", action="store_true", help="기존 results_raw.csv 에 있는 시행은 건너뜀")
    ap.add_argument("--plots-only", action="store_true", help="시뮬레이션 없이 CSV 로부터 요약/그래프만 생성")
    args = ap.parse_args(argv)

    exp = {k: (dict(v) if isinstance(v, dict) else list(v) if isinstance(v, list) else v) for k, v in EXPERIMENT.items()}
    if args.spins is not None: exp["spins_deg_s"] = args.spins
    if args.targets: exp["targets"] = args.targets
    if args.methods: exp["methods"] = args.methods
    if args.repeats is not None: exp["repeats"] = args.repeats
    if args.seed_base is not None: exp["seed_base"] = args.seed_base
    if args.spin_mode: exp["spin_mode"] = args.spin_mode
    if args.spin_axis: exp["target_dynamics"]["spin_axis"] = args.spin_axis
    if args.max_time: exp["max_time"] = args.max_time

    os.makedirs(args.out, exist_ok=True)
    raw_path = os.path.join(args.out, "results_raw.csv")

    if args.plots_only:
        write_summary(load_raw(raw_path), exp, args.out)
        return

    # 설정 기록 (재현성)
    with open(os.path.join(args.out, "experiment_config.txt"), "w", encoding="utf-8") as f:
        import capture_sim as cs
        _apply_overrides(exp)
        f.write("EXPERIMENT = " + repr(exp) + "\n\nCONFIG = " + repr(cs.CONFIG) + "\n")

    done = set()
    existing = []
    if args.resume:
        existing = load_raw(raw_path)
        done = {(r["method"], r["target"], float(r["spin_deg_s"]), int(r["seed"])) for r in existing}
    tasks = []
    for rep in range(exp["repeats"]):
        seed = exp["seed_base"] + rep
        for m in exp["methods"]:
            for t in exp["targets"]:
                for s in exp["spins_deg_s"]:
                    if (m, t, float(s), seed) not in done:
                        tasks.append((m, t, float(s), seed, rep))
    # 반복 번호 순으로 돌려, 중간에 멈춰도 모든 조합이 같은 반복 수만큼 채워져 있게 한다.
    # 같은 반복 안에서는 오래 걸리는 그물 시행을 먼저 배치해 마지막에 한 워커만 남아 도는 시간을 줄인다.
    # (시행마다 시드가 고정되어 있으므로 실행 순서는 결과에 영향이 없다)
    tasks.sort(key=lambda x: (x[4], {"net": 0, "arm": 1, "harpoon": 2}[x[0]]))
    total = len(tasks)
    print(f"tasks: {total} (skipped {len(done)} already done), workers: {args.workers}")
    if total == 0:
        write_summary(existing, exp, args.out)
        return

    new_file = not (args.resume and os.path.exists(raw_path))
    f = open(raw_path, "a" if not new_file else "w", newline="", encoding="utf-8")
    wr = csv.DictWriter(f, fieldnames=RAW_FIELDS)
    if new_file:
        wr.writeheader()
    t0 = time.time()
    n_done = 0
    rows = list(existing)
    seen: Dict[str, List[float]] = {}                  # 방식별 실측 벽시계 시간
    remaining: Dict[str, int] = {}                     # 방식별 남은 작업 수
    for t in tasks:
        remaining[t[0]] = remaining.get(t[0], 0) + 1
    try:
        with mp.Pool(args.workers, initializer=_init_worker, initargs=(exp,)) as pool:
            for row in pool.imap_unordered(_run_one, tasks, chunksize=1):
                wr.writerow(row); f.flush()
                rows.append(row); n_done += 1
                seen.setdefault(row["method"], []).append(float(row["wall_time_s"]))
                remaining[row["method"]] = remaining.get(row["method"], 1) - 1
                if n_done % 10 == 0 or n_done == total:
                    el = time.time() - t0
                    # 남은 작업을 방식별 실측 평균으로 환산한다. 지금까지의 전체 평균을 쓰면
                    # 느린 방식부터 처리되는 순서 때문에 ETA 가 몇 배로 부풀려진다.
                    cpu_left = 0.0
                    for meth, cnt in remaining.items():
                        obs = seen.get(meth)
                        mean = (sum(obs) / len(obs)) if obs else WALL_TIME_PRIOR.get(meth, 60.0)
                        cpu_left += cnt * mean
                    eta = cpu_left / max(1, args.workers)
                    rates = " ".join(f"{m}:{sum(v)/len(v):.0f}s" for m, v in sorted(seen.items()))
                    print(f"[{n_done}/{total}] elapsed {el/60:.1f} min, ETA {eta/60:.0f} min ({rates}) | "
                          f"last: {row['method']} {row['target']} spin={row['spin_deg_s']:g} seed={row['seed']} "
                          f"success={row['success']}", flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted — 지금까지의 결과로 요약 생성 (--resume 으로 이어서 실행 가능)")
    finally:
        f.close()
    write_summary(rows, exp, args.out)


if __name__ == "__main__":
    mp.freeze_support()
    main()
