"""
회전축 추가 실험 분석 (중형 500 kg 대상)

본 격자 실험(results/)은 대상을 동체 z 축으로 회전시켰다. z 축은 서비서의 접근축(x)에 수직이며,
중형 대상에서는 최대 관성축이다. 이 스크립트는 같은 대상을 x 축(접근축, 중간 관성축)과
y 축(태양전지판 방향, 최소 관성축)으로 회전시킨 추가 실험(results_axis/x, results_axis/y)과 합쳐,
방식 간 우열이 회전축에 따라 바뀌는지 본다.

비교는 세 축에 모두 있는 (방식, 회전속도, 시드) 시행끼리만 한다. 실험이 도중에 멈춰 있어도
같은 조건끼리 비교되고, 시드가 같으므로 축 간 비교도 짝지은 비교가 된다.
회전하지 않는 대상(0 deg/s)은 회전축이 의미가 없으므로 2~30 deg/s 만 쓴다.

출력 (--out 폴더):
    axis_summary.csv     축 × 방식별 성공률 (Wilson 95 % CI), 잔류 상대 각속도 중앙값
    axis_tests.csv       축 안 방식 간 짝지은 검정, 방식 안 축 간 짝지은 검정 (Holm 보정)
    axis_report.md       위 결과를 원고에 옮기기 좋은 형태로 정리
    fig_axis_rates.png   축별 방식 성공률 막대 (오차막대 = Wilson 95 % CI)
    fig_axis_curves.png  축별 회전 속도-성공률 곡선

사용법:
    python analyze_axis.py
    python analyze_axis.py --z-results results/results_raw.csv --axis-dir results_axis --out results_axis
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import Dict, List, Tuple

import numpy as np

from analyze_compare import holm, mcnemar_exact, paired_bootstrap_diff, stars, wilcoxon_rel_omega
from run_grid_experiment import METHOD_COLORS, METHOD_LABELS, _setup_matplotlib, wilson_interval

TARGET = "medium"
METHODS = ["arm", "net", "harpoon"]
AXES = ["z", "x", "y"]
# 중형 대상의 동체 좌표 주관성모멘트 [kg m^2]. capture_sim 의 모델(본체 450 kg + 패널 2장)에서
# PyBullet 이 계산한 값과 같다: Target(...).inertia_diag == (219.7, 79.2, 223.8)
INERTIA = {"x": 219.7, "y": 79.2, "z": 223.8}
AXIS_LABELS = {
    "z": "z축 (접근축에 수직, 최대 관성축) — 본 실험",
    "x": "x축 (접근축, 중간 관성축)",
    "y": "y축 (패널 방향, 최소 관성축)",
}
AXIS_SHORT = {"z": "z축 (본 실험)", "x": "x축 (접근축)", "y": "y축 (패널축)"}
METHOD_PAIRS = [("net", "arm"), ("net", "harpoon"), ("arm", "harpoon")]

Key = Tuple[str, float, int]   # (방식, 회전속도, 시드)


# =============================================================================
# 자료
# =============================================================================
def load_axis(path: str) -> Tuple[Dict[Key, Dict], int]:
    """중형·회전 대상 시행만 (방식, 회전속도, 시드) 로 색인한다. 예외로 끝난 시행 수도 센다."""
    if not os.path.exists(path):
        raise SystemExit(f"원자료가 없다: {path}\n먼저 `bash run_all.sh 10 axis` 로 회전축 실험을 돌릴 것.")
    out, n_exc = {}, 0
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            if r["target"] != TARGET or float(r["spin_deg_s"]) <= 0:
                continue
            if str(r.get("notes", "")).startswith("EXCEPTION"):
                n_exc += 1
            out[(r["method"], float(r["spin_deg_s"]), int(r["seed"]))] = r
    return out, n_exc


def arrays(rows: Dict[Key, Dict], keys: List[Key]) -> Tuple[np.ndarray, np.ndarray]:
    s = np.array([str(rows[k]["success"]).lower() == "true" for k in keys])
    w = []
    for k in keys:
        try:
            w.append(float(rows[k]["final_rel_omega_deg_s"]))
        except (TypeError, ValueError):
            w.append(float("nan"))
    return s, np.array(w)


def swap_method(keys: List[Key], m: str) -> List[Key]:
    return [(m, s, seed) for (_, s, seed) in keys]


# =============================================================================
# 분석
# =============================================================================
def summarize(data: Dict[str, Dict[Key, Dict]], common: Dict[str, List[Key]]) -> List[Dict]:
    out = []
    for ax in AXES:
        for m in METHODS:
            s, w = arrays(data[ax], common[m])
            k, n = int(s.sum()), len(s)
            lo, hi = wilson_interval(k, n)
            out.append({"axis": ax, "method": m, "n": n, "successes": k,
                        "success_rate": round(k / n, 6) if n else float("nan"),
                        "ci_low": round(lo, 6), "ci_high": round(hi, 6),
                        "median_final_rel_omega_deg_s": round(float(np.nanmedian(w)), 3) if n else float("nan")})
    return out


def _test(s1, s2, w1, w2) -> Dict:
    b, c, p = mcnemar_exact(s1, s2)
    diff, dlo, dhi = paired_bootstrap_diff(s1, s2)
    med, pw = wilcoxon_rel_omega(w1, w2)
    return {"n_pairs": len(s1), "rate_1": round(float(s1.mean()), 6), "rate_2": round(float(s2.mean()), 6),
            "diff": round(diff, 6), "diff_ci_low": round(dlo, 6), "diff_ci_high": round(dhi, 6),
            "only_1_success": b, "only_2_success": c, "mcnemar_p": p,
            "median_omega_diff": round(med, 3), "wilcoxon_p": pw}


def run_tests(data: Dict[str, Dict[Key, Dict]], common: Dict[str, List[Key]]) -> List[Dict]:
    """
    가족 1: 같은 축 안에서 방식끼리 (축 3 × 쌍 3 = 9 검정)
    가족 2: 같은 방식 안에서 z 축 대비 x, y 축 (방식 3 × 2 = 6 검정)
    가족마다 McNemar p 와 Wilcoxon p 에 각각 Holm 보정을 한다.
    """
    fam1, fam2 = [], []
    common_sets = {m: set(common[m]) for m in METHODS}
    for ax in AXES:
        for m1, m2 in METHOD_PAIRS:
            # 두 방식 모두 세 축 공통 시행에 있는 (회전속도, 시드) 만
            k1 = [k for k in common[m1] if (m2, k[1], k[2]) in common_sets[m2]]
            s1, w1 = arrays(data[ax], k1)
            s2, w2 = arrays(data[ax], swap_method(k1, m2))
            fam1.append({"family": "축 안 방식 비교", "axis": ax, "compare": f"{m1} - {m2}", **_test(s1, s2, w1, w2)})
    for m in METHODS:
        for ax in ("x", "y"):
            s1, w1 = arrays(data[ax], common[m])
            s0, w0 = arrays(data["z"], common[m])
            fam2.append({"family": "방식 안 축 비교", "axis": f"{ax} - z", "compare": m, **_test(s1, s0, w1, w0)})
    for fam in (fam1, fam2):
        for col in ("mcnemar_p", "wilcoxon_p"):
            adj = holm([r[col] for r in fam])
            for r, a in zip(fam, adj):
                r[col + "_holm"] = a
    return fam1 + fam2


def intermediate_axis_growth(spin_deg_s: float) -> float:
    """
    중간 관성축(x) 회전의 섭동 성장률 [1/s] (강체 오일러 방정식의 선형화).
    lambda = omega * sqrt((I_max - I_mid)(I_mid - I_min) / (I_max * I_min))
    """
    i_max, i_mid, i_min = INERTIA["z"], INERTIA["x"], INERTIA["y"]
    return math.radians(spin_deg_s) * math.sqrt((i_max - i_mid) * (i_mid - i_min) / (i_max * i_min))


# =============================================================================
# 그림
# =============================================================================
def plot_rates(summary: List[Dict], path: str):
    plt = _setup_matplotlib()
    fig, ax = plt.subplots(figsize=(7.4, 3.9), layout="constrained")
    width = 0.26
    for j, m in enumerate(METHODS):
        rows = [next(r for r in summary if r["axis"] == a and r["method"] == m) for a in AXES]
        xs = np.arange(len(AXES)) + (j - 1) * width
        rates = np.array([r["success_rate"] for r in rows]) * 100
        err = np.array([[r["success_rate"] - r["ci_low"] for r in rows],
                        [r["ci_high"] - r["success_rate"] for r in rows]]) * 100
        ax.bar(xs, rates, width * 0.92, color=METHOD_COLORS[m], label=METHOD_LABELS[m], zorder=2)
        ax.errorbar(xs, rates, yerr=err, fmt="none", ecolor="#0b0b0b", elinewidth=0.9, capsize=2.5, zorder=3)
        for x, v, e in zip(xs, rates, err[1]):
            ax.text(x, v + e + 1.5, f"{v:.0f}", ha="center", va="bottom", fontsize=8.5, color="#0b0b0b")
    ax.set_xticks(np.arange(len(AXES)), [AXIS_SHORT[a] for a in AXES])
    ax.set_ylim(0, 105)
    ax.set_ylabel("성공률 [%]")
    ax.grid(axis="y", zorder=0)
    n = summary[0]["n"] if summary else 0
    ax.set_title(f"회전축별 포획 성공률 (중형 500 kg, 2~30 deg/s, 축·방식당 n={n})", loc="left", fontsize=11)
    ax.legend(loc="upper right", frameon=False, ncol=3)
    fig.savefig(path, dpi=200)
    plt.close(fig)


def plot_curves(data: Dict[str, Dict[Key, Dict]], common: Dict[str, List[Key]], path: str):
    plt = _setup_matplotlib()
    fig, axes = plt.subplots(1, len(AXES), figsize=(10.5, 3.5), sharey=True, layout="constrained")
    for ax_plot, ax in zip(axes, AXES):
        for m in METHODS:
            spins = sorted({k[1] for k in common[m]})
            rate, lo, hi = [], [], []
            for sp in spins:
                ks = [k for k in common[m] if k[1] == sp]
                s, _ = arrays(data[ax], ks)
                a, b = wilson_interval(int(s.sum()), len(s))
                rate.append(s.mean() * 100); lo.append(a * 100); hi.append(b * 100)
            ax_plot.plot(spins, rate, color=METHOD_COLORS[m], lw=1.8, marker="o", ms=3, label=METHOD_LABELS[m])
            ax_plot.fill_between(spins, lo, hi, color=METHOD_COLORS[m], alpha=0.14, lw=0)
        ax_plot.set_title(AXIS_SHORT[ax], loc="left", fontsize=11)
        ax_plot.set_xlabel("회전 속도 [deg/s]")
        ax_plot.set_xlim(0, 31)
        ax_plot.set_ylim(-3, 103)
        ax_plot.grid(True)
    axes[0].set_ylabel("성공률 [%]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=3, frameon=False)
    fig.suptitle("회전축별 회전 속도에 따른 포획 성공률 (중형 500 kg, 음영 = Wilson 95 % CI)",
                 x=0.01, ha="left", fontsize=12)
    fig.savefig(path, dpi=200)
    plt.close(fig)


# =============================================================================
# 보고서
# =============================================================================
def pct(v: float) -> str:
    return f"{v * 100:.1f}"


def fmt_p(p: float) -> str:
    return "<10⁻⁴" if p < 1e-4 else f"{p:.4f}" if p < 0.01 else f"{p:.3f}"


def p_sentence(p: float) -> str:
    return "p < 10⁻⁴" if p < 1e-4 else f"p = {fmt_p(p)}"


def write_report(summary: List[Dict], tests: List[Dict], completeness: Dict[str, str],
                 n_exc: Dict[str, int], path: str) -> List[str]:
    L = ["# 회전축 추가 실험 결과 (중형 500 kg)", "",
         "자동 생성: `python analyze_axis.py`. 세 축에 모두 있는 (방식, 회전 속도, 시드) 시행끼리만 비교한다.", "",
         "## 자료", ""]
    for ax in AXES:
        L.append(f"- {AXIS_LABELS[ax]}: {completeness[ax]}, 예외 종료 {n_exc[ax]} 회")
    L += ["", "## 축 × 방식 성공률 (2~30 deg/s)", "",
          "| 회전축 | " + " | ".join(METHOD_LABELS[m] for m in METHODS) + " | 가장 유효 |",
          "|---|" + "---|" * (len(METHODS) + 1)]
    best = {}
    for ax in AXES:
        rows = {r["method"]: r for r in summary if r["axis"] == ax}
        top = max(METHODS, key=lambda m: rows[m]["success_rate"])
        best[ax] = top
        cells = []
        for m in METHODS:
            r = rows[m]
            c = f"{pct(r['success_rate'])} [{pct(r['ci_low'])}, {pct(r['ci_high'])}]"
            cells.append(f"**{c}**" if m == top else c)
        L.append(f"| {AXIS_SHORT[ax]} | " + " | ".join(cells) + f" | {METHOD_LABELS[top]} |")
    n = summary[0]["n"] if summary else 0
    L += ["", f"단위 %, 대괄호는 Wilson 95 % 신뢰구간. 축·방식당 n = {n}.", "",
          "## 축 안 방식 비교 (짝지은 검정, Holm 보정)", "",
          "| 회전축 | 비교 | 성공률 차 [95 % CI] | McNemar p (Holm) | 잔류 ω 중앙값 차 [deg/s] | Wilcoxon p (Holm) |",
          "|---|---|---|---|---|---|"]
    for r in tests:
        if r["family"] != "축 안 방식 비교":
            continue
        m1, m2 = r["compare"].split(" - ")
        L.append(f"| {AXIS_SHORT[r['axis']]} | {METHOD_LABELS[m1]} − {METHOD_LABELS[m2]} | "
                 f"{r['diff'] * 100:+.1f} %p [{r['diff_ci_low'] * 100:+.1f}, {r['diff_ci_high'] * 100:+.1f}] | "
                 f"{fmt_p(r['mcnemar_p_holm'])} {stars(r['mcnemar_p_holm'])} | {r['median_omega_diff']:+.2f} | "
                 f"{fmt_p(r['wilcoxon_p_holm'])} {stars(r['wilcoxon_p_holm'])} |")
    L += ["", "잔류 ω 차가 음수면 앞 방식이 회전을 더 많이 줄였다는 뜻이다.", "",
          "## 방식 안 축 비교 (z 축 대비, 짝지은 검정, Holm 보정)", "",
          "| 방식 | 비교 | 성공률 차 [95 % CI] | McNemar p (Holm) | 잔류 ω 중앙값 차 [deg/s] | Wilcoxon p (Holm) |",
          "|---|---|---|---|---|---|"]
    for r in tests:
        if r["family"] != "방식 안 축 비교":
            continue
        L.append(f"| {METHOD_LABELS[r['compare']]} | {r['axis']} | "
                 f"{r['diff'] * 100:+.1f} %p [{r['diff_ci_low'] * 100:+.1f}, {r['diff_ci_high'] * 100:+.1f}] | "
                 f"{fmt_p(r['mcnemar_p_holm'])} {stars(r['mcnemar_p_holm'])} | {r['median_omega_diff']:+.2f} | "
                 f"{fmt_p(r['wilcoxon_p_holm'])} {stars(r['wilcoxon_p_holm'])} |")
    L += ["", "## 축마다 가장 유효한 방식과 2위의 차이", ""]
    for ax in AXES:
        rows = sorted((r for r in summary if r["axis"] == ax), key=lambda r: -r["success_rate"])
        m1, m2 = rows[0]["method"], rows[1]["method"]
        t = next(r for r in tests if r["family"] == "축 안 방식 비교" and r["axis"] == ax
                 and set(r["compare"].split(" - ")) == {m1, m2})
        verdict = "유의" if t["mcnemar_p_holm"] < 0.05 else "유의하지 않음"
        L.append(f"- {AXIS_SHORT[ax]}: {METHOD_LABELS[m1]} {pct(rows[0]['success_rate'])} % vs "
                 f"{METHOD_LABELS[m2]} {pct(rows[1]['success_rate'])} % — 성공률 차 McNemar (Holm) "
                 f"{p_sentence(t['mcnemar_p_holm'])}, {verdict}")
    lam10, lam30 = intermediate_axis_growth(10.0), intermediate_axis_growth(30.0)
    L += ["", "## 참고: 중간 관성축(x) 회전의 불안정성", "",
          f"주관성모멘트 x {INERTIA['x']}, y {INERTIA['y']}, z {INERTIA['z']} kg·m² 에서 x 축은 중간 관성축이다. "
          "강체 오일러 방정식을 선형화하면 x 축 회전에 대한 작은 섭동이 "
          "λ = ω·√((I_z − I_x)(I_x − I_y)/(I_z·I_y)) 로 지수 성장한다. "
          f"10 deg/s 에서 e배 시간 {1 / lam10:.1f} s, 30 deg/s 에서 {1 / lam30:.1f} s 이다. "
          "초기 회전축 오차(표준편차 10°)가 있으므로, x 축 조건의 대상은 포획 시간 동안 순수한 x 축 회전을 "
          "유지하지 못하고 점차 뒤집히는 회전으로 바뀐다. 이는 모델의 결함이 아니라 강체 역학의 결과다.", ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    return L


def write_csv(rows: List[Dict], path: str):
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)


# =============================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description="회전축 추가 실험 분석 (중형 대상)")
    ap.add_argument("--z-results", default=os.path.join("results", "results_raw.csv"),
                    help="본 격자 실험 원자료 (z 축 회전)")
    ap.add_argument("--axis-dir", default="results_axis", help="x/, y/ 하위 폴더에 results_raw.csv 가 있는 폴더")
    ap.add_argument("--out", default="results_axis")
    args = ap.parse_args(argv)
    # 한글 윈도 콘솔(cp949)에 없는 기호(√, ⁻ 등)가 있어도 출력 때문에 죽지 않게
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")

    data, n_exc = {}, {}
    data["z"], n_exc["z"] = load_axis(args.z_results)
    for ax in ("x", "y"):
        data[ax], n_exc[ax] = load_axis(os.path.join(args.axis_dir, ax, "results_raw.csv"))

    # 세 축에 모두 있는 시행만
    common = {}
    for m in METHODS:
        keys = set(k for k in data["z"] if k[0] == m)
        for ax in ("x", "y"):
            keys &= set(k for k in data[ax] if k[0] == m)
        common[m] = sorted(keys, key=lambda k: (k[1], k[2]))
    if not all(common[m] for m in METHODS):
        raise SystemExit("세 축에 공통으로 있는 시행이 아직 없는 방식이 있다. 실험이 더 진행된 뒤 다시 실행할 것.")

    completeness = {}
    for ax in AXES:
        per = {}
        for (m, s, seed) in data[ax]:
            per.setdefault((m, s), set()).add(seed)
        reps = [len(v) for v in per.values()] or [0]
        completeness[ax] = f"시행 {len(data[ax])} 회 (방식·회전속도 조합당 반복 {min(reps)}~{max(reps)} 회)"

    summary = summarize(data, common)
    tests = run_tests(data, common)
    os.makedirs(args.out, exist_ok=True)
    write_csv(summary, os.path.join(args.out, "axis_summary.csv"))
    write_csv(tests, os.path.join(args.out, "axis_tests.csv"))
    lines = write_report(summary, tests, completeness, n_exc, os.path.join(args.out, "axis_report.md"))
    plot_rates(summary, os.path.join(args.out, "fig_axis_rates.png"))
    plot_curves(data, common, os.path.join(args.out, "fig_axis_curves.png"))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
