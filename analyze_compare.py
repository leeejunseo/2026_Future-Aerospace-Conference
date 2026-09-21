#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
방식 간 짝지은 통계 비교
==========================

격자 실험은 같은 반복 번호에 모든 조합이 같은 시드를 쓰도록 설계되어 있다(공통 난수).
따라서 같은 (대상, 회전 속도, 시드)에서 나온 두 방식의 결과는 **짝지어진 관측**이고,
짝지은 검정을 쓰면 시드에서 오는 변동이 상쇄되어 훨씬 작은 표본으로도 차이를 가릴 수 있다.

조합별 신뢰구간만으로는 "로봇팔이 그물보다 유의하게 낫다"를 말할 수 없다. 이 스크립트가 그 부분을 채운다.

수행하는 검정
    1. 성공 여부 (이진)    : McNemar 정확검정. 불일치 쌍만 쓰는 짝지은 검정이다.
    2. 성공률 차이 (효과크기): 짝지은 부트스트랩 95 % 신뢰구간
    3. 잔류 상대 각속도     : Wilcoxon 부호순위 검정. 성공/실패로 나뉘지 않는 연속 지표라 검정력이 높다.

다중 비교
    방식쌍 3개 x 대상 3개로 검정이 여러 번이므로 Holm-Bonferroni 로 보정한 p 값을 함께 낸다.
    회전 속도별 표는 탐색용이며, 결론은 대상별 통합 결과로 말하는 것이 안전하다.

실행
    conda activate debris-sim
    python analyze_compare.py                       # results/ 를 읽어 comparison/ 에 출력
    python analyze_compare.py --results results --out comparison
"""

from __future__ import annotations

import argparse
import csv
import itertools
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import stats

METHOD_LABELS = {"arm": "로봇팔", "net": "그물", "harpoon": "작살"}
TARGET_LABELS = {"small": "소형 10 kg", "medium": "중형 500 kg", "large": "대형 2000 kg"}
METHODS = ["arm", "net", "harpoon"]
TARGETS = ["small", "medium", "large"]


# =============================================================================
# 자료 읽기
# =============================================================================
def load(path: str) -> List[Dict]:
    if not os.path.exists(path):
        raise SystemExit(f"원자료가 없다: {path}\n먼저 `bash run_all.sh 11 grid` 로 격자 실험을 돌릴 것.")
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(v, default=float("nan")) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def index_rows(rows: List[Dict]) -> Dict[Tuple[str, str, float, int], Dict]:
    """(방식, 대상, 회전속도, 시드) -> 시행"""
    out = {}
    for r in rows:
        out[(r["method"], r["target"], _f(r["spin_deg_s"]), int(_f(r["seed"], -1)))] = r
    return out


def paired(idx: Dict, m1: str, m2: str, target: str,
           spin: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """두 방식의 짝지은 관측을 (성공1, 성공2, 잔류각속도1, 잔류각속도2) 로 반환"""
    keys = [k for k in idx if k[0] == m1 and k[1] == target and (spin is None or k[2] == spin)]
    s1, s2, w1, w2 = [], [], [], []
    for k in keys:
        other = (m2, k[1], k[2], k[3])
        if other not in idx:
            continue
        a, b = idx[k], idx[other]
        s1.append(str(a["success"]).lower() == "true")
        s2.append(str(b["success"]).lower() == "true")
        w1.append(_f(a["final_rel_omega_deg_s"]))
        w2.append(_f(b["final_rel_omega_deg_s"]))
    return np.array(s1), np.array(s2), np.array(w1), np.array(w2)


# =============================================================================
# 검정
# =============================================================================
def mcnemar_exact(s1: np.ndarray, s2: np.ndarray) -> Tuple[int, int, float]:
    """
    McNemar 정확검정. b = m1 만 성공한 쌍, c = m2 만 성공한 쌍.
    두 방식이 같다면 불일치 쌍이 b/(b+c) ~ Binomial(n, 0.5) 를 따른다.
    """
    b = int(np.sum(s1 & ~s2))
    c = int(np.sum(~s1 & s2))
    if b + c == 0:
        return b, c, 1.0
    p = stats.binomtest(b, b + c, 0.5, alternative="two-sided").pvalue
    return b, c, float(p)


def paired_bootstrap_diff(s1: np.ndarray, s2: np.ndarray, n_boot: int = 10000,
                          seed: int = 0) -> Tuple[float, float, float]:
    """성공률 차이(m1 - m2)의 짝지은 부트스트랩 95 % 신뢰구간"""
    n = len(s1)
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    diff = float(s1.mean() - s2.mean())
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(n_boot, n))       # 쌍 단위로 재표집
    boots = s1[idx].mean(axis=1) - s2[idx].mean(axis=1)
    return diff, float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))


def wilcoxon_rel_omega(w1: np.ndarray, w2: np.ndarray) -> Tuple[float, float]:
    """잔류 상대 각속도의 Wilcoxon 부호순위 검정. (중앙값 차이, p)"""
    ok = np.isfinite(w1) & np.isfinite(w2)
    a, b = w1[ok], w2[ok]
    med = float(np.median(a - b))
    if len(a) < 5 or np.allclose(a, b):
        return med, 1.0
    try:
        return med, float(stats.wilcoxon(a, b, zero_method="wilcox").pvalue)
    except ValueError:
        return med, 1.0


def holm(pvals: List[float]) -> List[float]:
    """Holm-Bonferroni 보정. 순서를 유지한 보정 p 값을 반환한다."""
    n = len(pvals)
    order = sorted(range(n), key=lambda i: pvals[i])
    adj = [0.0] * n
    running = 0.0
    for rank, i in enumerate(order):
        val = (n - rank) * pvals[i]
        running = max(running, val)
        adj[i] = min(1.0, running)
    return adj


def stars(p: float) -> str:
    return "***" if p < 0.001 else "**" if p < 0.01 else "*" if p < 0.05 else "n.s."


# =============================================================================
# 분석
# =============================================================================
def compare_by_target(idx: Dict) -> List[Dict]:
    """대상별로 모든 회전 속도를 통합한 짝지은 비교 (주 결과)"""
    out = []
    for target in TARGETS:
        for m1, m2 in itertools.combinations(METHODS, 2):
            s1, s2, w1, w2 = paired(idx, m1, m2, target)
            if len(s1) == 0:
                continue
            b, c, p_mc = mcnemar_exact(s1, s2)
            d, lo, hi = paired_bootstrap_diff(s1, s2)
            med, p_w = wilcoxon_rel_omega(w1, w2)
            out.append({"target": target, "method_1": m1, "method_2": m2, "n_pairs": len(s1),
                        "success_rate_1": float(s1.mean()), "success_rate_2": float(s2.mean()),
                        "diff": d, "diff_ci_low": lo, "diff_ci_high": hi,
                        "mcnemar_b": b, "mcnemar_c": c, "mcnemar_p": p_mc,
                        "rel_omega_median_diff": med, "wilcoxon_p": p_w})
    for key, field in (("mcnemar_p", "mcnemar_p_holm"), ("wilcoxon_p", "wilcoxon_p_holm")):
        adj = holm([r[key] for r in out])
        for r, a in zip(out, adj):
            r[field] = a
    return out


def compare_by_spin(idx: Dict) -> List[Dict]:
    """회전 속도별 비교 (탐색용). 검정 수가 많아 보정 p 값이 보수적으로 나온다."""
    spins = sorted({k[2] for k in idx})
    out = []
    for target in TARGETS:
        for spin in spins:
            for m1, m2 in itertools.combinations(METHODS, 2):
                s1, s2, w1, w2 = paired(idx, m1, m2, target, spin)
                if len(s1) == 0:
                    continue
                b, c, p_mc = mcnemar_exact(s1, s2)
                d, lo, hi = paired_bootstrap_diff(s1, s2)
                med, p_w = wilcoxon_rel_omega(w1, w2)
                out.append({"target": target, "spin_deg_s": spin, "method_1": m1, "method_2": m2,
                            "n_pairs": len(s1), "success_rate_1": float(s1.mean()),
                            "success_rate_2": float(s2.mean()), "diff": d,
                            "diff_ci_low": lo, "diff_ci_high": hi,
                            "mcnemar_p": p_mc, "rel_omega_median_diff": med, "wilcoxon_p": p_w})
    if out:
        adj = holm([r["mcnemar_p"] for r in out])
        for r, a in zip(out, adj):
            r["mcnemar_p_holm"] = a
    return out


def rank_methods(idx: Dict) -> List[Dict]:
    """대상별로 방식을 성공률 순으로 세우고, 1위가 2위보다 유의한지 표시한다."""
    out = []
    for target in TARGETS:
        rates = {}
        for m in METHODS:
            keys = [k for k in idx if k[0] == m and k[1] == target]
            if keys:
                rates[m] = float(np.mean([str(idx[k]["success"]).lower() == "true" for k in keys]))
        if len(rates) < 2:
            continue
        order = sorted(rates, key=lambda m: -rates[m])
        best, second = order[0], order[1]
        s1, s2, w1, w2 = paired(idx, best, second, target)
        _, _, p = mcnemar_exact(s1, s2)
        d, lo, hi = paired_bootstrap_diff(s1, s2)
        out.append({"target": target, "rank": " > ".join(METHOD_LABELS[m] for m in order),
                    "best": best, "best_rate": rates[best], "second": second, "second_rate": rates[second],
                    "diff": d, "diff_ci_low": lo, "diff_ci_high": hi, "p": p,
                    "significant": bool(p < 0.05)})
    return out


# =============================================================================
# 출력
# =============================================================================
def write_csv(rows: List[Dict], path: str):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def print_report(by_target: List[Dict], ranking: List[Dict]) -> List[str]:
    L: List[str] = []

    def log(s=""):
        print(s)
        L.append(s)

    log("=" * 100)
    log("대상별 방식 순위 (전 회전 속도 통합)")
    log("=" * 100)
    log(f"{'대상':14s} {'순위':28s} {'1위':8s} {'2위':8s} {'차이 [%p]':>16s} {'p':>9s}")
    for r in ranking:
        ci = f"{100*r['diff']:+.1f} [{100*r['diff_ci_low']:+.1f}, {100*r['diff_ci_high']:+.1f}]"
        log(f"{TARGET_LABELS[r['target']]:14s} {r['rank']:28s} "
            f"{100*r['best_rate']:6.1f}% {100*r['second_rate']:6.1f}% {ci:>16s} "
            f"{r['p']:9.2e} {stars(r['p'])}")
    log()
    log("=" * 100)
    log("짝지은 방식 비교 (McNemar 정확검정 · 성공률 차이 부트스트랩 · Wilcoxon 잔류 각속도)")
    log("Holm-Bonferroni 로 다중 비교를 보정한 p 값을 쓴다")
    log("=" * 100)
    log(f"{'대상':14s} {'비교':22s} {'n':>5s} {'성공률 차이 [%p] (95 % CI)':>30s} {'McNemar p':>12s} "
        f"{'잔류ω 차이':>12s} {'Wilcoxon p':>12s}")
    for r in by_target:
        name = f"{METHOD_LABELS[r['method_1']]} vs {METHOD_LABELS[r['method_2']]}"
        ci = f"{100*r['diff']:+.1f} [{100*r['diff_ci_low']:+.1f}, {100*r['diff_ci_high']:+.1f}]"
        log(f"{TARGET_LABELS[r['target']]:14s} {name:22s} {r['n_pairs']:5d} {ci:>30s} "
            f"{r['mcnemar_p_holm']:9.2e} {stars(r['mcnemar_p_holm']):>4s} "
            f"{r['rel_omega_median_diff']:+11.3f} {r['wilcoxon_p_holm']:9.2e} {stars(r['wilcoxon_p_holm']):>4s}")
    log()
    log("읽는 법")
    log("  성공률 차이는 (앞 방식 - 뒤 방식) 이다. 신뢰구간이 0 을 포함하지 않으면 차이가 유의하다.")
    log("  잔류ω 차이는 짝별 차이의 중앙값이며 음수이면 앞 방식의 감속이 더 잘 된 것이다.")
    log("  McNemar 는 성공 여부만 보고, Wilcoxon 은 연속 지표라 성공에 못 미친 차이까지 잡아낸다.")
    log("  n.s. 는 유의하지 않음을 뜻한다.")
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description="방식 간 짝지은 통계 비교")
    ap.add_argument("--results", default="results")
    ap.add_argument("--out", default="comparison")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    rows = load(os.path.join(args.results, "results_raw.csv"))
    idx = index_rows(rows)
    n_seeds = len({k[3] for k in idx})
    print(f"시행 {len(rows)}개, 시드 {n_seeds}개를 읽었다.\n")

    by_target = compare_by_target(idx)
    by_spin = compare_by_spin(idx)
    ranking = rank_methods(idx)

    write_csv(by_target, os.path.join(args.out, "compare_by_target.csv"))
    write_csv(by_spin, os.path.join(args.out, "compare_by_spin.csv"))
    write_csv(ranking, os.path.join(args.out, "method_ranking.csv"))

    lines = print_report(by_target, ranking)
    with open(os.path.join(args.out, "comparison_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n출력 -> {args.out}/ (compare_by_target.csv, compare_by_spin.csv, "
          f"method_ranking.csv, comparison_report.txt)")


if __name__ == "__main__":
    main()
