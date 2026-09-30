#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
결과 보고서 생성
==================

results/ · verification/ · sensitivity/ 의 산출물을 읽어 논문에 옮겨 쓸 수 있는
표와 수치를 하나의 텍스트 파일로 정리한다. 원고 템플릿(한글 파일)에 붙여넣기 좋게
표는 탭 구분과 마크다운 두 가지로 낸다.

가정값 표는 결과가 없어도 언제든 생성된다 (capture_sim.CONFIG 를 직접 읽는다).

실행
    conda activate debris-sim
    python make_report.py                      # report/report.md, report/assumptions.tsv
    python make_report.py --only assumptions   # 가정값 표만
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from typing import Dict, List, Optional

import capture_sim as cs

TARGET_LABELS = {"small": "소형 (10 kg, 정육면체)", "medium": "중형 (500 kg, 본체+태양전지판)",
                 "large": "대형 (2000 kg, 원통)"}
METHOD_LABELS = {"arm": "로봇팔", "net": "그물", "harpoon": "작살"}


# =============================================================================
# 가정값 표 (CONFIG 에서 직접 추출)
# =============================================================================
def collect_assumptions(path: str = "capture_sim.py") -> List[Dict]:
    """CONFIG 블록에서 `키: 값,  # 주석` 을 훑어 가정값과 출처 확인 필요 여부를 뽑는다."""
    src = open(path, encoding="utf-8").read()
    start = src.index("CONFIG: Dict = {")
    end = src.index("\n# 충돌 필터 그룹", start)
    block = src[start:end]
    rows: List[Dict] = []
    section = ""
    pending_note: List[str] = []
    for line in block.splitlines():
        stripped = line.strip()
        sec = re.match(r"#\s*-{3,}\s*(.+)$", stripped)
        if sec:
            section = sec.group(1).strip()
            pending_note = []
            continue
        if stripped.startswith("#"):
            pending_note.append(stripped.lstrip("# ").strip())
            continue
        m = re.match(r'"([a-z_0-9]+)"\s*:\s*(.+?),\s*(?:#\s*(.*))?$', stripped)
        if not m:
            if stripped and not stripped.startswith(('"', "}")):
                pending_note = []
            continue
        key, value, note = m.group(1), m.group(2).strip(), (m.group(3) or "").strip()
        if value.startswith("{") and not value.endswith("}"):
            pending_note = []
            continue
        full_note = " ".join([*pending_note, note]).strip()
        pending_note = []
        needs_source = "TODO" in full_note
        rows.append({"section": section, "key": key, "value": value,
                     "note": full_note.replace("TODO: 출처 확인 필요", "").strip(" ·"),
                     "needs_source": needs_source})
    return rows


# =============================================================================
# CSV 읽기
# =============================================================================
def read_csv(path: str) -> List[Dict]:
    # utf-8-sig 로 읽으면 엑셀용으로 BOM 을 붙여 저장한 파일도 첫 열 이름이 깨지지 않는다.
    # BOM 이 없는 파일도 그대로 읽힌다.
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def _f(v, default=float("nan")) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def md_table(header: List[str], rows: List[List[str]]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out)


# =============================================================================
# 결과 요약
# =============================================================================
def summarize_results(summary: List[Dict]) -> List[str]:
    if not summary:
        return ["> 격자 실험 결과가 아직 없다. `bash run_all.sh` 로 생성할 것."]
    out: List[str] = []
    methods = ["arm", "net", "harpoon"]
    targets = ["small", "medium", "large"]
    spins = sorted({_f(r["spin_deg_s"]) for r in summary})
    n = int(_f(summary[0]["n"], 0))
    tbl = {(r["method"], r["target"], _f(r["spin_deg_s"])): r for r in summary}

    out.append(f"시행 수는 조합당 n={n}, 회전 속도는 {min(spins):.0f}~{max(spins):.0f} deg/s 를 "
               f"{len(spins)}개 수준으로 훑었다.\n")

    # --- 표 1: 방식 x 질량 종합
    out.append("### 표 1. 방식과 질량별 종합 (전 회전 속도 통합)\n")
    rows = []
    for m in methods:
        for t in targets:
            g = [tbl[(m, t, s)] for s in spins if (m, t, s) in tbl]
            if not g:
                continue
            tot = sum(int(_f(r["n"], 0)) for r in g)
            suc = sum(int(_f(r["successes"], 0)) for r in g)
            ct = [_f(r["capture_time_median_s"]) for r in g if _f(r["capture_time_median_s"]) == _f(r["capture_time_median_s"])]
            rel = [_f(r["final_rel_omega_median_deg_s"]) for r in g]
            rel = [v for v in rel if v == v]
            imp = [_f(r["impulse_mean_Ns"]) for r in g]
            imp = [v for v in imp if v == v]
            att = [_f(r["servicer_attitude_error_mean_deg"]) for r in g]
            att = [v for v in att if v == v]
            rows.append([METHOD_LABELS[m], TARGET_LABELS[t].split(" (")[0],
                         f"{100 * suc / tot:.0f}% ({suc}/{tot})",
                         f"{sorted(ct)[len(ct) // 2]:.0f}" if ct else "-",
                         f"{sorted(rel)[len(rel) // 2]:.2f}" if rel else "-",
                         f"{sum(imp) / len(imp):.1f}" if imp else "-",
                         f"{sum(att) / len(att):.0f}" if att else "-"])
    out.append(md_table(["포획 방식", "대상", "성공률", "감속 시간 [s]", "잔류 상대 각속도 [deg/s]",
                         "최대 충격량 [N·s]", "서비서 자세 변화 [deg]"], rows))
    out.append("")

    # --- 표 2: 성공률 50 % 이상을 유지하는 회전 속도 구간
    # "처음으로 50 % 아래로 떨어지는 속도" 는 성공률이 회전 속도에 대해 단조 감소할 때만 의미가 있다.
    # 실제로는 중형 그물처럼 회전이 빠를수록 오르거나, 소형 로봇팔처럼 골짜기가 있는 곡선이 나오므로
    # 50 % 이상인 구간을 모두 나열한다.
    out.append("### 표 2. 성공률 50 % 이상을 유지하는 회전 속도 구간\n")
    rows = []
    for m in methods:
        for t in targets:
            ok = [s for s in spins if (m, t, s) in tbl and _f(tbl[(m, t, s)]["success_rate"]) >= 0.5]
            best = max((_f(tbl[(m, t, s)]["success_rate"]) for s in spins if (m, t, s) in tbl), default=0.0)
            rows.append([METHOD_LABELS[m], TARGET_LABELS[t].split(" (")[0],
                         _ranges(ok, spins) if ok else "없음",
                         f"{100 * best:.0f}%"])
    out.append(md_table(["포획 방식", "대상", "성공률 50 % 이상 구간 [deg/s]", "최고 성공률"], rows))
    out.append("")
    return out


def _ranges(values: List[float], grid: List[float]) -> str:
    """격자 위에서 연속한 값들을 '0~12, 16~30' 처럼 구간으로 묶는다 (한국어 논문의 범위 표기)."""
    idx = sorted(grid.index(v) for v in values)
    parts, start = [], idx[0]
    for a, b in zip(idx, idx[1:] + [None]):
        if b is None or b != a + 1:
            lo, hi = grid[start], grid[a]
            parts.append(f"{lo:.0f}" if lo == hi else f"{lo:.0f}~{hi:.0f}")
            start = b
    return ", ".join(parts)


def summarize_sensitivity(summary: List[Dict]) -> List[str]:
    if not summary:
        return ["> 민감도 분석 결과가 아직 없다. `bash run_all.sh 11 sens` 로 생성할 것."]
    base = {r["method"]: r for r in summary if r["param"] == "(기준)"}
    agg: Dict = {}
    for r in summary:
        if r["param"] == "(기준)":
            continue
        agg.setdefault((r["method"], r["param"]), {})[r["level"]] = r
    rows = []
    for (m, pname), lv in agg.items():
        b = base.get(m)
        if b is None or "low" not in lv or "high" not in lv:
            continue
        d_succ = 100 * abs(_f(lv["high"]["success_rate"]) - _f(lv["low"]["success_rate"]))
        d_rel = abs(_f(lv["high"]["final_rel_omega_median_deg_s"]) - _f(lv["low"]["final_rel_omega_median_deg_s"]))
        rows.append((d_rel if d_rel == d_rel else 0.0, d_succ, m, pname,
                     _f(lv["low"]["final_rel_omega_median_deg_s"]),
                     _f(lv["high"]["final_rel_omega_median_deg_s"])))
    rows.sort(reverse=True)
    out = ["민감도는 각 가정값을 낮음/높음으로 바꿔 돌린 One-At-a-Time 결과다.",
           "잔류 상대 각속도 변화 폭이 큰 순서로 정렬했다. 폭이 큰 값일수록 논문에서 출처를 밝히거나",
           "별도 논의가 필요하다.\n"]
    out.append("### 표 3. 가정값 민감도 상위 15개\n")
    out.append(md_table(["포획 방식", "가정값", "잔류 각속도 변화 폭 [deg/s]", "성공률 변화 폭 [%p]",
                         "낮음 → 높음 [deg/s]"],
                        [[METHOD_LABELS.get(m, m), pn, f"{dr:.2f}", f"{ds:.0f}", f"{lo:.2f} → {hi:.2f}"]
                         for dr, ds, m, pn, lo, hi in rows[:15]]))
    out.append("")
    return out


def summarize_comparison(by_target: List[Dict], ranking: List[Dict]) -> List[str]:
    if not by_target:
        return ["> 방식 간 비교 결과가 아직 없다. `python analyze_compare.py` 로 생성할 것."]
    out = ["같은 (대상, 회전 속도, 시드)에서 나온 두 방식의 결과는 짝지어진 관측이므로 짝지은 검정을 쓴다.",
           "성공 여부는 McNemar 정확검정, 성공률 차이는 짝지은 부트스트랩, "
           "잔류 상대 각속도는 Wilcoxon 부호순위 검정이다.",
           "방식쌍과 대상에 걸쳐 검정이 여러 번이므로 Holm-Bonferroni 로 보정한 p 값을 쓴다.\n"]
    out.append("### 표 4. 대상별 방식 순위\n")
    out.append(md_table(["대상", "순위", "1위 성공률", "2위 성공률", "차이 [%p] (95 % CI)", "p", "유의성"],
                        [[TARGET_LABELS.get(r["target"], r["target"]).split(" (")[0], r["rank"],
                          f"{100 * _f(r['best_rate']):.1f}%", f"{100 * _f(r['second_rate']):.1f}%",
                          f"{100 * _f(r['diff']):+.1f} [{100 * _f(r['diff_ci_low']):+.1f}, "
                          f"{100 * _f(r['diff_ci_high']):+.1f}]",
                          f"{_f(r['p']):.1e}",
                          "유의" if str(r["significant"]).lower() == "true" else "유의하지 않음"]
                         for r in ranking]))
    out.append("")
    out.append("### 표 5. 방식쌍별 짝지은 비교\n")
    rows = []
    for r in by_target:
        rows.append([TARGET_LABELS.get(r["target"], r["target"]).split(" (")[0],
                     f"{METHOD_LABELS.get(r['method_1'], r['method_1'])} vs "
                     f"{METHOD_LABELS.get(r['method_2'], r['method_2'])}",
                     str(int(_f(r["n_pairs"], 0))),
                     f"{100 * _f(r['diff']):+.1f} [{100 * _f(r['diff_ci_low']):+.1f}, "
                     f"{100 * _f(r['diff_ci_high']):+.1f}]",
                     f"{_f(r['mcnemar_p_holm']):.1e}",
                     f"{_f(r['rel_omega_median_diff']):+.2f}",
                     f"{_f(r['wilcoxon_p_holm']):.1e}"])
    out.append(md_table(["대상", "비교", "쌍 수", "성공률 차이 [%p] (95 % CI)", "McNemar p",
                         "잔류 각속도 차이 [deg/s]", "Wilcoxon p"], rows))
    out.append("")
    out.append("성공률 차이는 (앞 방식 − 뒤 방식)이며 신뢰구간이 0 을 포함하지 않으면 유의하다. "
               "잔류 각속도 차이는 짝별 차이의 중앙값이고 음수이면 앞 방식의 감속이 더 잘 된 것이다.\n")
    return out


def summarize_verification(path: str) -> List[str]:
    if not os.path.exists(path):
        return ["> 수치 검증 결과가 아직 없다. `python run_verification.py` 로 생성할 것."]
    txt = open(path, encoding="utf-8").read()
    out = ["검증 보고서 전문은 `verification/verification_report.txt` 에 있다. 요지만 옮기면 다음과 같다.\n"]
    m = re.search(r"상대 오차\s*:\s*([0-9.e+-]+)", txt)
    if m:
        out.append(f"- 토크-프리 회전을 오일러 방정식 RK4 기준해와 비교한 각속도 상대 오차는 {m.group(1)} 이다.")
    m = re.search(r"각운동량 상대 오차\s*:\s*([0-9.e+-]+)", txt)
    if m:
        out.append(f"- 같은 시행에서 각운동량 상대 오차는 {m.group(1)} 로, 감쇠 없이 보존된다.")
    if "성공일치" in txt or "success일치" in txt:
        out.append("- 시간 스텝을 1/240, 1/480, 1/960 로 바꾼 수렴성 시험 결과는 검증 보고서의 표에 있다.")
    out.append("")
    return out


# =============================================================================
# 보고서 작성
# =============================================================================
def build_report(res_dir: str, ver_dir: str, sens_dir: str, cmp_dir: str = "comparison") -> str:
    summary = read_csv(os.path.join(res_dir, "results_summary.csv"))
    sens = read_csv(os.path.join(sens_dir, "sensitivity_summary.csv"))
    cmp_by_target = read_csv(os.path.join(cmp_dir, "compare_by_target.csv"))
    cmp_ranking = read_csv(os.path.join(cmp_dir, "method_ranking.csv"))
    assumptions = collect_assumptions()
    todo = [a for a in assumptions if a["needs_source"]]

    L: List[str] = []
    L.append("# 회전 우주 파편 포획 방식 비교 — 결과 정리\n")
    L.append("이 문서는 `make_report.py` 가 시뮬레이션 산출물에서 자동 생성한다. "
             "수치를 옮길 때는 생성 시각의 결과와 원고가 같은 실행에서 나왔는지 확인할 것.\n")

    L.append("## 1. 실험 설계\n")
    L.append(f"- 물리 엔진은 PyBullet 이고 중력은 0 이다. 좌표계는 고도 "
             f"{cs.CONFIG['orbit']['altitude_km']:.0f} km 원궤도의 LVLH 프레임으로 해석한다.")
    L.append("- Hill(Clohessy–Wiltshire) 방정식의 겉보기 가속도와 중력구배 토크를 매 스텝 외력으로 적용한다.")
    L.append(f"- 성공 판정은 대상과 서비스 위성의 상대 각속도가 "
             f"{cs.CONFIG['success']['rel_omega_threshold_deg_s']:.0f} deg/s 이하로 떨어지고, "
             f"그 상태가 {cs.CONFIG['success']['hold_time']:.0f} s 이상 유지되며 결합이 끊기지 않는 경우다.")
    L.append(f"- 한 시행의 최대 시뮬레이션 시간은 {cs.CONFIG['sim']['max_time']:.0f} s, "
             f"적분 시간 간격은 1/{round(1 / cs.CONFIG['sim']['timestep'])} s 이다.")
    L.append("- 반복마다 회전축·회전속도 추정 오차, 상대 위치·잔류 속도 오차, 서비서 지향 오차, "
             "초기 자세를 시드로 섭동한다. 같은 반복 번호는 모든 조합에서 같은 시드를 쓴다.\n")

    L.append("## 2. 결과\n")
    L += summarize_results(summary)

    L.append("## 3. 방식 간 통계 비교\n")
    L += summarize_comparison(cmp_by_target, cmp_ranking)

    L.append("## 4. 수치 검증\n")
    L += summarize_verification(os.path.join(ver_dir, "verification_report.txt"))

    L.append("## 5. 가정값 민감도\n")
    L += summarize_sensitivity(sens)

    L.append("## 6. 가정값 목록\n")
    L.append(f"전체 가정값은 {len(assumptions)}개이고, 그중 문헌 근거를 확인하지 못한 값이 {len(todo)}개다. "
             "전체 표는 `report/assumptions.tsv` 에 있다. 아래는 출처 확인이 필요한 값만 추린 것이다.\n")
    L.append(md_table(["구분", "항목", "값", "메모"],
                      [[a["section"], a["key"], a["value"], a["note"][:60]] for a in todo]))
    L.append("")

    L.append("## 7. 그림 캡션 초안\n")
    L.append("- 그림 1. 포획 방식별 회전 속도와 대상 질량에 따른 포획 성공률. "
             "각 칸은 같은 조건을 시드를 바꿔 반복한 시행의 성공 비율이다. (`results/heatmap_success.png`)")
    L.append("- 그림 2. 시행 종료 시점의 잔류 상대 각속도 중앙값. "
             "성공 여부만으로는 드러나지 않는 감속 진행 정도를 보여준다. (`results/heatmap_final_rel_omega.png`)")
    L.append("- 그림 3. 성공한 시행의 감속 소요 시간 중앙값. "
             "로봇팔은 초 단위, 테더 방식은 분 단위로 차이가 크다. (`results/heatmap_capture_time.png`)")
    L.append("- 그림 4. 대상에 전달된 최대 충격량 평균. 2차 파편 발생 위험의 지표다. "
             "(`results/heatmap_impulse.png`)")
    L.append("- 그림 5. 대상 질량별 회전 속도-성공률 곡선. 음영은 Wilson 95 % 신뢰구간이다. "
             "(`results/curves_success.png`)")
    L.append("- 그림 6. 가정값 민감도. 막대는 각 값을 낮음/높음으로 바꿨을 때 기준 대비 변화량이다. "
             "(`sensitivity/sensitivity_tornado_rel_omega.png`)\n")

    L.append("## 8. 방법론에 반드시 적을 것\n")
    L.append("1. 그물과 작살의 감속은 전적으로 테더 윈치의 정장력 제어에 의존한다. "
             "윈치가 없으면 테더가 팽팽해지지 않아 각운동량 전달 경로 자체가 없다.")
    L.append("2. 테더 감쇠가 부족하면 대상이 테더에 매달려 진동이 남는다. "
             "명시적 적분의 안정조건 때문에 감쇠를 키우려면 테더 질점 질량도 함께 키워야 한다.")
    L.append("3. 그물 질량은 면밀도 × 면적으로 대상마다 달라진다. "
             "질점 질량을 고정하면 작은 그물이 비현실적으로 무거워져 대상을 쳐서 날려버린다.")
    L.append(f"4. 기본 회전축은 접근축에 수직인 z 축이다. 중형 대상은 태양전지판이 접근 평면을 쓸고 지나가므로 "
             "로봇팔에 불리하다. 회전축의 영향은 민감도 분석에서 따로 정량화했다.")
    L.append(f"5. 시간 창 {cs.CONFIG['sim']['max_time']:.0f} s 는 테더 방식의 감속 시간 규모에 맞춰 정한 값이다. "
             "이 값을 줄이면 테더 방식이 일방적으로 불리해진다.\n")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description="시뮬레이션 산출물에서 논문용 결과 정리 생성")
    ap.add_argument("--results", default="results")
    ap.add_argument("--verification", default="verification")
    ap.add_argument("--sensitivity", default="sensitivity")
    ap.add_argument("--comparison", default="comparison")
    ap.add_argument("--out", default="report")
    ap.add_argument("--only", choices=["assumptions", "report"], default=None)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    assumptions = collect_assumptions()
    ap_path = os.path.join(args.out, "assumptions.tsv")
    with open(ap_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter="\t")
        w.writerow(["구분", "항목", "값", "출처 확인 필요", "메모"])
        for a in assumptions:
            w.writerow([a["section"], a["key"], a["value"], "예" if a["needs_source"] else "아니오", a["note"]])
    print(f"가정값 {len(assumptions)}개 (출처 확인 필요 {sum(a['needs_source'] for a in assumptions)}개) -> {ap_path}")
    if args.only == "assumptions":
        return

    text = build_report(args.results, args.verification, args.sensitivity, args.comparison)
    rp = os.path.join(args.out, "report.md")
    with open(rp, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"보고서 -> {rp}")


if __name__ == "__main__":
    main()
