#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
수치 검증 (논문 부록용)
========================

1. 토크-프리 회전 검증
   PyBullet 이 적분한 강체 회전을 오일러 방정식의 RK4 기준해와 비교하고, 각운동량 보존을 확인한다.
   (각감쇠가 0 이 아니면 각운동량이 보존되지 않으므로 CONFIG 의 angular_damping = 0 을 검증하는 의미도 있다.)

2. 시간 스텝 수렴성
   dt 를 절반씩 줄여가며 같은 시행의 결과 지표가 수렴하는지 본다.
   결과가 dt 에 따라 크게 흔들리면 그 조건의 수치 결과는 신뢰할 수 없다.

3. 충격량 측정 일관성
   접촉 중 대상의 운동량 변화로 측정한 충격량이 접촉력 적분과 일치하는지 본다.

실행
    conda activate debris-sim
    python run_verification.py                 # 전체
    python run_verification.py --only dt       # 스텝 수렴만
    python run_verification.py --out verification/
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from typing import Dict, List

import numpy as np
import pybullet as p

import capture_sim as cs


# =============================================================================
# 1. 토크-프리 회전 (오일러 방정식 RK4 기준해와 비교)
# =============================================================================
def check_torque_free(duration: float = 20.0, dt: float = 1.0 / 480.0) -> Dict:
    """비대칭 관성체를 주축에서 벗어난 각속도로 돌려 세차 운동을 재현하고 기준해와 비교"""
    p.connect(p.DIRECT)
    try:
        p.setGravity(0, 0, 0)
        p.setTimeStep(dt)
        he = [0.5, 0.2, 0.1]                      # 세 주관성모멘트가 모두 다른 형상
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
        body = p.createMultiBody(baseMass=10.0, baseCollisionShapeIndex=col)
        p.changeDynamics(body, -1, linearDamping=0.0, angularDamping=0.0)
        I = np.array(p.getDynamicsInfo(body, -1)[2])
        w0 = np.array([0.0, 0.5, 0.05])           # 주축(y)에서 기울어진 각속도 → 세차
        p.resetBaseVelocity(body, angularVelocity=w0.tolist())

        def euler_eq(w):
            return -np.cross(w, I * w) / I

        w_ref = w0.copy()
        n = int(round(duration / dt))
        for _ in range(n):
            k1 = euler_eq(w_ref)
            k2 = euler_eq(w_ref + dt / 2 * k1)
            k3 = euler_eq(w_ref + dt / 2 * k2)
            k4 = euler_eq(w_ref + dt * k3)
            w_ref = w_ref + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            p.stepSimulation()

        w_world = np.array(p.getBaseVelocity(body)[1])
        q = p.getBasePositionAndOrientation(body)[1]
        R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
        w_body = R.T @ w_world
        L_world = R @ (I * w_body)
        L0 = I * w0
        return {
            "duration_s": duration, "dt_s": dt,
            "omega_body_pybullet": w_body, "omega_body_rk4": w_ref,
            "omega_abs_error_rad_s": float(np.linalg.norm(w_body - w_ref)),
            "omega_rel_error": float(np.linalg.norm(w_body - w_ref) / np.linalg.norm(w_ref)),
            "L_initial": L0, "L_final": L_world,
            "L_rel_error": float(np.linalg.norm(L_world - L0) / np.linalg.norm(L0)),
            "inertia": I,
        }
    finally:
        p.disconnect()


# =============================================================================
# 2. 시간 스텝 수렴성
# =============================================================================
DT_METRICS = ["success", "capture_time", "impulse", "servicer_attitude_error", "final_rel_omega_deg_s",
              "first_contact_time", "peak_contact_force"]


def check_timestep(cases: List[Dict], dts: List[float]) -> List[Dict]:
    """
    같은 시행을 여러 dt 로 반복한다.

    그물의 스프링 계수는 적분 안정조건에서 유도하므로(k = f x m/dt^2) dt 를 바꾸면
    모델 자체가 달라져 수렴성 시험이 성립하지 않는다. 그래서 기준 dt 로 계수를 고정하고
    적분 스텝만 바꾼다. 기준보다 큰 dt 는 안정조건을 넘으므로 기준 이하만 시험한다.
    """
    rows = []
    base_dt = cs.CONFIG["sim"]["timestep"]
    base_fps = cs.CONFIG["scene"]["viewer_fps"]
    base_ref = cs.CONFIG["net"]["spring_reference_dt"]
    dts = [d for d in dts if d <= base_dt + 1e-12]
    if not dts:
        dts = [base_dt]
    cs.CONFIG["net"]["spring_reference_dt"] = base_dt   # 모델 고정
    try:
        for case in cases:
            for dt in dts:
                cs.CONFIG["sim"]["timestep"] = dt
                r = cs.run_capture(case["method"], case["target"], case["spin"], seed=case.get("seed", 0))
                row = {"method": case["method"], "target": case["target"], "spin_deg_s": case["spin"],
                       "dt_s": dt, "steps_per_s": round(1 / dt)}
                row.update({k: r.get(k) for k in DT_METRICS})
                rows.append(row)
                print(f"  {case['method']:8s} {case['target']:7s} spin={case['spin']:4.0f}  dt=1/{round(1/dt):4d} "
                      f"-> success={r['success']} impulse={r['impulse']:8.2f} "
                      f"final_rel={r['final_rel_omega_deg_s']:7.3f} att_err={r['servicer_attitude_error']:6.2f}", flush=True)
    finally:
        cs.CONFIG["sim"]["timestep"] = base_dt
        cs.CONFIG["scene"]["viewer_fps"] = base_fps
        cs.CONFIG["net"]["spring_reference_dt"] = base_ref
    return rows


def summarize_timestep(rows: List[Dict]) -> List[Dict]:
    """가장 작은 dt 를 기준으로 각 지표의 상대 편차를 계산"""
    out = []
    groups: Dict[tuple, List[Dict]] = {}
    for r in rows:
        groups.setdefault((r["method"], r["target"], r["spin_deg_s"]), []).append(r)
    for key, g in groups.items():
        g = sorted(g, key=lambda r: r["dt_s"])
        ref = g[0]
        for r in g:
            item = {"method": key[0], "target": key[1], "spin_deg_s": key[2], "dt_s": r["dt_s"],
                    "success": r["success"], "success_matches_ref": r["success"] == ref["success"]}
            for k in ("impulse", "servicer_attitude_error", "final_rel_omega_deg_s"):
                a, b = r.get(k), ref.get(k)
                try:
                    item[k + "_rel_dev"] = abs(float(a) - float(b)) / max(abs(float(b)), 1e-9)
                except (TypeError, ValueError):
                    item[k + "_rel_dev"] = float("nan")
            out.append(item)
    return out


# =============================================================================
# 3. 충격량 측정 일관성 (운동량 변화 vs 접촉력 적분)
# =============================================================================
def check_impulse(method: str = "harpoon", target: str = "medium", spin: float = 10.0,
                  seed: int = 0, duration: float = 6.0) -> Dict:
    """대상의 운동량 변화 누적과 접촉 법선력 적분을 비교한다 (제약력이 없는 비행·충돌 구간에서 일치해야 함)"""
    cfg = cs.CONFIG
    world = cs.World(cfg, False)
    try:
        rng = np.random.default_rng(seed)
        m = cs.METHODS[method](cfg, rng)
        tgt = cs.Target(cfg, target, spin, "principal", rng)
        svc = cs.Servicer(cfg, m.servicer_position(tgt), m.arm_spec())
        m.setup(world, svc, tgt)
        v_prev = tgt.velocity()[0]
        dp_sum = np.zeros(3)
        f_sum = np.zeros(3)
        for _ in range(int(duration / world.dt)):
            m.pre_step()
            world.step()
            v = tgt.velocity()[0]
            dp_sum += tgt.mass * (v - v_prev)
            v_prev = v
            contacts = p.getContactPoints(bodyA=tgt.body)
            for c in contacts:
                f_sum += np.array(c[7]) * c[9] * world.dt * (1 if c[1] == tgt.body else -1)
            m.post_step(contacts)
        return {"method": method, "target": target, "spin_deg_s": spin, "duration_s": duration,
                "momentum_change_Ns": dp_sum, "contact_force_integral_Ns": f_sum,
                "abs_difference_Ns": float(np.linalg.norm(dp_sum - f_sum)),
                "momentum_change_norm_Ns": float(np.linalg.norm(dp_sum))}
    finally:
        world.close()


# =============================================================================
# 실행
# =============================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description="수치 검증 (토크-프리 회전 / 스텝 수렴 / 충격량 일관성)")
    ap.add_argument("--only", choices=["rotation", "dt", "impulse"], default=None)
    ap.add_argument("--out", default="verification")
    ap.add_argument("--dts", type=int, nargs="+", default=[480, 960, 1920],
                    help="시험할 스텝 (1/N 초). 기준 dt 보다 큰 스텝은 안정조건을 넘어 자동 제외된다")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    lines: List[str] = []

    def log(s=""):
        print(s, flush=True)
        lines.append(s)

    if args.only in (None, "rotation"):
        log("=" * 78)
        log("1. 토크-프리 회전: PyBullet vs 오일러 방정식 RK4")
        log("=" * 78)
        r = check_torque_free()
        log(f"  관성 주모멘트      : {np.round(r['inertia'], 4)} kg·m^2")
        log(f"  {r['duration_s']:.0f} s 후 동체 각속도")
        log(f"    PyBullet         : {np.round(r['omega_body_pybullet'], 6)} rad/s")
        log(f"    RK4 기준해       : {np.round(r['omega_body_rk4'], 6)} rad/s")
        log(f"    상대 오차        : {r['omega_rel_error']:.3e}")
        log(f"  각운동량 상대 오차 : {r['L_rel_error']:.3e}  (0 에 가까울수록 보존)")
        log(f"  판정               : {'통과' if r['omega_rel_error'] < 1e-3 and r['L_rel_error'] < 1e-3 else '확인 필요'}")
        log()

    if args.only in (None, "dt"):
        log("=" * 78)
        log("2. 시간 스텝 수렴성")
        log("=" * 78)
        cases = [{"method": "arm", "target": "small", "spin": 10.0},
                 {"method": "arm", "target": "large", "spin": 10.0},
                 {"method": "harpoon", "target": "medium", "spin": 10.0},
                 {"method": "net", "target": "medium", "spin": 10.0}]
        dts = sorted(1.0 / n for n in args.dts)
        rows = check_timestep(cases, dts)
        with open(os.path.join(args.out, "timestep_raw.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        summ = summarize_timestep(rows)
        with open(os.path.join(args.out, "timestep_convergence.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
        log()
        log("  가장 작은 dt 대비 상대 편차 (작을수록 수렴)")
        log(f"  {'method':8s} {'target':7s} {'dt':>8s} {'success일치':>10s} {'impulse':>10s} {'att_err':>10s} {'rel_omega':>10s}")
        for r in summ:
            log(f"  {r['method']:8s} {r['target']:7s} 1/{round(1/r['dt_s']):<6d} {str(r['success_matches_ref']):>10s} "
                f"{r['impulse_rel_dev']:10.3f} {r['servicer_attitude_error_rel_dev']:10.3f} {r['final_rel_omega_deg_s_rel_dev']:10.3f}")
        log()
        log("  해석: 접촉이 있는 다물체 시뮬레이션은 스텝을 바꾸면 접촉 순서가 달라져 결과가 완전히 같아지지 않는다.")
        log("        논문에는 '성공 판정이 dt 에 따라 뒤집히는가'와 '충격량이 같은 자릿수인가'를 근거로 제시할 것.")
        log("  주의: 그물의 스프링 계수는 안정조건에서 dt 로부터 유도되므로, 이 시험에서는 기준 dt 의 계수로")
        log("        고정해 모델을 붙들어 두고 적분 스텝만 바꿨다. 그러지 않으면 모델이 함께 바뀌어 수렴성을 볼 수 없다.")
        log()

    if args.only in (None, "impulse"):
        log("=" * 78)
        log("3. 충격량 측정 일관성 (운동량 변화 누적 vs 접촉력 적분)")
        log("=" * 78)
        for case in (("harpoon", "medium"), ("net", "medium")):
            r = check_impulse(case[0], case[1])
            log(f"  {r['method']:8s} {r['target']:7s} {r['duration_s']:.0f} s")
            log(f"    운동량 변화 누적 : {np.round(r['momentum_change_Ns'], 3)} N·s (크기 {r['momentum_change_norm_Ns']:.3f})")
            log(f"    접촉력 적분      : {np.round(r['contact_force_integral_Ns'], 3)} N·s")
            log(f"    차이             : {r['abs_difference_Ns']:.3f} N·s")
        log()
        log("  해석: 제약(파지·작살 고정)과 스프링 외력이 함께 작용하는 구간에서는 두 값이 일치하지 않는 것이 정상이다.")
        log("        결과 dict 의 impulse 는 운동량 변화 기준이며, 이것이 2차 파편 위험 지표로 타당하다.")
        log()

    path = os.path.join(args.out, "verification_report.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"report -> {path}")


if __name__ == "__main__":
    main()
