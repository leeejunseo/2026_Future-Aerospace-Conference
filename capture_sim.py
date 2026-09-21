#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
회전 우주 파편 포획 시뮬레이터 (포획 순간 단계)
================================================

연구 목적
    회전하는 우주 파편을 포획할 때, 대상의 회전 속도와 질량에 따라
    로봇팔 / 그물 / 작살 중 어떤 방식이 유효한지 정량 비교한다.

구현 범위 (이번 단계)
    - 물리 엔진: PyBullet, 중력 0
    - 서비스 위성 1기가 이미 목표 근처에서 랜데부한 상태에서 시작
    - 접근 궤도 계산은 제외 (초기 상대 위치·속도는 CONFIG 로 지정)

실행 예
    # 헤드리스 단일 시행
    python capture_sim.py --method arm --target medium --spin 10
    # GUI 시각화
    python capture_sim.py --method net --target large --spin 20 --gui
    # 세차 운동(장동각 20도) 대상
    python capture_sim.py --method harpoon --target small --spin 30 --spin-mode precession --nutation 20
    # 전체 조합 스윕 → CSV
    python capture_sim.py --sweep --spins 0 5 10 20 30 --out results.csv

환경
    conda create -n debris-sim -c conda-forge python=3.12 pybullet numpy
    (PyPI 의 pybullet 은 최신 파이썬용 윈도우 휠이 없어 conda-forge 사용)

설계 메모
    - 모든 물리 상수·가정값은 파일 상단 CONFIG 에 모아두었다.
      근거가 없는 임의값은 "# TODO: 출처 확인 필요" 로 표시했다.
    - 포획 방식 3종은 CaptureMethod 공통 인터페이스(setup/pre_step/post_step/bond_intact)를 갖는다.
    - 성공률·성공 여부는 어떠한 경험식도 쓰지 않고, 물리 시뮬레이션 상태(상대 각속도, 결합 유지)로만 판정한다.
    - 그물/테더는 PyBullet 소프트바디 대신 명시적 질점-스프링(Kelvin-Voigt, 인장 전용) 모델을 사용해
      논문에서 수식으로 기술할 수 있게 했다.
    - PyBullet 이 토크-프리 세차 운동(오일러 방정식)을 올바로 적분하는지 RK4 기준해와 비교 검증했다
      (각감쇠를 0 으로 두어야 각운동량이 보존됨 → CONFIG["target_dynamics"]["angular_damping"] = 0).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pybullet as p

# =============================================================================
# CONFIG : 모든 물리 상수 및 가정값
# =============================================================================
CONFIG: Dict = {
    # ---------------------------------------------------------------- 시뮬레이션
    "sim": {
        "timestep": 1.0 / 480.0,      # PyBullet 기본(1/240)의 절반. 명시적 스프링 안정조건 k*dt^2/m < 0.25, c*dt/m < 0.5 를 위해 축소
        # 포획 시도 최대 시간 [s]. 테더 방식(그물·작살)의 감속은 100 s 규모라 60 s 창에서는 방식 비교가 성립하지 않는다.
        "max_time": 180.0,            # TODO: 출처 확인 필요
        "gravity": (0.0, 0.0, 0.0),   # 궤도상 자유낙하 → 상대운동 좌표계에서 중력 0 (연구 정의)
        "solver_iterations": 150,     # 제약 솔버 반복 횟수. 강성 제약(파지/작살 고정)의 드리프트 억제용  # TODO: 출처 확인 필요
        "gui_realtime": True,         # GUI 모드에서 실시간 속도로 재생
        "debug_lines": True,          # GUI 모드에서 스프링을 선으로 표시
        "debug_line_interval": 8,     # 몇 스텝마다 디버그 선을 갱신할지 (렌더 비용 절감)
        "record_history": False,      # True 면 결과 dict 에 시계열(history) 포함
        "escape_distance_factor": 4.0,  # 대상이 초기 이격거리의 이 배수 이상 멀어지면 조기 종료(실패)  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 성공 판정
    "success": {
        "rel_omega_threshold_deg_s": 1.0,  # 대상-서비서 상대 각속도 임계값 (연구 정의)
        "hold_time": 3.0,                  # 임계값 이하 상태가 이 시간 이상 유지되어야 성공 (순간 통과 배제)  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 대상 파편
    "targets": {
        "small": {
            "mass": 10.0,                          # 연구 정의 (소형)
            "shape": "box",
            "half_extents": (0.15, 0.15, 0.15),    # 한 변 0.3 m 정육면체 (큐브샛 급 크기 가정)  # TODO: 출처 확인 필요
            "grasp_point": (-0.15, 0.0, 0.0),      # 로봇팔 파지 / 작살 조준점 (동체 좌표, 서비서를 향한 -x 면 중심)
            "grasp_normal": (-1.0, 0.0, 0.0),      # 파지점 외향 법선 (동체 좌표)
        },
        "medium": {
            "mass": 500.0,                         # 연구 정의 (중형)
            "shape": "bus_with_panels",
            "bus_half_extents": (0.5, 0.5, 0.5),   # 본체 1 m 정육면체  # TODO: 출처 확인 필요
            "panel_half_extents": (0.5, 1.0, 0.01),  # 태양전지판 1 m x 2 m x 0.02 m, 본체 ±y 방향  # TODO: 출처 확인 필요
            "panel_offset_y": 1.6,                 # 본체 중심에서 패널 중심까지 거리 (0.5 + 0.1 간격 + 1.0)
            "panel_mass": 25.0,                    # 패널 1장 질량 (총 질량의 5 %)  # TODO: 출처 확인 필요
            "grasp_point": (-0.5, 0.0, 0.0),
            "grasp_normal": (-1.0, 0.0, 0.0),
        },
        "large": {
            "mass": 2000.0,                        # 연구 정의 (대형)
            "shape": "cylinder",
            "radius": 1.0,                         # 상단 로켓 단(段) 급 원통 가정  # TODO: 출처 확인 필요
            "length": 3.0,                         # TODO: 출처 확인 필요
            "grasp_point": (-1.0, 0.0, 0.0),       # 원통 옆면(-x) 점
            "grasp_normal": (-1.0, 0.0, 0.0),
        },
    },
    # ---------------------------------------------------------------- 대상 동역학 공통
    "target_dynamics": {
        "restitution": 0.2,        # 금속 구조물 충돌 반발계수 가정  # TODO: 출처 확인 필요
        "lateral_friction": 0.5,   # 표면 마찰계수 가정  # TODO: 출처 확인 필요
        "linear_damping": 0.0,     # 우주 공간: 공기 저항 없음 → 0 (PyBullet 기본값 0.04 를 반드시 덮어씀)
        "angular_damping": 0.0,    # 동일. 0 이 아니면 각운동량이 보존되지 않아 세차 운동이 왜곡됨
        "initial_attitude_jitter_deg": 5.0,  # 시드 기반 초기 자세 무작위 섭동 (표준편차)  # TODO: 출처 확인 필요
        "spin_axis": "z",          # 주축 회전축: "x"/"y"/"z" 또는 "major"(최대 관성축)/"minor"(최소 관성축)
        "nutation_angle_deg": 20.0,  # 세차 모드에서 각속도 벡터와 회전축 사이 각도 (장동각)  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 서비스 위성
    "servicer": {
        "mass": 1000.0,                     # 중형 서비스 위성 가정  # TODO: 출처 확인 필요
        "half_extents": (0.75, 0.75, 0.75),  # 1.5 m 정육면체 본체  # TODO: 출처 확인 필요
        "attach_point": (0.75, 0.0, 0.0),   # 팔 장착부 / 테더·그물 발사구 위치 (동체 좌표, +x 면 중심)
        "restitution": 0.2,                 # TODO: 출처 확인 필요
        "lateral_friction": 0.5,            # TODO: 출처 확인 필요
        "attitude_hold": False,             # True 면 반작용휠 PD 자세 유지. 접촉 중에는 보통 free-drift 로 두므로 기본 False  # TODO: 출처 확인 필요
        "attitude_kp": 200.0,               # PD 자세 제어 비례 이득 [N·m/rad]  # TODO: 출처 확인 필요
        "attitude_kd": 400.0,               # PD 자세 제어 미분 이득 [N·m·s/rad]  # TODO: 출처 확인 필요
        "attitude_max_torque": 1.0,         # 반작용휠 최대 토크 [N·m]  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 궤도 환경 (LVLH 회전좌표계 상대 역학)
    # 시뮬레이션 좌표계 = 대상 기준 궤도의 LVLH 프레임. x: 진행 방향(along-track), z: 지구 반대(radial, 천정), y: 궤도면 수직(cross-track)
    # 각 강체에 Hill(Clohessy-Wiltshire) 방정식의 겉보기 가속도와 중력구배 토크를 매 스텝 외력으로 적용한다.
    # 60 s 포획 구간에서 크기는 mm~cm, 토크는 mN·m 수준이지만 "실제 궤도 환경" 재현을 위해 포함 (--no-orbit 으로 끔)
    "orbit": {
        "enabled": True,
        "altitude_km": 500.0,            # 기준 궤도 고도 (LEO 파편 밀집 고도대 가정)  # TODO: 출처 확인 필요
        "mu_earth_m3s2": 3.986004418e14,  # 지구 중력 상수 (표준 상수)
        "earth_radius_km": 6371.0,       # 지구 평균 반경
        "apply_to_nodes": False,         # 그물/테더 질점(0.05 kg)에는 미적용: 효과 mm 이하, 호출 비용만 큼
        "gravity_gradient_torque": True,  # 대상·서비서에 3n^2 (r_hat x I r_hat) 토크 적용
    },
    # ---------------------------------------------------------------- GUI 장면 연출 (물리에 영향 없음)
    "scene": {
        "earth_texture": "assets/earth.jpg",  # NASA Blue Marble (land_shallow_topo_2048). 없으면 단색 구
        "three_js": "assets/three.min.js",  # 웹 뷰어에 내장할 three.js. 없으면 CDN 에서 불러오므로 인터넷 필요
        "earth_visual_radius_m": 300.0,  # 지구를 실제 크기 대신 축소 구로 표현. 거리는 실제 고도에서 보이는 각크기(반각 asin(Re/(Re+h)))와 일치시킴 (GUI 원거리 클리핑 ~1000 m 이내)
        "stars": 2000,                   # 배경 별(디버그 포인트) 개수
        "star_distance_m": 900.0,        # GUI 원거리 클리핑(~1000 m) 이내
        "sun_direction": (0.6, 0.5, 0.62),  # 태양광 방향 (GUI 조명)
        "camera_follow": True,           # 카메라 주시점을 대상-서비서 중점으로 자동 추적 (마우스 회전/줌은 유지)
        "hud": True,                     # 시각·상대 각속도·단계 HUD 텍스트
        "hud_interval": 24,              # HUD/카메라 갱신 주기 [step]
        "record_interval": 8,            # 프레임 녹화 주기 [step] (--record-frames)
        "viewer_fps": 30,                # 웹 뷰어(--viewer) 궤적 기록 주기 [frame/s]
        "frame_size": (1280, 720),
    },
    # ---------------------------------------------------------------- 몬테카를로 섭동 (시드 기반, 실제 임무의 불확실성 재현)
    # 기본값 0 → 단일 시행 재현성. 격자 실험 스크립트(run_grid_experiment.py)에서 현실적 값으로 덮어씀
    "monte_carlo": {
        "spin_axis_jitter_deg": 0.0,        # 회전축 방향 무작위 편차 표준편차 [deg] (지상 관측으로 추정한 회전축의 불확실성)  # TODO: 출처 확인 필요
        "spin_rate_jitter_frac": 0.0,       # 회전 속도 상대 오차 표준편차 (관측 불확실성)  # TODO: 출처 확인 필요
        "relative_position_error_m": 0.0,   # 랑데부 종료 시 횡방향(y,z) 상대 위치 오차 표준편차 [m]  # TODO: 출처 확인 필요
        "relative_velocity_error_mps": 0.0, # 잔류 상대 속도 오차 표준편차 (3축) [m/s]  # TODO: 출처 확인 필요
        "servicer_pointing_error_deg": 0.0, # 서비서 지향(자세) 오차 표준편차 [deg]  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 초기 이격 (표면-표면 거리, 방식별)
    "standoff": {
        "arm": 0.3,       # 팔 작업 반경 내 근접 정지 상태  # TODO: 출처 확인 필요
        "net": 6.0,       # 그물 발사 거리  # TODO: 출처 확인 필요
        "harpoon": 8.0,   # 작살 발사 거리  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 1) 로봇팔
    "arm": {
        "link_lengths": (0.6, 0.6, 0.5, 0.4, 0.3, 0.2),  # 6관절, 총 도달거리 2.6 m  # TODO: 출처 확인 필요
        "joint_axes": ((0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0)),  # 요-피치-피치-피치-요-롤
        "link_radius": 0.05,          # 링크 캡슐 반경 [m]  # TODO: 출처 확인 필요
        "link_mass": 8.0,             # 링크 질량 [kg]  # TODO: 출처 확인 필요
        "tool_mass": 2.0,             # 그리퍼 질량 [kg]  # TODO: 출처 확인 필요
        "tool_radius": 0.06,          # 그리퍼(구) 반경 [m]
        "joint_max_torque": 200.0,    # 관절 최대 토크 [N·m]  # TODO: 출처 확인 필요
        "joint_max_velocity": 1.0,    # 관절 최대 각속도 [rad/s]  # TODO: 출처 확인 필요
        "tool_max_speed": 1.0,        # 그리퍼 목표점 이동 속도 한계 [m/s]. 목표점을 직선(데카르트) 경로로 보간해 관절보간 시 호를 그리며 대상을 관통하는 현상 방지  # TODO: 출처 확인 필요
        "joint_damping": 2.0,         # 관절 점성 감쇠 [N·m·s/rad]  # TODO: 출처 확인 필요
        "joint_limit_deg": 170.0,     # 관절 가동 범위 ± [deg]  # TODO: 출처 확인 필요
        "position_gain": 0.3,         # PyBullet 위치제어 이득 (무차원)  # TODO: 출처 확인 필요
        "ready_pose": (1.2, 0.6, -1.2, 0.6, 0.0, 0.0),  # 대기 자세 [rad]: 요 1.2 rad 로 옆으로 접어 대상과 초기 겹침 방지
        "approach_standoff": 0.3,     # 파지점 법선 방향 대기 거리 [m]  # TODO: 출처 확인 필요
        "pre_approach_distance": 0.6,  # 대기점보다 법선 방향으로 더 바깥의 예비 접근점 거리 [m]. 대기 자세→대기점 직행 시 대상 관통 방지  # TODO: 출처 확인 필요
        "approach_cone_deg": 60.0,    # 파지점 법선이 어깨 방향과 이 각도 이내일 때만 접근 (그 외에는 대상 정면 안전 대기점에서 대기)  # TODO: 출처 확인 필요
        "approach_tolerance": 0.08,   # 대기점 도달 허용오차 [m]
        "grasp_tolerance": 0.05,      # 파지 성립 위치 허용오차 [m] (그리퍼 개구 폭 가정)  # TODO: 출처 확인 필요
        "grasp_max_rel_speed": 0.5,   # 파지 성립 최대 상대속도 [m/s]  # TODO: 출처 확인 필요
        "grasp_force_limit": 2000.0,  # 그리퍼 유지력 한계 [N] (PyBullet 제약 maxForce). 초과 요구 시 미끄러짐 발생  # TODO: 출처 확인 필요
        "grasp_closure_time": 1.0,    # 그리퍼 닫힘 시간 [s]. 이 동안 유지력 한계를 0→최대로 선형 증가(순응 파지)  # TODO: 출처 확인 필요
        "slip_tolerance": 0.10,       # 파지점이 그리퍼 중심에서 이만큼 벗어나면 이탈(그리퍼 개구 폭 가정) [m]  # TODO: 출처 확인 필요
        "reach_margin": 0.95,         # 도달 가능 판정 시 총 링크 길이의 비율
        "post_grasp_mode": "brake",   # "brake": 파지 후 관절을 속도 0 목표(토크 제한)로 → 관절이 브레이크 역할
        "reach_fraction_at_standoff": 0.6,  # 초기 배치: 대기점이 어깨로부터 도달거리의 이 비율 위치에 오도록
    },
    # ---------------------------------------------------------------- 2) 그물
    "net": {
        "grid_n": 9,                  # 한 변 질점 수 (9x9 = 81 질점)  # TODO: 출처 확인 필요
        "size_by_target": {"small": 2.0, "medium": 7.0, "large": 5.0},  # 전개 후 한 변 길이 [m], 대상 외접 지름의 ~1.4배  # TODO: 출처 확인 필요
        # 질량은 그물 면적에 비례한다. 질점 질량을 크기와 무관하게 고정하면 작은 그물이 비현실적으로 무거워져
        # (2 m 그물이 7.9 kg → 10 kg 대상의 79 %) 대상을 포획하지 않고 쳐서 날려버린다.
        "areal_density": 0.05,        # 그물 면밀도 [kg/m^2] → 질점 질량 = 면밀도 x 면적 / 질점수  # TODO: 출처 확인 필요
        "corner_mass_multiple": 20.0,  # 모서리 추 질량 = 일반 질점 질량 x 이 값 (전개용 추)  # TODO: 출처 확인 필요
        "node_radius_ratio": 0.45,    # 질점 충돌 반경 = 격자 간격 x 비율. 격자 사이로 대상이 빠져나가지 않게 하는 모델링 가정
        # 스프링 계수는 명시적(반-암시적 오일러) 적분의 안정조건에서 유도한다.
        #   강성  k <= 0.25 * m / dt^2 , 감쇠  c <= 0.5 * m / dt
        # 그물 질량이 대상별로 달라지므로 계수를 고정값으로 두면 작은 그물에서 발산한다.
        # 아래 비율은 안정 한계 대비 여유를 둔 값이며, 그물실은 "안정조건이 허용하는 한 비신축성"으로 모델링된다.
        "spring_k_factor": 0.20,      # k = 이 값 x m / dt^2  (안정 한계 0.25)
        "spring_c_factor": 0.30,      # c = 이 값 x m / dt    (안정 한계 0.5)
        "shear_springs": True,        # 대각(전단) 스프링 포함 여부
        "tension_only": True,         # 실은 압축력을 전달하지 않음
        "deploy_speed": 4.0,          # 발사 속도 [m/s]  # TODO: 출처 확인 필요
        "folded_ratio": 0.3,          # 발사 시 접힌 크기 / 전개 크기
        "open_at_fraction": 0.7,      # 비행 시간의 이 비율 시점에 완전 전개되도록 모서리 추 측방 속도 계산
        "launch_offset": 1.0,         # 서비서 발사구에서 그물 초기 중심까지 거리 [m]
        "launch_angle_error_deg": 1.0,  # 발사 방향 오차 표준편차 (시드 랜덤)  # TODO: 출처 확인 필요
        # 테더: 포획 직후 팽팽해져야 대상의 각운동량이 서비서로 전달된다(감속 경로).
        # 자유 길이가 실제 도달 거리보다 길면 인장 전용 스프링이 끝까지 힘을 내지 않아 감속이 전혀 일어나지 않는다.
        "tether_length_factor": 1.05,  # 테더 자유 길이 = (초기 이격 거리 + 대상 반경) x 이 값  # TODO: 출처 확인 필요
        # 테더 계수도 같은 방식으로 모서리 추 질량에서 유도한다 (테더는 모서리 추에 연결된다).
        "tether_k_factor": 0.20,      # k = 이 값 x 모서리 추 질량 / dt^2
        "tether_c_factor": 0.30,      # c = 이 값 x 모서리 추 질량 / dt
        # 윈치: 결합 성립 후 테더를 되감아 장력을 유지한다. 실제 그물 포획 임무의 회수 단계에 해당.
        "winch_enabled": True,
        "winch_start_delay": 1.0,     # 결합 성립 후 되감기 시작까지 지연 [s]  # TODO: 출처 확인 필요
        "winch_reel_rate": 0.35,      # 되감기 속도 [m/s]  # TODO: 출처 확인 필요
        # 최소 자유 길이 비율. 이 값이 크면 되감기 완료 후 테더가 느슨해져 장력이 0 이 되고 감속이 멈춘다.
        # 작게 두면 스프링이 항상 늘어난 상태가 되어 winch_max_tension 으로 제한되는 "정장력 윈치"로 동작한다.
        "winch_min_length_factor": 0.04,  # TODO: 출처 확인 필요
        "winch_max_tension": 250.0,   # 테더 장력 상한 [N]. 초과 시 윈치가 풀려 장력을 유지  # TODO: 출처 확인 필요
        "closing_delay": 0.5,         # 첫 접촉 후 조임(closing) 시작까지 지연 [s]  # TODO: 출처 확인 필요
        "closing_duration": 2.0,      # 둘레 스프링 자유길이 축소에 걸리는 시간 [s]  # TODO: 출처 확인 필요
        "closing_ratio": 0.15,        # 조임 완료 후 둘레 자유길이 비율  # TODO: 출처 확인 필요
        "min_contact_nodes": 4,       # 결합 유지 판정: 대상과 접촉 중인 질점 최소 개수  # TODO: 출처 확인 필요
        "lateral_friction": 0.6,      # 그물-대상 마찰  # TODO: 출처 확인 필요
        "restitution": 0.0,           # 그물 질점 반발계수 (섬유, 거의 비탄성)  # TODO: 출처 확인 필요
    },
    # ---------------------------------------------------------------- 3) 작살
    "harpoon": {
        "mass": 2.0,                  # 발사체 질량 [kg]  # TODO: 출처 확인 필요
        "radius": 0.03,               # 발사체 반경 [m]
        "length": 0.4,                # 발사체 길이 [m]  # TODO: 출처 확인 필요
        "speed": 12.0,                # 발사 속도 [m/s]  # TODO: 출처 확인 필요
        "aim_error_deg": 0.5,         # 조준 오차 표준편차 [deg] (시드 랜덤)  # TODO: 출처 확인 필요
        "min_penetration_speed": 4.0,  # 관통에 필요한 최소 법선 충돌 속도 [m/s]  # TODO: 출처 확인 필요
        "max_incidence_deg": 45.0,    # 관통 허용 최대 입사각 (표면 법선 기준)  # TODO: 출처 확인 필요
        "anchor_break_force": 3000.0,  # 고정부(미늘) 이탈 한계 힘 [N]  # TODO: 출처 확인 필요
        "tether_length_factor": 1.05,  # 테더 자유 길이 = 초기 이격 거리 x 이 값  # TODO: 출처 확인 필요
        "tether_nodes": 10,           # 테더를 이루는 중간 질점 수 (대상 둘레 감김 표현용)
        # 테더 질점 질량 [kg]. 총 2 kg (~0.18 kg/m) 급 하중 지지 테더를 가정.
        # 감쇠 안정조건 c*dt/m < 0.5 때문에 질점이 가벼우면 충분한 감쇠를 줄 수 없고,
        # 감쇠가 부족하면 대상이 테더에 매달려 계속 진동해 상대 회전이 수렴하지 않는다.
        "tether_node_mass": 0.2,      # TODO: 출처 확인 필요
        "tether_node_radius": 0.02,   # 테더 질점 충돌 반경 [m]
        "tether_k": 500.0,            # 테더 세그먼트 강성 [N/m]  # TODO: 출처 확인 필요
        "tether_c": 30.0,             # 테더 세그먼트 감쇠 [N·s/m]. 테더 진동을 흡수해 상대 회전을 수렴시킨다  # TODO: 출처 확인 필요
        # 윈치: 관통·고정 후 테더를 되감아 장력을 유지한다. 고정점의 모멘트 암을 통해 대상에 감속 토크가 걸린다.
        "winch_enabled": True,
        "winch_start_delay": 0.5,     # 고정 성립 후 되감기 시작까지 지연 [s]  # TODO: 출처 확인 필요
        "winch_reel_rate": 0.35,      # 되감기 속도 [m/s]  # TODO: 출처 확인 필요
        # 최소 자유 길이 비율. 이 값이 크면 되감기 완료 후 테더가 느슨해져 장력이 0 이 되고 감속이 멈춘다.
        # 작게 두면 스프링이 항상 늘어난 상태가 되어 winch_max_tension 으로 제한되는 "정장력 윈치"로 동작한다.
        "winch_min_length_factor": 0.04,  # TODO: 출처 확인 필요
        "winch_max_tension": 250.0,   # 테더 장력 상한 [N]. anchor_break_force 보다 충분히 작아야 고정부가 버틴다  # TODO: 출처 확인 필요
        "bounce_fail_delay": 2.0,     # 튕겨나간 뒤 이 시간 후 조기 종료 [s]
        "use_ccd": True,              # 고속 발사체 터널링 방지용 연속 충돌 검출
    },
}

# 충돌 필터 그룹 (비트마스크). 충돌 조건: (A.group & B.mask) && (B.group & A.mask)
GROUP_TARGET = 1
GROUP_NODE = 2      # 그물·테더 질점: 대상과만 충돌 (질점끼리는 충돌하지 않음)
GROUP_SERVICER = 4  # 서비서 본체·팔: 대상과 자기 자신 그룹과 충돌
GROUP_HARPOON = 8   # 작살: 대상과만 충돌


# =============================================================================
# 수학 유틸
# =============================================================================
def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros(3)


def quat_to_R(q) -> np.ndarray:
    return np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)


def quat_angle_deg(q0, q1) -> float:
    """두 쿼터니언 사이 회전각 [deg]"""
    _, q_rel = p.multiplyTransforms([0, 0, 0], p.invertTransform([0, 0, 0], q0)[1], [0, 0, 0], q1)
    w = min(1.0, abs(q_rel[3]))
    return math.degrees(2.0 * math.acos(w))


def quat_from_z_to(dir_vec: np.ndarray):
    """z축을 dir_vec 방향으로 보내는 쿼터니언"""
    z = np.array([0.0, 0.0, 1.0])
    d = unit(dir_vec)
    c = float(np.dot(z, d))
    if c > 1 - 1e-9:
        return [0, 0, 0, 1]
    if c < -1 + 1e-9:
        return [1, 0, 0, 0]
    axis = unit(np.cross(z, d))
    ang = math.acos(c)
    s = math.sin(ang / 2)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2)]


def random_small_rotation(rng: np.random.Generator, sigma_deg: float):
    if sigma_deg <= 0:
        return [0, 0, 0, 1]
    rotvec = rng.normal(0.0, math.radians(sigma_deg), 3)
    ang = np.linalg.norm(rotvec)
    if ang < 1e-12:
        return [0, 0, 0, 1]
    axis = rotvec / ang
    s = math.sin(ang / 2)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2)]


# =============================================================================
# World
# =============================================================================
class World:
    """PyBullet 연결/스텝 관리"""

    def __init__(self, cfg: Dict, gui: bool = False):
        self.cfg = cfg
        self.gui = gui
        self.dt = cfg["sim"]["timestep"]
        self.t = 0.0
        self.step_count = 0
        self.time_scale = 1.0   # GUI 재생 배속 (1 = 실시간, 0.25 = 슬로모션)
        self.client = p.connect(p.GUI, options="--background_color_red=0 --background_color_green=0 "
                                               "--background_color_blue=0") if gui else p.connect(p.DIRECT)
        p.resetSimulation()
        p.setGravity(*cfg["sim"]["gravity"])
        p.setTimeStep(self.dt)
        p.setPhysicsEngineParameter(
            numSolverIterations=cfg["sim"]["solver_iterations"],
            deterministicOverlappingPairs=1,  # 시드 고정 재현성
        )
        SpacecraftVisuals.reset()
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def step(self):
        p.stepSimulation()
        self.t += self.dt
        self.step_count += 1
        if self.gui and self.cfg["sim"]["gui_realtime"]:
            time.sleep(self.dt / self.time_scale)

    def close(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass


# =============================================================================
# Target (파편)
# =============================================================================
class Target:
    """파편 강체. 형상·질량·관성텐서·초기 회전 설정"""

    def __init__(self, cfg: Dict, kind: str, spin_deg_s: float, spin_mode: str,
                 rng: np.random.Generator, nutation_deg: Optional[float] = None):
        self.cfg = cfg
        self.kind = kind
        self.tcfg = cfg["targets"][kind]
        self.dcfg = cfg["target_dynamics"]
        self.mass = float(self.tcfg["mass"])
        self.spin_deg_s = spin_deg_s
        self.spin_mode = spin_mode
        self.nutation_deg = self.dcfg["nutation_angle_deg"] if nutation_deg is None else nutation_deg
        self.rng = rng
        self.body: int = -1
        self.inertia_diag = np.zeros(3)
        self.bounding_radius = 0.0
        self.grasp_point_body = np.array(self.tcfg["grasp_point"], dtype=float)
        self.grasp_normal_body = unit(np.array(self.tcfg["grasp_normal"], dtype=float))
        self.omega_body_initial = np.zeros(3)
        self._build()

    # ---- 형상 및 관성 ----------------------------------------------------
    @staticmethod
    def _box_inertia(m: float, he: Sequence[float]) -> np.ndarray:
        a, b, c = [2 * h for h in he]
        return m / 12.0 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])

    def _build(self):
        t = self.tcfg
        rgba = [0.75, 0.75, 0.8, 1.0]
        if t["shape"] == "box":
            he = t["half_extents"]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=rgba)
            self.inertia_diag = self._box_inertia(self.mass, he)
            self.bounding_radius = float(np.linalg.norm(he))
        elif t["shape"] == "cylinder":
            r, L = t["radius"], t["length"]
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=L)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=L, rgbaColor=rgba)
            Izz = 0.5 * self.mass * r * r
            Ixx = self.mass / 12.0 * (3 * r * r + L * L)
            self.inertia_diag = np.array([Ixx, Ixx, Izz])
            self.bounding_radius = math.sqrt(r * r + (L / 2) ** 2)
        elif t["shape"] == "bus_with_panels":
            bhe, phe, dy = t["bus_half_extents"], t["panel_half_extents"], t["panel_offset_y"]
            m_panel = t["panel_mass"]
            m_bus = self.mass - 2 * m_panel
            assert m_bus > 0, "패널 질량이 총 질량보다 큼"
            shapes = [p.GEOM_BOX] * 3
            hes = [bhe, phe, phe]
            poss = [[0, 0, 0], [0, dy, 0], [0, -dy, 0]]
            col = p.createCollisionShapeArray(shapeTypes=shapes, halfExtents=hes, collisionFramePositions=poss)
            vis = p.createVisualShapeArray(shapeTypes=shapes, halfExtents=hes, visualFramePositions=poss,
                                           rgbaColors=[rgba, [0.1, 0.2, 0.6, 1], [0.1, 0.2, 0.6, 1]])
            I = self._box_inertia(m_bus, bhe)
            Ip = self._box_inertia(m_panel, phe)
            # 평행축 정리: y축으로 dy 만큼 이격된 패널 → Ixx, Izz 에 m*dy^2 추가
            I += 2 * (Ip + m_panel * np.array([dy * dy, 0.0, dy * dy]))
            self.inertia_diag = I
            self.bounding_radius = float(np.linalg.norm([phe[0], dy + phe[1], max(bhe[2], phe[2])]))
        else:
            raise ValueError(f"unknown target shape {t['shape']}")

        # 세부 외형 (GUI/녹화 시에만; 충돌 형상은 위의 단순 형상 그대로)
        chunks = []
        if VISUAL_DETAIL["enabled"]:
            chunks = SpacecraftVisuals.target(self.kind, t)
            vis = chunks[0][0]
        # 초기 자세: 시드 기반 소섭동
        q0 = random_small_rotation(self.rng, self.dcfg["initial_attitude_jitter_deg"])
        self.body = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=col,
                                      baseVisualShapeIndex=vis, basePosition=[0, 0, 0],
                                      baseOrientation=q0)
        if chunks:
            SpacecraftVisuals.apply_textures(self.body, -1, chunks[0][1])
            SpacecraftVisuals.attach_overlays(self.body, chunks)
        p.changeDynamics(self.body, -1,
                         localInertiaDiagonal=self.inertia_diag.tolist(),
                         restitution=self.dcfg["restitution"],
                         lateralFriction=self.dcfg["lateral_friction"],
                         linearDamping=self.dcfg["linear_damping"],
                         angularDamping=self.dcfg["angular_damping"])
        p.setCollisionFilterGroupMask(self.body, -1, GROUP_TARGET, -1)
        self._set_spin()

    # ---- 초기 회전 -------------------------------------------------------
    def _spin_axis_body(self) -> np.ndarray:
        name = self.dcfg["spin_axis"]
        if name == "major":
            idx = int(np.argmax(self.inertia_diag))
        elif name == "minor":
            idx = int(np.argmin(self.inertia_diag))
        else:
            idx = {"x": 0, "y": 1, "z": 2}[name]
        e = np.zeros(3)
        e[idx] = 1.0
        return e

    def _set_spin(self):
        mc = self.cfg["monte_carlo"]
        rate = math.radians(self.spin_deg_s) * (1.0 + self.rng.normal(0.0, mc["spin_rate_jitter_frac"]))
        axis = quat_to_R(random_small_rotation(self.rng, mc["spin_axis_jitter_deg"])) @ self._spin_axis_body()
        if self.spin_mode == "principal":
            w_body = rate * axis
        elif self.spin_mode == "precession":
            # 각속도 벡터를 회전축에서 장동각만큼 기울임 → 비대칭 관성체에서 토크-프리 세차 발생
            nu = math.radians(self.nutation_deg)
            perp = unit(np.cross(axis, np.array([1.0, 0.0, 0.0])))
            if np.linalg.norm(perp) < 1e-6:
                perp = unit(np.cross(axis, np.array([0.0, 1.0, 0.0])))
            w_body = rate * (math.cos(nu) * axis + math.sin(nu) * perp)
        else:
            raise ValueError("spin_mode must be 'principal' or 'precession'")
        self.omega_body_initial = w_body
        _, q = p.getBasePositionAndOrientation(self.body)
        w_world = quat_to_R(q) @ w_body
        p.resetBaseVelocity(self.body, linearVelocity=[0, 0, 0], angularVelocity=w_world.tolist())

    # ---- 상태 조회 -------------------------------------------------------
    def pose(self):
        pos, q = p.getBasePositionAndOrientation(self.body)
        return np.array(pos), q

    def velocity(self):
        v, w = p.getBaseVelocity(self.body)
        return np.array(v), np.array(w)

    def grasp_point_world(self):
        """파지점 위치·속도·외향 법선 (월드 좌표)"""
        pos, q = self.pose()
        R = quat_to_R(q)
        r = R @ self.grasp_point_body
        v, w = self.velocity()
        return pos + r, v + np.cross(w, r), R @ self.grasp_normal_body


# =============================================================================
# Servicer (서비스 위성) — 팔 링크를 선택적으로 포함하는 멀티바디
# =============================================================================
@dataclass
class ArmSpec:
    link_lengths: Sequence[float]
    joint_axes: Sequence[Sequence[float]]
    link_radius: float
    link_mass: float
    tool_mass: float
    tool_radius: float


class Servicer:
    def __init__(self, cfg: Dict, position: np.ndarray, arm_spec: Optional[ArmSpec] = None,
                 orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0)):
        self.cfg = cfg
        self.scfg = cfg["servicer"]
        self.mass = float(self.scfg["mass"])
        self.he = np.array(self.scfg["half_extents"], dtype=float)
        self.attach_local = np.array(self.scfg["attach_point"], dtype=float)
        self.body = -1
        self.n_arm_joints = 0
        self.tool_link = -1
        self.arm_spec = arm_spec
        self._build(position, list(orientation))
        _, self.q0 = p.getBasePositionAndOrientation(self.body)
        self.max_attitude_error_deg = 0.0

    def _build(self, position: np.ndarray, orientation: List[float]):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.he.tolist())
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=self.he.tolist(), rgbaColor=[0.9, 0.8, 0.2, 1])
        chunks, tool_chunks = [], []
        if VISUAL_DETAIL["enabled"]:
            chunks = SpacecraftVisuals.servicer(self.he)
            vis = chunks[0][0]
        inertia = Target._box_inertia(self.mass, self.he)
        kwargs = dict(baseMass=self.mass, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                      basePosition=position.tolist(), baseOrientation=orientation)
        if self.arm_spec is not None:
            a = self.arm_spec
            n = len(a.link_lengths)
            masses, cols, viss, poss, orns, ipos, iorn, parents, jtypes, jaxes = ([] for _ in range(10))
            q_x = p.getQuaternionFromEuler([0, math.pi / 2, 0])  # 캡슐 z축 → 링크 x축
            for i, L in enumerate(a.link_lengths):
                c = p.createCollisionShape(p.GEOM_CAPSULE, radius=a.link_radius, height=L,
                                           collisionFramePosition=[L / 2, 0, 0], collisionFrameOrientation=q_x)
                v = p.createVisualShape(p.GEOM_CAPSULE, radius=a.link_radius, length=L,
                                        visualFramePosition=[L / 2, 0, 0], visualFrameOrientation=q_x,
                                        rgbaColor=[0.85, 0.85, 0.88, 1] if i % 2 == 0 else [0.3, 0.3, 0.35, 1])
                masses.append(a.link_mass); cols.append(c); viss.append(v)
                # 링크 프레임 = 관절 프레임. 부모 링크 프레임 기준 위치 (검증됨: 부모 관성프레임 오프셋과 무관)
                poss.append(self.attach_local.tolist() if i == 0 else [a.link_lengths[i - 1], 0, 0])
                orns.append([0, 0, 0, 1])
                ipos.append([L / 2, 0, 0]); iorn.append([0, 0, 0, 1])
                parents.append(i)  # 0 = base
                jtypes.append(p.JOINT_REVOLUTE); jaxes.append(list(a.joint_axes[i]))
            # 툴(그리퍼) 링크: 마지막 링크 끝에 고정 관절
            c = p.createCollisionShape(p.GEOM_SPHERE, radius=a.tool_radius)
            v = p.createVisualShape(p.GEOM_SPHERE, radius=a.tool_radius, rgbaColor=[0.9, 0.2, 0.2, 1])
            if VISUAL_DETAIL["enabled"]:
                tool_chunks = SpacecraftVisuals.gripper(a.tool_radius)
                v = tool_chunks[0][0]
            masses.append(a.tool_mass); cols.append(c); viss.append(v)
            poss.append([a.link_lengths[-1], 0, 0]); orns.append([0, 0, 0, 1])
            ipos.append([0, 0, 0]); iorn.append([0, 0, 0, 1])
            parents.append(n); jtypes.append(p.JOINT_FIXED); jaxes.append([0, 0, 1])
            kwargs.update(linkMasses=masses, linkCollisionShapeIndices=cols, linkVisualShapeIndices=viss,
                          linkPositions=poss, linkOrientations=orns, linkInertialFramePositions=ipos,
                          linkInertialFrameOrientations=iorn, linkParentIndices=parents,
                          linkJointTypes=jtypes, linkJointAxis=jaxes)
            self.n_arm_joints = n
            self.tool_link = n
        self.body = p.createMultiBody(**kwargs)
        if chunks:
            SpacecraftVisuals.apply_textures(self.body, -1, chunks[0][1])
            SpacecraftVisuals.attach_overlays(self.body, chunks)
        if tool_chunks:
            SpacecraftVisuals.apply_textures(self.body, self.tool_link, tool_chunks[0][1])
        p.changeDynamics(self.body, -1, localInertiaDiagonal=inertia.tolist(),
                         restitution=self.scfg["restitution"], lateralFriction=self.scfg["lateral_friction"],
                         linearDamping=0.0, angularDamping=0.0)
        p.setCollisionFilterGroupMask(self.body, -1, GROUP_SERVICER, GROUP_TARGET | GROUP_SERVICER)
        for li in range(p.getNumJoints(self.body)):
            p.changeDynamics(self.body, li, linearDamping=0.0, angularDamping=0.0,
                             restitution=self.scfg["restitution"], lateralFriction=self.scfg["lateral_friction"])
            p.setCollisionFilterGroupMask(self.body, li, GROUP_SERVICER, GROUP_TARGET | GROUP_SERVICER)

    def pose(self):
        pos, q = p.getBasePositionAndOrientation(self.body)
        return np.array(pos), q

    def velocity(self):
        v, w = p.getBaseVelocity(self.body)
        return np.array(v), np.array(w)

    def attach_point_world(self):
        pos, q = self.pose()
        r = quat_to_R(q) @ self.attach_local
        v, w = self.velocity()
        return pos + r, v + np.cross(w, r)

    def add_impulse(self, dp: np.ndarray):
        """발사 반동 등 순간 충격량을 본체에 부여"""
        v, w = self.velocity()
        p.resetBaseVelocity(self.body, linearVelocity=(v + dp / self.mass).tolist(), angularVelocity=w.tolist())

    def attitude_error_deg(self) -> float:
        _, q = self.pose()
        return quat_angle_deg(self.q0, q)

    def attitude_control(self):
        """선택적 PD 자세 유지 (반작용휠 토크 한계 반영)"""
        if not self.scfg["attitude_hold"]:
            return
        pos, q = self.pose()
        _, w = self.velocity()
        _, q_err = p.multiplyTransforms([0, 0, 0], q, [0, 0, 0], p.invertTransform([0, 0, 0], self.q0)[1])
        # 오차 회전벡터(월드): 소각 근사 2*vec 부호 처리
        sgn = 1.0 if q_err[3] >= 0 else -1.0
        err_vec = 2.0 * sgn * np.array(q_err[:3])
        tau = -self.scfg["attitude_kp"] * err_vec - self.scfg["attitude_kd"] * w
        n = np.linalg.norm(tau)
        if n > self.scfg["attitude_max_torque"]:
            tau *= self.scfg["attitude_max_torque"] / n
        p.applyExternalTorque(self.body, -1, tau.tolist(), p.WORLD_FRAME)


# =============================================================================
# SpringNetwork : 질점-스프링 망 (그물, 테더 공용)
# =============================================================================
@dataclass
class Anchor:
    """질점 ↔ 강체 위 고정점 사이의 스프링 (인장 전용)"""
    node_idx: int
    body: int
    local_point: np.ndarray   # 강체 동체 좌표의 고정점
    rest_length: float
    k: float
    c: float
    max_force: float = float("inf")   # 장력 상한 [N] (윈치 슬립 모델)


class SpringNetwork:
    """
    명시적 Kelvin-Voigt 스프링-댐퍼 질점계.
        F = k*(|d|-L0) + c*(v_rel·u) ,  인장 전용이면 |d|<L0 일 때 0
    매 스텝 pre_step 에서 힘을 계산해 applyExternalForce 로 적용한다 (PyBullet 반-암시적 오일러).
    안정 조건(대략): k*dt^2/m < 0.25
    """

    def __init__(self, world: World, positions: np.ndarray, masses: np.ndarray, radii: np.ndarray,
                 springs: List[Tuple[int, int, float, float, float]], tension_only: bool,
                 friction: float, restitution: float, color=(0.2, 0.9, 0.3, 1.0)):
        self.world = world
        self.n = len(positions)
        self.masses = np.asarray(masses, dtype=float)
        self.tension_only = tension_only
        self.bodies: List[int] = []
        for i in range(self.n):
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=float(radii[i]))
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=float(radii[i]), rgbaColor=list(color)) if world.gui else -1
            b = p.createMultiBody(baseMass=float(self.masses[i]), baseCollisionShapeIndex=col,
                                  baseVisualShapeIndex=vis, basePosition=positions[i].tolist())
            p.changeDynamics(b, -1, linearDamping=0.0, angularDamping=0.0,
                             lateralFriction=friction, restitution=restitution)
            p.setCollisionFilterGroupMask(b, -1, GROUP_NODE, GROUP_TARGET)
            self.bodies.append(b)
        sp = np.array(springs, dtype=float)
        self.si = sp[:, 0].astype(int)
        self.sj = sp[:, 1].astype(int)
        self.L0 = sp[:, 2].copy()
        self.L0_initial = sp[:, 2].copy()
        self.k = sp[:, 3]
        self.c = sp[:, 4]
        self.anchors: List[Anchor] = []
        self.tension_cap = float("inf")   # 세그먼트 장력 상한 [N] (윈치 슬립 모델)
        self.debug_ids: List[int] = []
        self._refresh_anchor_arrays()
        # 안정성 경고
        dt = world.dt
        ratio = np.max(self.k * dt * dt / np.minimum(self.masses[self.si], self.masses[self.sj]))
        if ratio > 0.25:
            print(f"[warn] spring stiffness may be unstable: k*dt^2/m = {ratio:.3f} (> 0.25)", file=sys.stderr)
        ratio_c = np.max(self.c * dt / np.minimum(self.masses[self.si], self.masses[self.sj]))
        if ratio_c > 0.5:
            print(f"[warn] spring damping may be unstable: c*dt/m = {ratio_c:.3f} (> 0.5)", file=sys.stderr)

    # ---- 상태 -----------------------------------------------------------
    def gather(self):
        pos = np.empty((self.n, 3)); vel = np.empty((self.n, 3))
        for i, b in enumerate(self.bodies):
            pos[i] = p.getBasePositionAndOrientation(b)[0]
            vel[i] = p.getBaseVelocity(b)[0]
        return pos, vel

    def set_velocity(self, vel: np.ndarray):
        for i, b in enumerate(self.bodies):
            p.resetBaseVelocity(b, linearVelocity=vel[i].tolist(), angularVelocity=[0, 0, 0])

    def momentum(self) -> np.ndarray:
        _, vel = self.gather()
        return (self.masses[:, None] * vel).sum(axis=0)

    # ---- 힘 ------------------------------------------------------------
    def _spring_force(self, d: np.ndarray, dv: np.ndarray, L0, k, c):
        dist = np.linalg.norm(d, axis=1)
        u = d / np.maximum(dist, 1e-9)[:, None]
        ext = dist - L0
        vrel = np.einsum("ij,ij->i", dv, u)
        F = k * ext + c * vrel
        if self.tension_only:
            F = np.where(ext > 0, np.maximum(F, 0.0), 0.0)
        return F[:, None] * u, dist

    def apply_forces(self):
        pos, vel = self.gather()
        d = pos[self.sj] - pos[self.si]
        dv = vel[self.sj] - vel[self.si]
        Fv, _ = self._spring_force(d, dv, self.L0, self.k, self.c)
        if np.isfinite(self.tension_cap):
            mag = np.linalg.norm(Fv, axis=1)
            over = mag > self.tension_cap
            if np.any(over):
                Fv[over] *= (self.tension_cap / mag[over])[:, None]
        # 스프링 힘을 질점별로 합산. np.add.at 은 같은 일을 하지만 수십 배 느려 시뮬레이션 전체의 병목이 된다.
        F = np.empty((self.n, 3))
        for a in range(3):
            F[:, a] = (np.bincount(self.si, weights=Fv[:, a], minlength=self.n)
                       - np.bincount(self.sj, weights=Fv[:, a], minlength=self.n))
        # 앵커(강체 고정점) 스프링. 앵커마다 강체 상태를 따로 읽고 1원소 배열로 계산하면
        # 파이썬/넘파이 호출 오버헤드가 스텝 비용의 20 % 를 넘으므로, 강체별로 한 번만 읽고 벡터화한다.
        if self.anchors:
            bodies = {a.body for a in self.anchors}
            state = {}
            for b in bodies:
                bpos, bq = p.getBasePositionAndOrientation(b)
                bv, bw = p.getBaseVelocity(b)
                state[b] = (np.array(bpos), quat_to_R(bq), np.array(bv), np.array(bw))
            na = len(self.anchors)
            ap = np.empty((na, 3)); av = np.empty((na, 3))
            for i, a in enumerate(self.anchors):
                bpos, R, bv, bw = state[a.body]
                r = R @ a.local_point
                ap[i] = bpos + r
                av[i] = bv + np.cross(bw, r)
            Fa, _ = self._spring_force(ap - pos[self.anchor_nodes], av - vel[self.anchor_nodes],
                                       self.anchor_L0, self.anchor_k, self.anchor_c)
            mag = np.linalg.norm(Fa, axis=1)
            over = mag > self.anchor_max_force      # 윈치 장력 상한: 초과분은 윈치가 풀리며 흡수
            if np.any(over):
                Fa[over] *= (self.anchor_max_force[over] / mag[over])[:, None]
            for a in range(3):
                F[:, a] += np.bincount(self.anchor_nodes, weights=Fa[:, a], minlength=self.n)
            Fa_react = (-Fa).tolist(); ap_l = ap.tolist()
            for i, a in enumerate(self.anchors):
                p.applyExternalForce(a.body, -1, Fa_react[i], ap_l[i], p.WORLD_FRAME)
        # tolist 를 질점마다 부르면 호출 오버헤드가 커서 한 번에 변환한다 (np.any 검사도 제거).
        F_l = F.tolist(); pos_l = pos.tolist()
        for i, b in enumerate(self.bodies):
            p.applyExternalForce(b, -1, F_l[i], pos_l[i], p.WORLD_FRAME)
        return pos

    def _refresh_anchor_arrays(self):
        """앵커 파라미터를 벡터 연산용 배열로 캐싱. 앵커를 추가하거나 rest_length 를 바꾼 뒤 호출한다."""
        self.anchor_nodes = np.array([a.node_idx for a in self.anchors], dtype=int)
        self.anchor_L0 = np.array([a.rest_length for a in self.anchors], dtype=float)
        self.anchor_k = np.array([a.k for a in self.anchors], dtype=float)
        self.anchor_c = np.array([a.c for a in self.anchors], dtype=float)
        self.anchor_max_force = np.array([a.max_force for a in self.anchors], dtype=float)

    def contact_count_with(self, body: int) -> int:
        cnt = 0
        for b in self.bodies:
            if p.getContactPoints(bodyA=b, bodyB=body):
                cnt += 1
        return cnt

    def draw(self, pos: np.ndarray):
        if not self.world.gui or not self.world.cfg["sim"]["debug_lines"]:
            return
        if self.world.step_count % self.world.cfg["sim"]["debug_line_interval"]:
            return
        first = not self.debug_ids
        for n, (i, j) in enumerate(zip(self.si, self.sj)):
            if first:
                self.debug_ids.append(p.addUserDebugLine(pos[i].tolist(), pos[j].tolist(), [0.1, 0.8, 0.2], 1.5))
            else:
                p.addUserDebugLine(pos[i].tolist(), pos[j].tolist(), [0.1, 0.8, 0.2], 1.5,
                                   replaceItemUniqueId=self.debug_ids[n])


# =============================================================================
# TetherWinch : 테더 되감기 (그물·작살 공용)
# =============================================================================
class TetherWinch:
    """
    결합 성립 후 테더의 자유 길이를 일정 속도로 줄여 장력을 유지한다.
    테더가 대상의 고정점(또는 그물 모서리)에 걸린 상태에서 장력 x 모멘트 암 = 대상에 걸리는 감속 토크가 되고,
    반작용으로 서비서가 회전하여 두 물체의 상대 각속도가 줄어든다(각운동량은 계 전체에서 보존).
    장력 상한을 두어 고정부 파손과 수치 발산을 막는다.
    """

    def __init__(self, cfg: Dict, initial_length: float):
        self.enabled = bool(cfg["winch_enabled"])
        self.delay = float(cfg["winch_start_delay"])
        self.rate = float(cfg["winch_reel_rate"])
        self.max_tension = float(cfg["winch_max_tension"])
        self.L0 = float(initial_length)
        self.L_min = self.L0 * float(cfg["winch_min_length_factor"])
        self.length = self.L0
        self.started: Optional[float] = None

    def update(self, t: float, bond_time: Optional[float]) -> float:
        """현재 테더 자유 길이를 반환. bond_time 은 결합이 성립한 시각(None 이면 대기)"""
        if not self.enabled or bond_time is None:
            return self.length
        if self.started is None:
            if t < bond_time + self.delay:
                return self.length
            self.started = t
        self.length = max(self.L_min, self.L0 - self.rate * (t - self.started))
        return self.length


# =============================================================================
# CaptureMethod 공통 인터페이스
# =============================================================================
class CaptureMethod(ABC):
    name: str = "base"

    def __init__(self, cfg: Dict, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.world: World
        self.servicer: Servicer
        self.target: Target
        self.first_contact_time: Optional[float] = None
        self.bond_established_time: Optional[float] = None
        self.bond_lost_time: Optional[float] = None
        self.max_interaction_force = 0.0  # 제약(파지/고정부)에 걸린 최대 힘 [N]
        self.notes: List[str] = []

    # ---- 배치: 서비서 중심 위치와 팔 스펙 ------------------------------
    @abstractmethod
    def servicer_position(self, target: Target) -> np.ndarray: ...

    def arm_spec(self) -> Optional[ArmSpec]:
        return None

    @abstractmethod
    def setup(self, world: World, servicer: Servicer, target: Target): ...

    @abstractmethod
    def pre_step(self): ...

    @abstractmethod
    def post_step(self, target_contacts: list): ...

    @abstractmethod
    def bond_intact(self) -> bool: ...

    def should_abort(self) -> bool:
        """더 이상 포획 가능성이 없어 조기 종료해도 되는지"""
        return False

    def device_bodies(self) -> List[int]:
        """대상과의 '첫 접촉' 판정에 포함할 포획 장치 바디 목록"""
        return [self.servicer.body]

    def extra_metrics(self) -> Dict:
        return {}

    def status_text(self) -> str:
        """HUD 표시용 현재 상태 문자열"""
        return ""

    def orbit_bodies(self) -> List[Tuple[int, float]]:
        """궤도 상대 역학(CW)을 적용할 방식 고유 강체 목록 [(body, mass)]. 그물/테더 질점은 기본 제외"""
        return []

    def trajectory_static(self) -> Dict:
        """웹 뷰어용 정적 모델 정보 (형상, 스프링 연결 등)"""
        return {}

    def trajectory_frame(self) -> Dict:
        """웹 뷰어용 프레임별 상태 (링크 자세, 질점 위치 등)"""
        return {}

    # 공통: 첫 접촉 시각 갱신
    def _update_first_contact(self, target_contacts: list):
        if self.first_contact_time is not None:
            return
        devs = set(self.device_bodies())
        for c in target_contacts:
            other = c[2] if c[1] == self.target.body else c[1]
            if other in devs:
                self.first_contact_time = self.world.t
                return


# -----------------------------------------------------------------------------
# 1) 로봇팔
# -----------------------------------------------------------------------------
class RobotArmCapture(CaptureMethod):
    name = "arm"

    def __init__(self, cfg, rng):
        super().__init__(cfg, rng)
        self.acfg = cfg["arm"]
        self.reach = float(sum(self.acfg["link_lengths"]))
        self.phase = "preapproach"   # preapproach → approach → close → grasped
        self.constraint: Optional[int] = None
        self.grasp_child_frame = None
        self.grasp_time = 0.0
        self.lost_count = 0
        self.cmd_pos: Optional[np.ndarray] = None  # 데카르트 보간 중인 현재 명령 위치

    def arm_spec(self) -> ArmSpec:
        a = self.acfg
        return ArmSpec(a["link_lengths"], a["joint_axes"], a["link_radius"], a["link_mass"],
                       a["tool_mass"], a["tool_radius"])

    def servicer_position(self, target: Target) -> np.ndarray:
        # 대기점(파지점 + 법선*standoff)이 어깨에서 reach*fraction 거리에 오도록 배치하되,
        # 서비서 본체(+x 면 = 어깨)는 대상의 외접구(회전 시 쓸고 가는 영역) 밖에 두어 본체 충돌을 배제
        gp = target.grasp_point_body
        standoff_pt_x = gp[0] - self.acfg["approach_standoff"]
        shoulder_x = standoff_pt_x - self.acfg["reach_fraction_at_standoff"] * self.reach
        shoulder_x = min(shoulder_x, -(target.bounding_radius + self.cfg["standoff"]["arm"]))
        center_x = shoulder_x - self.cfg["servicer"]["attach_point"][0]
        return np.array([center_x, 0.0, 0.0])

    def setup(self, world, servicer, target):
        self.world, self.servicer, self.target = world, servicer, target
        a = self.acfg
        self.joints = list(range(servicer.n_arm_joints))
        lim = math.radians(a["joint_limit_deg"])
        self.ll = [-lim] * len(self.joints); self.ul = [lim] * len(self.joints)
        self.jr = [2 * lim] * len(self.joints); self.rp = list(a["ready_pose"])
        for j in self.joints:
            p.resetJointState(servicer.body, j, self.rp[j])
            p.changeDynamics(servicer.body, j, jointLowerLimit=-lim, jointUpperLimit=lim,
                             jointDamping=a["joint_damping"])
        self._command_joints(self.rp)

    def device_bodies(self):
        return [self.servicer.body]

    # ---- 제어 ----------------------------------------------------------
    def _tool_state(self):
        s = p.getLinkState(self.servicer.body, self.servicer.tool_link, computeLinkVelocity=1,
                           computeForwardKinematics=1)
        return np.array(s[4]), np.array(s[6]), s[5]

    def _shoulder_pos(self):
        s = p.getLinkState(self.servicer.body, 0, computeForwardKinematics=1)
        return np.array(s[4])

    def _command_joints(self, q: Sequence[float]):
        a = self.acfg
        for j in self.joints:
            p.setJointMotorControl2(self.servicer.body, j, p.POSITION_CONTROL, targetPosition=float(q[j]),
                                    force=a["joint_max_torque"], maxVelocity=a["joint_max_velocity"],
                                    positionGain=a["position_gain"])

    def _brake_joints(self):
        for j in self.joints:
            p.setJointMotorControl2(self.servicer.body, j, p.VELOCITY_CONTROL, targetVelocity=0.0,
                                    force=self.acfg["joint_max_torque"])

    def _track(self, goal: np.ndarray):
        """그리퍼 명령점을 goal 방향으로 tool_max_speed 이하로 이동시키며 IK 추종"""
        if self.cmd_pos is None:
            self.cmd_pos = self._tool_state()[0]
        step = self.acfg["tool_max_speed"] * self.world.dt
        d = goal - self.cmd_pos
        n = float(np.linalg.norm(d))
        self.cmd_pos = goal.copy() if n <= step else self.cmd_pos + d / n * step
        desired = self.cmd_pos
        q = p.calculateInverseKinematics(self.servicer.body, self.servicer.tool_link, desired.tolist(),
                                         lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr,
                                         restPoses=self.rp, maxNumIterations=50, residualThreshold=1e-4)
        self._command_joints(q[:len(self.joints)])

    def pre_step(self):
        a = self.acfg
        gp, gv, gn = self.target.grasp_point_world()
        gp = gp + gn * a["tool_radius"]  # 그리퍼(구) 중심이 표면에 접하는 위치를 파지 목표로 사용
        if self.phase == "grasped":
            # 그리퍼 닫힘: 제약 힘 한계를 선형 증가 → 한 스텝 강체 결합에 의한 비물리적 충격 방지
            frac = min(1.0, (self.world.t - self.grasp_time) / max(a["grasp_closure_time"], 1e-6))
            p.changeConstraint(self.constraint, maxForce=a["grasp_force_limit"] * max(frac, 0.01))
            if frac < 1.0:
                self._track(gp)          # 닫히는 동안 팔은 파지점을 계속 추종
            elif a["post_grasp_mode"] == "brake":
                self._brake_joints()     # 닫힘 완료 후 관절 브레이크 (토크 제한)
            else:
                self._track(gp)
            return
        tool_pos, tool_vel, _ = self._tool_state()
        shoulder = self._shoulder_pos()
        standoff_pt = gp + gn * a["approach_standoff"]
        pre_pt = standoff_pt + gn * a["pre_approach_distance"]
        reachable = (np.linalg.norm(standoff_pt - shoulder) < self.reach * a["reach_margin"]
                     and np.dot(gn, shoulder - gp) > 0)
        # 접근 원뿔: 파지점 법선이 어깨를 향하는 각도가 작을 때만 접근. 아니면 대상 정면의 안전 대기점 유지
        cos_ang = float(np.dot(gn, unit(shoulder - gp)))
        in_cone = reachable and cos_ang > math.cos(math.radians(a["approach_cone_deg"]))
        tpos, _ = self.target.pose()
        wait_pt = tpos + unit(shoulder - tpos) * (float(np.linalg.norm(self.target.grasp_point_body))
                                                  + a["tool_radius"] + a["approach_standoff"] + a["pre_approach_distance"])
        if self.phase == "preapproach":
            goal = pre_pt if in_cone else wait_pt
            self._track(goal)
            if in_cone and np.linalg.norm(tool_pos - pre_pt) < a["approach_tolerance"]:
                self.phase = "approach"
        elif self.phase == "approach":
            if not in_cone:
                self.phase = "preapproach"
                return
            self._track(standoff_pt)
            if np.linalg.norm(tool_pos - standoff_pt) < a["approach_tolerance"]:
                self.phase = "close"
        elif self.phase == "close":
            if not in_cone or np.linalg.norm(tool_pos - gp) > 2 * a["approach_standoff"]:
                self.phase = "preapproach"
                return
            self._track(gp)
            if (np.linalg.norm(tool_pos - gp) < a["grasp_tolerance"]
                    and np.linalg.norm(tool_vel - gv) < a["grasp_max_rel_speed"]):
                self._grasp()

    def _grasp(self):
        """현재 상대 자세 그대로 고정 제약 생성 (충격 없는 파지)"""
        tool_pos, _, tool_q = self._tool_state()
        tpos, tq = self.target.pose()
        inv_p, inv_q = p.invertTransform(tpos.tolist(), tq)
        child_pos, child_q = p.multiplyTransforms(inv_p, inv_q, tool_pos.tolist(), tool_q)
        self.constraint = p.createConstraint(self.servicer.body, self.servicer.tool_link, self.target.body, -1,
                                             p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], child_pos,
                                             [0, 0, 0, 1], child_q)
        p.changeConstraint(self.constraint, maxForce=self.acfg["grasp_force_limit"] * 0.01)
        self.grasp_child_frame = np.array(child_pos)
        self.grasp_time = self.world.t
        self.phase = "grasped"
        self.bond_established_time = self.world.t
        if self.first_contact_time is None:
            self.first_contact_time = self.world.t

    def post_step(self, target_contacts):
        self._update_first_contact(target_contacts)
        if self.constraint is None:
            return
        a = self.acfg
        st = p.getConstraintState(self.constraint)
        F = float(np.linalg.norm(st[:3]))
        self.max_interaction_force = max(self.max_interaction_force, F)
        # 유지력 한계 초과 요구 → 제약이 미끄러짐 → 파지점 드리프트가 허용치를 넘으면 이탈
        tool_pos, _, _ = self._tool_state()
        tpos, tq = self.target.pose()
        gp_world = tpos + quat_to_R(tq) @ self.grasp_child_frame
        slip = float(np.linalg.norm(tool_pos - gp_world))
        self.max_slip = max(getattr(self, "max_slip", 0.0), slip)
        if slip > a["slip_tolerance"]:
            p.removeConstraint(self.constraint)
            self.constraint = None
            self.phase = "preapproach"
            self.bond_lost_time = self.world.t
            self.lost_count += 1
            self.notes.append(f"grasp lost at t={self.world.t:.2f}s (F={F:.0f}N, slip={slip:.3f}m)")

    def bond_intact(self) -> bool:
        return self.constraint is not None

    def status_text(self):
        return f"phase={self.phase} lost={self.lost_count}"

    def trajectory_static(self):
        a = self.acfg
        return {"arm": {"link_lengths": list(a["link_lengths"]), "link_radius": a["link_radius"],
                        "tool_radius": a["tool_radius"],
                        "gripper_parts": SpacecraftVisuals.parts_to_json(SpacecraftVisuals.gripper_parts(a["tool_radius"]))}}

    def trajectory_frame(self):
        n = self.servicer.n_arm_joints
        states = p.getLinkStates(self.servicer.body, list(range(n + 1)), computeForwardKinematics=1)
        flat = []
        for st in states:
            flat.extend(st[4]); flat.extend(st[5])
        return {"arm": flat}

    def extra_metrics(self):
        return {"grasp_lost_count": self.lost_count, "phase": self.phase,
                "max_slip": getattr(self, "max_slip", 0.0)}


# -----------------------------------------------------------------------------
# 2) 그물
# -----------------------------------------------------------------------------
class NetCapture(CaptureMethod):
    name = "net"

    def __init__(self, cfg, rng):
        super().__init__(cfg, rng)
        self.ncfg = cfg["net"]
        self.net: Optional[SpringNetwork] = None
        self.closing_started: Optional[float] = None
        self.perimeter_mask = None
        self.contact_nodes = 0

    def servicer_position(self, target: Target) -> np.ndarray:
        d = self.cfg["standoff"]["net"] + target.bounding_radius + self.cfg["servicer"]["half_extents"][0]
        return np.array([-d, 0.0, 0.0])

    def setup(self, world, servicer, target):
        self.world, self.servicer, self.target = world, servicer, target
        n = self.ncfg["grid_n"]
        size = self.ncfg["size_by_target"][target.kind]
        spacing = size / (n - 1)
        # 격자 좌표 (전개 완료 기준, 로컬 y-z 평면)
        g = np.linspace(-size / 2, size / 2, n)
        gy, gz = np.meshgrid(g, g, indexing="ij")
        local = np.stack([np.zeros(n * n), gy.ravel(), gz.ravel()], axis=1)
        idx = lambda i, j: i * n + j
        # 질량: 면밀도 x 면적 / 질점수. 모서리 4점은 전개용 추라 더 무겁다.
        node_mass = self.ncfg["areal_density"] * size * size / (n * n)
        corner_mass = node_mass * self.ncfg["corner_mass_multiple"]
        masses = np.full(n * n, node_mass)
        corners = [idx(0, 0), idx(0, n - 1), idx(n - 1, 0), idx(n - 1, n - 1)]
        masses[corners] = corner_mass
        radii = np.full(n * n, spacing * self.ncfg["node_radius_ratio"])
        # 스프링 계수를 안정조건에서 유도 (가장 가벼운 질점 기준)
        dt = world.dt
        k = self.ncfg["spring_k_factor"] * node_mass / (dt * dt)
        c = self.ncfg["spring_c_factor"] * node_mass / dt
        self.node_mass, self.corner_mass = node_mass, corner_mass
        self.net_total_mass = float(masses.sum())
        springs = []
        perim = []
        for i in range(n):
            for j in range(n):
                if i + 1 < n:
                    springs.append((idx(i, j), idx(i + 1, j), spacing, k, c))
                    perim.append(j == 0 or j == n - 1)
                if j + 1 < n:
                    springs.append((idx(i, j), idx(i, j + 1), spacing, k, c))
                    perim.append(i == 0 or i == n - 1)
                if self.ncfg["shear_springs"] and i + 1 < n and j + 1 < n:
                    springs.append((idx(i, j), idx(i + 1, j + 1), spacing * math.sqrt(2), k, c)); perim.append(False)
                    springs.append((idx(i + 1, j), idx(i, j + 1), spacing * math.sqrt(2), k, c)); perim.append(False)
        self.perimeter_mask = np.array(perim)

        # 발사 방향 (시드 랜덤 오차 포함)
        att_pos, att_vel = servicer.attach_point_world()
        tpos, _ = target.pose()
        dir0 = unit(tpos - att_pos)
        q_err = random_small_rotation(self.rng, self.ncfg["launch_angle_error_deg"])
        launch_dir = quat_to_R(q_err) @ dir0
        R = quat_to_R(quat_from_z_to(launch_dir)) @ quat_to_R(p.getQuaternionFromEuler([0, math.pi / 2, 0])).T
        # 위 R: 로컬 x축(그물 법선)을 launch_dir 로 보내는 회전
        center = att_pos + launch_dir * self.ncfg["launch_offset"]
        folded = local * self.ncfg["folded_ratio"]
        positions = center + (R @ folded.T).T
        # 초기 속도: 전방 + 모서리로 갈수록 큰 측방 속도 (비행 중 전개)
        flight = float(np.linalg.norm(tpos - center)) - target.bounding_radius
        t_open = self.ncfg["open_at_fraction"] * flight / self.ncfg["deploy_speed"]
        v_spread_corner = (size / 2 * math.sqrt(2)) * (1 - self.ncfg["folded_ratio"]) / max(t_open, 1e-3)
        radial = local[:, 1:] / (size / 2 * math.sqrt(2))  # 모서리에서 크기 1
        v_local = np.concatenate([np.full((n * n, 1), self.ncfg["deploy_speed"]), radial * v_spread_corner], axis=1)
        v_world = (R @ v_local.T).T + att_vel
        self.net = SpringNetwork(world, positions, masses, radii, springs, self.ncfg["tension_only"],
                                 self.ncfg["lateral_friction"], self.ncfg["restitution"])
        self.net.set_velocity(v_world)
        # 발사 반동
        servicer.add_impulse(-(masses[:, None] * (v_world - att_vel)).sum(axis=0))
        # 모서리 4점 ↔ 서비서 발사구 테더 (인장 전용)
        d0 = self.cfg["standoff"]["net"] + target.bounding_radius
        L_tether = self.ncfg["tether_length_factor"] * (d0 + target.bounding_radius)
        tether_k = self.ncfg["tether_k_factor"] * corner_mass / (dt * dt)
        tether_c = self.ncfg["tether_c_factor"] * corner_mass / dt
        for ci in corners:
            self.net.anchors.append(Anchor(ci, servicer.body, servicer.attach_local.copy(), L_tether,
                                           tether_k, tether_c, self.ncfg["winch_max_tension"]))
        self.net._refresh_anchor_arrays()
        self.tether_length = L_tether
        self.winch = TetherWinch(self.ncfg, L_tether)
        self.size = size

    def device_bodies(self):
        return self.net.bodies

    def pre_step(self):
        # 조임 기구: 첫 접촉 후 지연 시간 뒤 둘레 스프링 자유길이를 서서히 축소
        if self.first_contact_time is not None and self.closing_started is None \
                and self.world.t >= self.first_contact_time + self.ncfg["closing_delay"]:
            self.closing_started = self.world.t
        if self.closing_started is not None:
            s = min(1.0, (self.world.t - self.closing_started) / self.ncfg["closing_duration"])
            ratio = 1.0 + (self.ncfg["closing_ratio"] - 1.0) * s
            self.net.L0[self.perimeter_mask] = self.net.L0_initial[self.perimeter_mask] * ratio
        # 윈치 되감기: 결합 성립 후 테더 자유 길이를 줄여 장력을 유지 → 대상에 감속 토크
        self.tether_length = self.winch.update(self.world.t, self.bond_established_time)
        for a in self.net.anchors:
            a.rest_length = self.tether_length
        self.net.anchor_L0[:] = self.tether_length
        pos = self.net.apply_forces()
        self.net.draw(pos)

    def post_step(self, target_contacts):
        self._update_first_contact(target_contacts)
        nodes = set(self.net.bodies)
        touching = set()
        for c in target_contacts:
            other = c[2] if c[1] == self.target.body else c[1]
            if other in nodes:
                touching.add(other)
        self.contact_nodes = len(touching)
        intact = self.bond_intact()
        if intact and self.bond_established_time is None:
            self.bond_established_time = self.world.t
        if not intact and self.bond_established_time is not None and self.bond_lost_time is None:
            self.bond_lost_time = self.world.t

    def bond_intact(self) -> bool:
        if self.contact_nodes < self.ncfg["min_contact_nodes"]:
            return False
        tpos, _ = self.target.pose()
        spos, _ = self.servicer.pose()
        return bool(np.linalg.norm(tpos - spos) < self.tether_length + self.size)

    def should_abort(self) -> bool:
        # 그물이 대상을 완전히 지나쳤는데 한 번도 접촉 못 한 경우
        if self.first_contact_time is None:
            pos, _ = self.net.gather()
            tpos, _ = self.target.pose()
            if pos[:, 0].min() > tpos[0] + self.target.bounding_radius + self.size:
                self.notes.append("net missed the target")
                return True
        return False

    def status_text(self):
        return f"contact_nodes={self.contact_nodes} closing={'on' if self.closing_started is not None else 'off'}"

    def trajectory_static(self):
        net = self.net
        radii = [float(p.getCollisionShapeData(b, -1)[0][3][0]) for b in net.bodies[:1]]
        return {"net": {"n": net.n, "si": net.si.tolist(), "sj": net.sj.tolist(),
                        "node_radius": radii[0] if radii else 0.05,
                        "anchors": [{"node": a.node_idx, "local": a.local_point.tolist()} for a in net.anchors],
                        "perimeter": self.perimeter_mask.tolist()}}

    def trajectory_frame(self):
        pos, _ = self.net.gather()
        return {"net": pos.ravel().tolist()}

    def extra_metrics(self):
        return {"contact_nodes": self.contact_nodes,
                "closing_started": self.closing_started,
                "net_total_mass_kg": getattr(self, "net_total_mass", None)}


# -----------------------------------------------------------------------------
# 3) 작살
# -----------------------------------------------------------------------------
class HarpoonCapture(CaptureMethod):
    name = "harpoon"

    def __init__(self, cfg, rng):
        super().__init__(cfg, rng)
        self.hcfg = cfg["harpoon"]
        self.harpoon = -1
        self.tether: Optional[SpringNetwork] = None
        self.anchor_constraint: Optional[int] = None
        self.state = "flying"  # flying / anchored / bounced / torn
        self.state_time = 0.0

    def servicer_position(self, target: Target) -> np.ndarray:
        d = self.cfg["standoff"]["harpoon"] + target.bounding_radius + self.cfg["servicer"]["half_extents"][0]
        return np.array([-d, 0.0, 0.0])

    def setup(self, world, servicer, target):
        self.world, self.servicer, self.target = world, servicer, target
        h = self.hcfg
        att_pos, att_vel = servicer.attach_point_world()
        gp, _, _ = target.grasp_point_world()
        dir0 = unit(gp - att_pos)
        launch_dir = quat_to_R(random_small_rotation(self.rng, h["aim_error_deg"])) @ dir0
        col = p.createCollisionShape(p.GEOM_CAPSULE, radius=h["radius"], height=h["length"])
        vis = p.createVisualShape(p.GEOM_CAPSULE, radius=h["radius"], length=h["length"], rgbaColor=[0.9, 0.3, 0.1, 1])
        start = att_pos + launch_dir * (h["length"] / 2 + 0.05)
        q = quat_from_z_to(launch_dir)
        self.harpoon = p.createMultiBody(baseMass=h["mass"], baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                         basePosition=start.tolist(), baseOrientation=q)
        p.changeDynamics(self.harpoon, -1, linearDamping=0.0, angularDamping=0.0,
                         lateralFriction=self.cfg["target_dynamics"]["lateral_friction"], restitution=0.1)
        if h["use_ccd"]:
            p.changeDynamics(self.harpoon, -1, ccdSweptSphereRadius=h["radius"])
        p.setCollisionFilterGroupMask(self.harpoon, -1, GROUP_HARPOON, GROUP_TARGET)
        v_launch = launch_dir * h["speed"] + att_vel
        p.resetBaseVelocity(self.harpoon, linearVelocity=v_launch.tolist(), angularVelocity=[0, 0, 0])
        servicer.add_impulse(-h["mass"] * launch_dir * h["speed"])
        self.launch_dir = launch_dir
        # 테더: 서비서 발사구 ↔ 질점 체인 ↔ 작살 꼬리. 초기에는 발사구 근처에 느슨하게 뭉쳐 있음
        d0 = self.cfg["standoff"]["harpoon"] + target.bounding_radius
        self.tether_length = h["tether_length_factor"] * d0
        nn = h["tether_nodes"]
        seg = self.tether_length / (nn + 1)
        pos = att_pos + launch_dir * 0.2 + self.rng.normal(0, 0.02, (nn, 3))
        masses = np.full(nn, h["tether_node_mass"]); radii = np.full(nn, h["tether_node_radius"])
        springs = [(i, i + 1, seg, h["tether_k"], h["tether_c"]) for i in range(nn - 1)]
        self.tether = SpringNetwork(world, pos, masses, radii, springs, True,
                                    self.cfg["target_dynamics"]["lateral_friction"], 0.0, color=(0.9, 0.6, 0.1, 1))
        self.tether.set_velocity(np.tile(att_vel, (nn, 1)))
        self.tether.anchors.append(Anchor(0, servicer.body, servicer.attach_local.copy(), seg,
                                          h["tether_k"], h["tether_c"], h["winch_max_tension"]))
        self.tether.anchors.append(Anchor(nn - 1, self.harpoon, np.array([0, 0, -h["length"] / 2]), seg,
                                          h["tether_k"], h["tether_c"], h["winch_max_tension"]))
        self.tether._refresh_anchor_arrays()
        self.seg = seg
        self.tether.tension_cap = h["winch_max_tension"]
        self.winch = TetherWinch(h, self.tether_length)

    def device_bodies(self):
        return [self.harpoon]

    def pre_step(self):
        # 충돌 판정용: 스텝 이전(충돌 전) 발사체 속도·자세 저장 (스텝 후 값은 이미 반발된 속도)
        self._hv_prev = np.array(p.getBaseVelocity(self.harpoon)[0])
        self._hq_prev = p.getBasePositionAndOrientation(self.harpoon)[1]
        self._target_v_prev = self.target.velocity()
        # 윈치 되감기: 고정 성립 후 테더 전체 자유 길이를 줄인다 (세그먼트와 앵커를 같은 비율로)
        if self.state == "anchored":
            L = self.winch.update(self.world.t, self.bond_established_time)
            scale = L / self.tether_length if self.tether_length > 0 else 1.0
            if abs(scale - 1.0) > 1e-9:
                self.tether.L0[:] = self.tether.L0_initial * scale
                for a in self.tether.anchors:
                    a.rest_length = self.seg * scale
                self.tether.anchor_L0[:] = self.seg * scale
        pos = self.tether.apply_forces()
        self.tether.draw(pos)

    def post_step(self, target_contacts):
        self._update_first_contact(target_contacts)
        h = self.hcfg
        if self.state == "flying":
            hits = [c for c in target_contacts if self.harpoon in (c[1], c[2])]
            if not hits:
                return
            c = hits[0]
            # 접촉 법선: contactNormalOnB 는 B→A 방향. A=target 이면 부호 반전하여 '대상 표면 외향'으로 통일
            n = np.array(c[7]); n = n if c[2] == self.target.body else -n
            n = unit(n)
            gp_pos = np.array(c[6] if c[2] == self.target.body else c[5])
            tpos, _ = self.target.pose(); tv, tw = self._target_v_prev
            surf_v = tv + np.cross(tw, gp_pos - tpos)
            v_rel = self._hv_prev - surf_v
            v_n = float(-np.dot(v_rel, n))  # 표면 안쪽으로 파고드는 속도 (충돌 전 상대속도 기준)
            axis = quat_to_R(self._hq_prev) @ np.array([0, 0, 1.0])
            incidence = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(axis, -n))))))
            if v_n >= h["min_penetration_speed"] and incidence <= h["max_incidence_deg"]:
                self._anchor()
                self.notes.append(f"penetrated at t={self.world.t:.2f}s v_n={v_n:.2f}m/s incidence={incidence:.1f}deg")
            else:
                self.state = "bounced"; self.state_time = self.world.t
                self.notes.append(f"bounced at t={self.world.t:.2f}s v_n={v_n:.2f}m/s incidence={incidence:.1f}deg")
        elif self.state == "anchored":
            st = p.getConstraintState(self.anchor_constraint)
            F = float(np.linalg.norm(st[:3]))
            self.max_interaction_force = max(self.max_interaction_force, F)
            if F > h["anchor_break_force"]:
                p.removeConstraint(self.anchor_constraint)
                self.anchor_constraint = None
                self.state = "torn"; self.state_time = self.world.t
                self.bond_lost_time = self.world.t
                self.notes.append(f"anchor torn out at t={self.world.t:.2f}s F={F:.0f}N")

    def _anchor(self):
        hpos, hq = p.getBasePositionAndOrientation(self.harpoon)
        tpos, tq = self.target.pose()
        inv_p, inv_q = p.invertTransform(tpos.tolist(), tq)
        child_pos, child_q = p.multiplyTransforms(inv_p, inv_q, hpos, hq)
        self.anchor_constraint = p.createConstraint(self.harpoon, -1, self.target.body, -1, p.JOINT_FIXED,
                                                    [0, 0, 0], [0, 0, 0], child_pos, [0, 0, 0, 1], child_q)
        p.changeConstraint(self.anchor_constraint, maxForce=1e6)
        p.setCollisionFilterPair(self.harpoon, self.target.body, -1, -1, 0)  # 박힌 뒤 접촉력 제거
        self.state = "anchored"; self.state_time = self.world.t
        self.bond_established_time = self.world.t

    def bond_intact(self) -> bool:
        return self.state == "anchored"

    def should_abort(self) -> bool:
        if self.state in ("bounced", "torn") and self.world.t > self.state_time + self.hcfg["bounce_fail_delay"]:
            return True
        if self.state == "flying":
            hpos, _ = p.getBasePositionAndOrientation(self.harpoon)
            tpos, _ = self.target.pose()
            if hpos[0] > tpos[0] + self.target.bounding_radius + 1.0:
                self.notes.append("harpoon missed the target")
                return True
        return False

    def status_text(self):
        return f"harpoon={self.state}"

    def trajectory_static(self):
        h = self.hcfg
        return {"harpoon": {"radius": h["radius"], "length": h["length"], "tether_n": self.tether.n,
                            "tether_node_radius": h["tether_node_radius"],
                            "anchors": [{"node": a.node_idx, "body": ("servicer" if a.body == self.servicer.body else "harpoon"),
                                         "local": a.local_point.tolist()} for a in self.tether.anchors]}}

    def trajectory_frame(self):
        hp, hq = p.getBasePositionAndOrientation(self.harpoon)
        pos, _ = self.tether.gather()
        return {"harp": list(hp) + list(hq), "teth": pos.ravel().tolist()}

    def orbit_bodies(self):
        return [(self.harpoon, self.hcfg["mass"])]

    def extra_metrics(self):
        return {"harpoon_state": self.state}


METHODS: Dict[str, Callable[..., CaptureMethod]] = {
    "arm": RobotArmCapture,
    "net": NetCapture,
    "harpoon": HarpoonCapture,
}



# =============================================================================
# OrbitalDynamics : LVLH 회전좌표계 상대 궤도 역학 (Hill / Clohessy-Wiltshire) + 중력구배 토크
# =============================================================================
class OrbitalDynamics:
    """
    시뮬레이션 좌표계를 기준 원궤도의 LVLH 프레임으로 해석한다.
        radial(천정) = +z, along-track = +x, cross-track = +y
    Hill 방정식 (r: radial, t: along-track, c: cross-track, n: 궤도 평균운동):
        r'' = 3 n^2 r + 2 n t'
        t'' = -2 n r'
        c'' = -n^2 c
    각 강체에 m * a 를 외력으로, 대상·서비서에는 중력구배 토크 3 n^2 (r_hat x I r_hat) 를 적용한다.
    참고: 회전좌표계이므로 관성 각속도 = 프레임 각속도(n, ~0.06 deg/s) + 상대 각속도. 판정 임계값 1 deg/s 대비 무시 가능.
    """

    def __init__(self, cfg: Dict):
        o = cfg["orbit"]
        a = (o["earth_radius_km"] + o["altitude_km"]) * 1e3
        self.n = math.sqrt(o["mu_earth_m3s2"] / a ** 3)
        self.gg = o["gravity_gradient_torque"]
        self.bodies: List[Tuple[int, float, Optional[np.ndarray]]] = []  # (body, mass, inertia_diag or None)

    def register(self, body: int, mass: float, inertia_diag: Optional[np.ndarray] = None):
        self.bodies.append((body, mass, inertia_diag))

    def apply(self):
        n = self.n
        for body, m, I in self.bodies:
            pos, q = p.getBasePositionAndOrientation(body)
            v, _ = p.getBaseVelocity(body)
            x, y, z = pos            # t = x, c = y, r = z
            vx, vy, vz = v
            a_t = -2 * n * vz
            a_c = -n * n * y
            a_r = 3 * n * n * z + 2 * n * vx
            p.applyExternalForce(body, -1, [m * a_t, m * a_c, m * a_r], pos, p.WORLD_FRAME)
            if self.gg and I is not None:
                R = quat_to_R(q)
                r_b = R.T @ np.array([0.0, 0.0, 1.0])
                tau_b = 3 * n * n * np.cross(r_b, I * r_b)
                p.applyExternalTorque(body, -1, (R @ tau_b).tolist(), p.WORLD_FRAME)


# =============================================================================
# SpacecraftVisuals : 위성·파편 세부 외형 (시각 전용) — 충돌 형상·질량·관성은 변경하지 않음
# =============================================================================
VISUAL_DETAIL = {"enabled": False}   # CaptureSimulation 이 GUI/녹화 시에만 True 로 설정 (스윕 실행 비용 절감)
MAX_VISUAL_ARRAY = 8                 # PyBullet createVisualShapeArray 는 도형이 많으면(≈16+) 내부 메모리가 깨져 이후 호출이 크래시 → 배열당 8개로 제한
VISUAL_OVERLAYS: List[Tuple[int, int]] = []   # (부모 바디, 시각 전용 오버레이 바디). 매 스텝 부모 자세로 동기화


class SpacecraftVisuals:
    """
    createVisualShapeArray 로 여러 기본 도형을 조합해 위성/파편 외형을 만든다.
    절차적 텍스처(태양전지 격자, MLI 금박, 로켓 단 도장)는 ASCII 임시 폴더에 PNG 로 생성해 loadTexture 로 올린다.
    시각 부품이 충돌 형상 밖으로 튀어나올 수 있으나(안테나, 노즐 등) 접촉 물리에는 관여하지 않는다.
    """
    _tex_ids: Dict[str, int] = {}

    GOLD = [0.82, 0.66, 0.22, 1]
    SILVER = [0.78, 0.78, 0.8, 1]
    DARK = [0.18, 0.18, 0.2, 1]
    WHITE = [0.92, 0.92, 0.9, 1]
    PANEL = [0.1, 0.15, 0.4, 1]

    @classmethod
    def reset(cls):
        cls._tex_ids = {}
        VISUAL_OVERLAYS.clear()

    @staticmethod
    def sync_overlays():
        """시각 전용 오버레이 바디를 부모 강체 자세에 맞춤 (GUI/녹화 시 매 스텝 호출)"""
        for parent, overlay in VISUAL_OVERLAYS:
            pos, orn = p.getBasePositionAndOrientation(parent)
            p.resetBasePositionAndOrientation(overlay, pos, orn)

    @classmethod
    def attach_overlays(cls, parent: int, chunks: List[Tuple[int, Optional[str]]]):
        """첫 청크 이후의 시각 배열들을 질량 0·충돌 없음 오버레이 바디로 생성해 부모에 붙임"""
        pos, orn = p.getBasePositionAndOrientation(parent)
        for vid, tex in chunks[1:]:
            ob = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vid,
                                   basePosition=pos, baseOrientation=orn)
            cls.apply_textures(ob, -1, tex)
            VISUAL_OVERLAYS.append((parent, ob))

    # ---- 절차적 텍스처 --------------------------------------------------
    @classmethod
    def texture(cls, name: str) -> int:
        if name in cls._tex_ids:
            return cls._tex_ids[name]
        try:
            import tempfile
            from PIL import Image
            rng = np.random.default_rng(7)
            if name == "solar":
                img = np.full((256, 256, 3), (16, 26, 80), np.uint8)
                img += (rng.normal(0, 4, (256, 256, 1))).astype(np.int8).clip(-8, 8).astype(np.uint8)
                img[::32, :] = (140, 160, 210); img[:, ::32] = (140, 160, 210)
                img[16::32, :] = (60, 80, 140); img[:, 16::32] = (60, 80, 140)
            elif name == "mli":
                base = np.array((215, 170, 60), np.float32)
                noise = rng.uniform(0.7, 1.15, (256, 256, 1))
                crease = (np.sin(np.linspace(0, 40, 256))[:, None, None] * 0.08 + 1.0)
                img = (base * noise * crease).clip(0, 255).astype(np.uint8)
            elif name == "stage":
                img = np.full((256, 256, 3), (225, 225, 220), np.uint8)
                img[:, 200:] = (150, 150, 155)               # 상단 어두운 밴드
                img[:, 40:52] = (30, 30, 35)                 # 검정 스트라이프
                img[100:130, 90:170] = (180, 40, 40)         # 붉은 마킹
                img += rng.normal(0, 3, (256, 256, 3)).astype(np.int8).astype(np.uint8)
            else:
                raise ValueError(name)
            path = os.path.join(tempfile.gettempdir(), f"debris_sim_tex_{name}.png")
            Image.fromarray(img).save(path)
            tid = p.loadTexture(path) if path.isascii() else -1
        except Exception as e:
            print(f"[warn] texture '{name}' unavailable: {e}", file=sys.stderr)
            tid = -1
        cls._tex_ids[name] = tid
        return tid

    # ---- 부품 조립 ------------------------------------------------------
    @classmethod
    def _assemble(cls, parts: List[Dict]) -> List[Tuple[int, Optional[str]]]:
        """
        parts: dict(type, size, pos, orn, color, tex) → [(visual id, 텍스처 이름 또는 None)]
        PyBullet 텍스처는 링크 전체에 적용되므로 텍스처 종류별로 배열을 나누고, 배열당 MAX_VISUAL_ARRAY 개로 제한한다.
        """
        groups: Dict[Optional[str], List[Dict]] = {}
        for q in parts:
            groups.setdefault(q.get("tex"), []).append(q)
        keys = ([None] if None in groups else []) + [k for k in groups if k is not None]
        chunks = []
        for k in keys:
            g = groups[k]
            for i in range(0, len(g), MAX_VISUAL_ARRAY):
                chunks.append((cls._assemble_one(g[i:i + MAX_VISUAL_ARRAY]), k))
        return chunks

    @staticmethod
    def _assemble_one(parts: List[Dict]) -> int:
        types, radii, hes, lens, poss, orns, cols = [], [], [], [], [], [], []
        for i, q in enumerate(parts):
            t = q["type"]
            types.append(t)
            if t == p.GEOM_BOX:
                hes.append(list(q["size"])); radii.append(0.1); lens.append(0.1)
            elif t == p.GEOM_CYLINDER:
                hes.append([0.1] * 3); radii.append(q["size"][0]); lens.append(q["size"][1])
            elif t == p.GEOM_SPHERE:
                hes.append([0.1] * 3); radii.append(q["size"][0]); lens.append(0.1)
            poss.append(list(q.get("pos", (0, 0, 0)))); orns.append(list(q.get("orn", (0, 0, 0, 1))))
            cols.append(list(q.get("color", SpacecraftVisuals.WHITE)))
        return p.createVisualShapeArray(shapeTypes=types, radii=radii, halfExtents=hes, lengths=lens,
                                        visualFramePositions=poss, visualFrameOrientations=orns, rgbaColors=cols)

    @classmethod
    def apply_textures(cls, body: int, link: int, tex: Optional[str]):
        if not tex:
            return
        tid = cls.texture(tex)
        if tid >= 0:
            try:
                p.changeVisualShape(body, link, textureUniqueId=tid)
            except Exception:
                pass

    @staticmethod
    def _qx():
        return p.getQuaternionFromEuler([0, math.pi / 2, 0])   # 원통 축 z → x

    @staticmethod
    def _qy():
        return p.getQuaternionFromEuler([math.pi / 2, 0, 0])   # 원통 축 z → y

    # ---- 서비스 위성 ----------------------------------------------------
    @classmethod
    def servicer(cls, he: Sequence[float]) -> List[Tuple[int, Optional[str]]]:
        return cls._assemble(cls.servicer_parts(he))

    @classmethod
    def target(cls, kind: str, tcfg: Dict) -> List[Tuple[int, Optional[str]]]:
        return cls._assemble(cls.target_parts(kind, tcfg))

    @classmethod
    def gripper(cls, radius: float) -> List[Tuple[int, Optional[str]]]:
        return cls._assemble(cls.gripper_parts(radius))

    @staticmethod
    def parts_to_json(parts: List[Dict]) -> List[Dict]:
        names = {p.GEOM_BOX: "box", p.GEOM_CYLINDER: "cylinder", p.GEOM_SPHERE: "sphere"}
        out = []
        for q in parts:
            out.append({"type": names[q["type"]], "size": [float(v) for v in q["size"]],
                        "pos": [float(v) for v in q.get("pos", (0, 0, 0))],
                        "orn": [float(v) for v in q.get("orn", (0, 0, 0, 1))],
                        "color": [float(v) for v in q.get("color", SpacecraftVisuals.WHITE)],
                        "tex": q.get("tex")})
        return out

    @classmethod
    def servicer_parts(cls, he: Sequence[float]) -> List[Dict]:
        hx, hy, hz = he
        B, C, S = p.GEOM_BOX, p.GEOM_CYLINDER, p.GEOM_SPHERE
        parts = [dict(type=B, size=(hx, hy, hz), color=cls.GOLD, tex="mli")]
        for sgn in (1, -1):
            parts.append(dict(type=C, size=(0.03, 0.45), pos=(0, sgn * (hy + 0.22), 0), orn=cls._qy(), color=cls.SILVER))
            parts.append(dict(type=B, size=(0.6, 1.5, 0.012), pos=(0, sgn * (hy + 0.45 + 1.5), 0), color=cls.PANEL, tex="solar"))
            parts.append(dict(type=B, size=(0.62, 0.02, 0.02), pos=(0, sgn * (hy + 0.45 + 3.0), 0), color=cls.SILVER))
        # 고이득 안테나 (마스트 + 접시, +x 방향으로 기울임)
        parts.append(dict(type=C, size=(0.025, 0.36), pos=(-0.2, 0, hz + 0.18), color=cls.SILVER))
        parts.append(dict(type=C, size=(0.5, 0.03), pos=(-0.05, 0, hz + 0.42),
                          orn=p.getQuaternionFromEuler([0, -0.6, 0]), color=cls.WHITE))
        parts.append(dict(type=C, size=(0.012, 0.35), pos=(0.12, 0, hz + 0.55),
                          orn=p.getQuaternionFromEuler([0, -0.6, 0]), color=cls.DARK))
        # 추력기 노즐 4개 (-x 후면)
        for y in (-0.45, 0.45):
            for z in (-0.45, 0.45):
                parts.append(dict(type=C, size=(0.07, 0.2), pos=(-hx - 0.1, y, z), orn=cls._qx(), color=cls.DARK))
        # 주 추력기 (후면 중앙)
        parts.append(dict(type=C, size=(0.18, 0.3), pos=(-hx - 0.15, 0, 0), orn=cls._qx(), color=cls.DARK))
        # 근접 센서/카메라 (전면 +x, 팔 장착부 옆)
        parts.append(dict(type=B, size=(0.05, 0.08, 0.08), pos=(hx + 0.05, 0.45, 0.45), color=cls.DARK))
        parts.append(dict(type=C, size=(0.04, 0.04), pos=(hx + 0.12, 0.45, 0.45), orn=cls._qx(), color=[0.3, 0.5, 0.9, 1]))
        parts.append(dict(type=B, size=(0.05, 0.08, 0.08), pos=(hx + 0.05, -0.45, 0.45), color=cls.DARK))
        # 별 추적기, 라디에이터
        parts.append(dict(type=C, size=(0.06, 0.16), pos=(0.3, -hy - 0.08, hz - 0.2),
                          orn=p.getQuaternionFromEuler([math.pi / 2, 0, 0]), color=cls.DARK))
        parts.append(dict(type=B, size=(0.5, 0.005, 0.4), pos=(0, 0, -hz - 0.006), orn=p.getQuaternionFromEuler([math.pi / 2, 0, 0]), color=cls.WHITE))
        return parts

    # ---- 파편 -----------------------------------------------------------
    @classmethod
    def target_parts(cls, kind: str, tcfg: Dict) -> List[Dict]:
        B, C, S = p.GEOM_BOX, p.GEOM_CYLINDER, p.GEOM_SPHERE
        parts: List[Dict] = []
        if tcfg["shape"] == "box":                      # 큐브샛형 소형 파편
            h = tcfg["half_extents"][0]
            parts.append(dict(type=B, size=(h, h, h), color=[0.35, 0.36, 0.38, 1]))
            for sx in (-1, 1):
                for sy in (-1, 1):
                    parts.append(dict(type=B, size=(0.012, 0.012, h + 0.005), pos=(sx * (h - 0.01), sy * (h - 0.01), 0), color=cls.SILVER))
            f = h - 0.03
            for sgn in (-1, 1):
                parts.append(dict(type=B, size=(0.003, f, f), pos=(sgn * (h + 0.003), 0, 0), color=cls.PANEL, tex="solar"))
                parts.append(dict(type=B, size=(f, 0.003, f), pos=(0, sgn * (h + 0.003), 0), color=cls.PANEL, tex="solar"))
            parts.append(dict(type=C, size=(0.004, 0.4), pos=(0.06, 0.06, h + 0.2), color=cls.SILVER))
            parts.append(dict(type=B, size=(0.05, 0.05, 0.006), pos=(-0.05, -0.05, h + 0.006), color=cls.GOLD))
        elif tcfg["shape"] == "bus_with_panels":        # 태양전지판 위성 (중형)
            bhe, phe, dy = tcfg["bus_half_extents"], tcfg["panel_half_extents"], tcfg["panel_offset_y"]
            parts.append(dict(type=B, size=bhe, color=cls.GOLD, tex="mli"))
            for sgn in (-1, 1):
                parts.append(dict(type=B, size=phe, pos=(0, sgn * dy, 0), color=cls.PANEL, tex="solar"))
                parts.append(dict(type=C, size=(0.03, dy - phe[1] - bhe[1] + 0.02), pos=(0, sgn * (bhe[1] + (dy - phe[1] - bhe[1]) / 2), 0),
                                  orn=cls._qy(), color=cls.SILVER))
                for ex in (-1, 1):   # 패널 프레임
                    parts.append(dict(type=B, size=(0.015, phe[1], 0.02), pos=(ex * phe[0], sgn * dy, 0), color=cls.SILVER))
                    parts.append(dict(type=B, size=(phe[0], 0.015, 0.02), pos=(0, sgn * (dy + ex * phe[1]), 0), color=cls.SILVER))
            # 안테나 접시(+z), 노즐(-z), 그래플 픽스처(-x 파지면), 장비 박스(+x)
            parts.append(dict(type=C, size=(0.02, 0.3), pos=(0, 0, bhe[2] + 0.15), color=cls.SILVER))
            parts.append(dict(type=C, size=(0.35, 0.03), pos=(0, 0, bhe[2] + 0.32), color=cls.WHITE))
            parts.append(dict(type=C, size=(0.16, 0.3), pos=(0, 0, -bhe[2] - 0.15), color=cls.DARK))
            parts.append(dict(type=C, size=(0.12, 0.02), pos=(-bhe[0] - 0.01, 0, 0), orn=cls._qx(), color=cls.SILVER))
            parts.append(dict(type=C, size=(0.03, 0.05), pos=(-bhe[0] - 0.035, 0, 0), orn=cls._qx(), color=cls.DARK))
            parts.append(dict(type=B, size=(0.05, 0.15, 0.1), pos=(bhe[0] + 0.05, 0.2, -0.2), color=cls.SILVER))
            parts.append(dict(type=B, size=(0.03, 0.1, 0.1), pos=(bhe[0] + 0.03, -0.25, 0.25), color=cls.DARK))
        elif tcfg["shape"] == "cylinder":               # 로켓 상단 (대형)
            r, L = tcfg["radius"], tcfg["length"]
            parts.append(dict(type=C, size=(r, L), color=cls.WHITE, tex="stage"))
            parts.append(dict(type=C, size=(r * 1.02, 0.2), pos=(0, 0, L / 2 - 0.12), color=cls.DARK))    # 인터스테이지 링
            parts.append(dict(type=S, size=(0.98 * r,), pos=(0, 0, L / 2 - 0.6 * r), color=cls.SILVER))  # 상단 탱크 돔: 원통 안 구의 캡(높이 0.4r)만 노출
            for k, (rr, zz) in enumerate(((0.3, 0.15), (0.45, 0.45), (0.6, 0.75))):                        # 노즐 벨
                parts.append(dict(type=C, size=(rr * r, 0.3), pos=(0, 0, -L / 2 - zz), color=[0.25, 0.22, 0.2, 1]))
            parts.append(dict(type=C, size=(0.2 * r, 0.2), pos=(0, 0, -L / 2 - 0.05), color=cls.DARK))
            for ang in (0.0, math.pi / 2):                                                                # 레이스웨이
                parts.append(dict(type=B, size=(0.04, 0.08, L / 2 - 0.15), pos=(r * math.cos(ang), r * math.sin(ang), 0),
                                  orn=p.getQuaternionFromEuler([0, 0, ang]), color=cls.DARK))
            parts.append(dict(type=B, size=(0.05, 0.12, 0.12), pos=(r * 0.7, -r * 0.7, 0.8),
                              orn=p.getQuaternionFromEuler([0, 0, -math.pi / 4]), color=cls.SILVER))     # 장비 박스
        else:
            raise ValueError(tcfg["shape"])
        return parts

    @classmethod
    def gripper_parts(cls, radius: float) -> List[Dict]:
        """그리퍼: 손바닥 구 + 3 손가락"""
        parts = [dict(type=p.GEOM_SPHERE, size=(radius,), color=[0.85, 0.2, 0.2, 1])]
        for k in range(3):
            ang = 2 * math.pi * k / 3
            parts.append(dict(type=p.GEOM_BOX, size=(radius * 0.9, 0.012, 0.012),
                              pos=(radius * 0.8, radius * 0.6 * math.cos(ang), radius * 0.6 * math.sin(ang)),
                              orn=p.getQuaternionFromEuler([ang, -0.35, 0]), color=cls.SILVER))
        return parts


# =============================================================================
# SpaceScenery : GUI 장면 연출 + 카메라 조작 (지구, 별, 태양광, HUD, 키보드/마우스 카메라) — 물리에 영향 없음
# =============================================================================
class SpaceScenery:
    """
    조작키
        ← → ↑ ↓ : 카메라 회전(요/피치)      Z / X : 줌 인/아웃
        W A S D  : 앞뒤/좌우 이동           Q / E : 위/아래 이동
        F        : 추적 모드 순환 (mid → target → servicer → free)
        1 / 2 / 3: 프리셋 (전체 보기 / 대상 근접 / 서비서 시점)     R : 카메라 리셋
        Space    : 일시정지/재개            [ / ] : 슬로모션 / 정상 속도       H : HUD 켜기/끄기
        마우스   : Ctrl+왼쪽 드래그 회전, 휠 줌 (PyBullet 기본)
    """
    MODES = ("mid", "target", "servicer", "free")

    def __init__(self, world: World, target: Target, servicer: Servicer):
        self.world, self.target, self.servicer = world, target, servicer
        self.cfg = world.cfg["scene"]
        self.hud_ids: List[int] = []
        self.axis_line = None
        self.grasp_marker = None
        self.paused = False
        self.hud_on = self.cfg["hud"]
        self.mode = "mid" if self.cfg["camera_follow"] else "free"
        self.offset = np.zeros(3)
        self.yaw, self.pitch = 35.0, -18.0
        tpos, _ = target.pose(); spos, _ = servicer.pose()
        self.dist0 = max(5.0, 1.6 * float(np.linalg.norm(tpos - spos)))
        self.dist = self.dist0
        self.free_anchor = (tpos + spos) / 2
        self._build()
        self._apply_camera()

    # ---- 장면 구성 ---------------------------------------------------------
    def _build(self):
        c = self.cfg
        o = self.world.cfg["orbit"]
        alpha = math.asin(o["earth_radius_km"] / (o["earth_radius_km"] + o["altitude_km"]))
        R_vis = c["earth_visual_radius_m"]
        dist = R_vis / math.sin(alpha)
        import pybullet_data
        mesh = os.path.join(pybullet_data.getDataPath(), "textured_sphere_smooth.obj")
        vis = p.createVisualShape(p.GEOM_MESH, fileName=mesh, meshScale=[R_vis] * 3, rgbaColor=[1, 1, 1, 1])
        self.earth = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
                                       basePosition=[0, 0, -dist],
                                       baseOrientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0]))
        tex_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), c["earth_texture"])
        if os.path.exists(tex_path) and not tex_path.isascii():
            import shutil, tempfile
            ascii_path = os.path.join(tempfile.gettempdir(), "debris_sim_earth" + os.path.splitext(tex_path)[1])
            if ascii_path.isascii():
                shutil.copyfile(tex_path, ascii_path)
                tex_path = ascii_path
        if os.path.exists(tex_path):
            try:
                p.changeVisualShape(self.earth, -1, textureUniqueId=p.loadTexture(tex_path))
            except Exception as e:
                print(f"[warn] earth texture load failed: {e}", file=sys.stderr)
                p.changeVisualShape(self.earth, -1, rgbaColor=[0.2, 0.4, 0.9, 1])
        else:
            p.changeVisualShape(self.earth, -1, rgbaColor=[0.2, 0.4, 0.9, 1])
        rng = np.random.default_rng(42)
        v = rng.normal(size=(c["stars"], 3)); v /= np.linalg.norm(v, axis=1)[:, None]
        b = rng.uniform(0.4, 1.0, c["stars"])
        cols = np.stack([b, b, b * rng.uniform(0.9, 1.0, c["stars"])], axis=1).tolist()
        p.addUserDebugPoints((v * c["star_distance_m"]).tolist(), cols, pointSize=2, lifeTime=0)
        sun = np.array(c["sun_direction"], dtype=float); sun = sun / np.linalg.norm(sun) * 100.0
        p.configureDebugVisualizer(lightPosition=sun.tolist())
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)  # PyBullet 기본 단축키(w=와이어프레임 등)와 충돌 방지
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)      # 마우스로 물체를 끌어 물리를 교란하지 않도록

    # ---- 카메라 -------------------------------------------------------------
    def _anchor(self) -> np.ndarray:
        tpos, _ = self.target.pose(); spos, _ = self.servicer.pose()
        if self.mode == "mid":
            return (tpos + spos) / 2
        if self.mode == "target":
            return tpos
        if self.mode == "servicer":
            return spos
        return self.free_anchor

    def _apply_camera(self):
        p.resetDebugVisualizerCamera(cameraDistance=self.dist, cameraYaw=self.yaw, cameraPitch=self.pitch,
                                     cameraTargetPosition=(self._anchor() + self.offset).tolist())

    def _preset(self, k: int):
        self.offset[:] = 0
        if k == 1:
            self.mode, self.yaw, self.pitch, self.dist = "mid", 35.0, -18.0, self.dist0
        elif k == 2:
            self.mode, self.yaw, self.pitch, self.dist = "target", 60.0, -10.0, max(2.5, 3.0 * self.target.bounding_radius)
        elif k == 3:
            self.mode, self.yaw, self.pitch, self.dist = "servicer", -90.0, -5.0, 3.0 * self.servicer.he[0]

    def poll_input(self):
        """매 스텝 호출: 키 입력 처리 (마우스 조작으로 바뀐 요/피치/거리는 창에서 읽어 동기화)"""
        cam = p.getDebugVisualizerCamera()
        self.yaw, self.pitch, self.dist = cam[8], cam[9], cam[10]
        ev = p.getKeyboardEvents()
        if not ev:
            if self.mode != "free" and self.world.step_count % 4 == 0:
                self._apply_camera()
            return
        down = lambda k: k in ev and (ev[k] & p.KEY_IS_DOWN)
        trig = lambda k: k in ev and (ev[k] & p.KEY_WAS_TRIGGERED)
        changed = False
        if down(p.B3G_LEFT_ARROW): self.yaw -= 1.5; changed = True
        if down(p.B3G_RIGHT_ARROW): self.yaw += 1.5; changed = True
        if down(p.B3G_UP_ARROW): self.pitch = min(89.0, self.pitch + 1.0); changed = True
        if down(p.B3G_DOWN_ARROW): self.pitch = max(-89.0, self.pitch - 1.0); changed = True
        if down(ord('z')): self.dist = max(0.5, self.dist * 0.97); changed = True
        if down(ord('x')): self.dist = min(300.0, self.dist * 1.03); changed = True
        # 화면 기준 이동: forward = 카메라 전방(수평), right = 전방 x up
        fwd = np.array(cam[5], dtype=float); fwd[2] = 0
        fwd = unit(fwd) if np.linalg.norm(fwd) > 1e-6 else np.array([1.0, 0, 0])
        right = np.cross(fwd, [0, 0, 1.0])
        step = 0.03 * self.dist
        if down(ord('w')): self.offset += fwd * step; changed = True
        if down(ord('s')): self.offset -= fwd * step; changed = True
        if down(ord('d')): self.offset += right * step; changed = True
        if down(ord('a')): self.offset -= right * step; changed = True
        if down(ord('q')): self.offset[2] += step; changed = True
        if down(ord('e')): self.offset[2] -= step; changed = True
        if trig(ord('f')):
            if self.mode == "servicer":
                self.free_anchor = self._anchor() + self.offset; self.offset[:] = 0
            self.mode = self.MODES[(self.MODES.index(self.mode) + 1) % len(self.MODES)]
            changed = True
        for k in (1, 2, 3):
            if trig(ord(str(k))):
                self._preset(k); changed = True
        if trig(ord('r')):
            self._preset(1); changed = True
        if trig(ord('h')):
            self.hud_on = not self.hud_on
            if not self.hud_on:
                for i in self.hud_ids:
                    p.removeUserDebugItem(i)
                self.hud_ids = []
        if trig(p.B3G_SPACE):
            self.paused = not self.paused
        if trig(ord('[')):
            self.world.time_scale = 0.25
        if trig(ord(']')):
            self.world.time_scale = 1.0
        if changed or self.mode != "free":
            self._apply_camera()

    # ---- HUD / 마커 ---------------------------------------------------------
    def update(self, method: "CaptureMethod", rel_omega_deg_s: float, bond: bool, force: bool = False):
        c = self.cfg
        if not force and self.world.step_count % c["hud_interval"]:
            return
        tpos, _ = self.target.pose(); spos, _ = self.servicer.pose()
        _, w = self.target.velocity()
        L = self.target.bounding_radius * 1.5
        wn = unit(w) * L
        a0, a1 = (tpos - wn).tolist(), (tpos + wn).tolist()
        if self.axis_line is None:
            self.axis_line = p.addUserDebugLine(a0, a1, [1, 0.3, 0.3], 2)
        else:
            p.addUserDebugLine(a0, a1, [1, 0.3, 0.3], 2, replaceItemUniqueId=self.axis_line)
        gp, _, gn = self.target.grasp_point_world()
        g0, g1 = gp.tolist(), (gp + gn * 0.3).tolist()
        if self.grasp_marker is None:
            self.grasp_marker = p.addUserDebugLine(g0, g1, [1, 1, 0.2], 3)
        else:
            p.addUserDebugLine(g0, g1, [1, 1, 0.2], 3, replaceItemUniqueId=self.grasp_marker)
        if not self.hud_on:
            return
        lines = [
            f"t={self.world.t:6.2f}s  {method.name}  {method.status_text()}   "
            f"|w|={math.degrees(np.linalg.norm(w)):5.2f} deg/s  rel w={rel_omega_deg_s:6.2f} deg/s  "
            f"bond={'YES' if bond else 'no'}  d={np.linalg.norm(tpos - spos):5.2f} m"
            + ("   [PAUSED]" if self.paused else "") + (f"   x{self.world.time_scale:g}" if self.world.time_scale != 1 else ""),
            f"cam: {self.mode}   Arrows rotate  Z/X zoom  WASD/QE move  F follow  1/2/3 views  R reset  Space pause  [ ] slow/normal  H hud",
        ]
        # 카메라 좌표계에 붙여 항상 화면 좌상단 부근에 표시
        cam = p.getDebugVisualizerCamera()
        fwd = np.array(cam[5], dtype=float); up = np.array(cam[4], dtype=float)
        right = unit(np.cross(fwd, up))
        anchor = self._anchor() + self.offset
        base = anchor - fwd * 0 + up * (0.42 * self.dist) - right * (0.75 * self.dist)
        size = max(0.8, 0.07 * self.dist)
        for i, text in enumerate(lines):
            pos = (base - up * (i * 0.06 * self.dist)).tolist()
            if i < len(self.hud_ids):
                p.addUserDebugText(text, pos, [1, 1, 1], textSize=size, replaceItemUniqueId=self.hud_ids[i])
            else:
                self.hud_ids.append(p.addUserDebugText(text, pos, [1, 1, 1], textSize=size))



# =============================================================================
# TrajectoryRecorder : 웹 3D 뷰어(viewer_template.html)용 궤적 기록 → 독립 실행 HTML / JSON
# =============================================================================
class TrajectoryRecorder:
    """
    매 1/viewer_fps 초마다 대상·서비서·팔 링크·그물/테더 질점·작살의 자세와 지표를 기록한다.
    finish() 후 write_html() 로 three.js 뷰어가 내장된 단일 HTML 파일을 만든다 (브라우저에서 바로 열림, 인터넷 필요: three.js CDN).
    """

    def __init__(self, world: World, target: Target, servicer: Servicer, method: "CaptureMethod", fps: float):
        self.world, self.target, self.servicer, self.method = world, target, servicer, method
        self.every = max(1, int(round(1.0 / (world.dt * fps))))
        self.frames: List[Dict] = []
        self.status_table: List[str] = []
        cfg = world.cfg
        tcfg = cfg["targets"][target.kind]
        self.static = {
            "target": {"kind": target.kind, "mass": target.mass, "shape": tcfg["shape"],
                       "bounding_radius": target.bounding_radius,
                       "grasp_point": target.grasp_point_body.tolist(), "grasp_normal": target.grasp_normal_body.tolist(),
                       "inertia": target.inertia_diag.tolist(),
                       "parts": SpacecraftVisuals.parts_to_json(SpacecraftVisuals.target_parts(target.kind, tcfg))},
            "servicer": {"half_extents": servicer.he.tolist(), "mass": servicer.mass,
                         "attach": servicer.attach_local.tolist(),
                         "parts": SpacecraftVisuals.parts_to_json(SpacecraftVisuals.servicer_parts(servicer.he))},
            "orbit": dict(cfg["orbit"]),
            "scene": {"sun_direction": list(cfg["scene"]["sun_direction"])},
            "success": dict(cfg["success"]),
            "method": method.name,
        }
        self.static.update(method.trajectory_static())

    @staticmethod
    def _r(vals, nd=4):
        return [round(float(v), nd) for v in vals]

    def capture(self, rel_omega: float, bond: bool):
        if self.world.step_count % self.every:
            return
        tpos, tq = self.target.pose(); tv, tw = self.target.velocity()
        spos, sq = self.servicer.pose()
        st = self.method.status_text()
        if st not in self.status_table:
            self.status_table.append(st)
        fr = {"t": round(self.world.t, 4),
              "tgt": self._r(list(tpos) + list(tq)), "svc": self._r(list(spos) + list(sq)),
              "w": self._r(tw, 5), "rel": round(float(rel_omega), 4), "bond": int(bond),
              "st": self.status_table.index(st)}
        for k, v in self.method.trajectory_frame().items():
            fr[k] = self._r(v, 3)
        self.frames.append(fr)

    def finish(self, result: Dict) -> Dict:
        res = {k: v for k, v in result.items() if k != "history"}
        return {"static": self.static, "status_table": self.status_table, "frames": self.frames,
                "result": res, "fps": 1.0 / (self.world.dt * self.every)}

    @staticmethod
    def _json_default(o):
        """numpy 스칼라·배열을 표준 파이썬 타입으로 (np.bool_ 등은 기본 인코더가 처리하지 못함)"""
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(f"not JSON serializable: {type(o).__name__}")

    @classmethod
    def write_json(cls, data: Dict, path: str):
        import json
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, separators=(",", ":"), default=cls._json_default)

    @staticmethod
    def build_html(data: Dict, standalone: bool = True) -> str:
        """viewer_template.html 에 데이터·지구 텍스처·three.js 를 심어 HTML 문자열 생성"""
        import json, base64
        here = os.path.dirname(os.path.abspath(__file__))
        tpl = open(os.path.join(here, "viewer_template.html"), encoding="utf-8").read()
        earth = ""
        tex = os.path.join(here, CONFIG["scene"]["earth_texture"])
        if os.path.exists(tex):
            earth = "data:image/jpeg;base64," + base64.b64encode(open(tex, "rb").read()).decode("ascii")
        payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"),
                             default=TrajectoryRecorder._json_default).replace("</", "<\\/")
        html = tpl.replace("/*__DATA_JSON__*/null", payload).replace("__EARTH_DATA_URI__", earth)
        # three.js 를 파일에 내장해 오프라인에서도 열리게 한다 (assets/three.min.js 가 있을 때).
        # 없으면 템플릿의 CDN <script> 가 그대로 남아 인터넷 연결이 필요하다.
        three = os.path.join(here, CONFIG["scene"]["three_js"])
        if os.path.exists(three):
            src = open(three, encoding="utf-8").read().replace("</script", "<\\/script")
            html = html.replace('<!--__THREE_INLINE__--><script src="https://cdnjs.cloudflare.com/ajax/libs/'
                                'three.js/r128/three.min.js"></script>',
                                "<script>" + src + "</script>")
        if standalone:
            html = ('<!DOCTYPE html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
                    '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                    '</head>\n<body>\n' + html + '\n</body>\n</html>\n')
        return html

    @classmethod
    def write_html(cls, data: Dict, path: str):
        with open(path, "w", encoding="utf-8") as f:
            f.write(cls.build_html(data, standalone=True))

# =============================================================================
# FrameRecorder : PNG 프레임 녹화 (GUI/헤드리스 모두 가능, 논문 그림·영상 제작용)
# =============================================================================
class FrameRecorder:
    """
    GUI 모드: OpenGL 렌더러로 지구·조명 포함 장면을 캡처 (별·HUD 등 디버그 항목은 캡처에 미포함).
    헤드리스: PyBullet 내장 소프트웨어 렌더러(TINY)는 약 20 m 이상 떨어진 물체를 그리지 못해 지구 배경이 빠지고
    포획 장면만 검은 배경 위에 기록된다. 논문용 장면 그림은 --gui --record-frames 조합을 권장.
    """
    def __init__(self, world: World, out_dir: str, target: Target, servicer: Servicer,
                 yaw: float = 35.0, pitch: float = -18.0, distance: Optional[float] = None):
        self.world, self.target, self.servicer = world, target, servicer
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.w, self.h = world.cfg["scene"]["frame_size"]
        self.interval = world.cfg["scene"]["record_interval"]
        self.yaw, self.pitch = yaw, pitch
        tpos, _ = target.pose(); spos, _ = servicer.pose()
        self.distance = distance or max(5.0, 1.6 * float(np.linalg.norm(tpos - spos)))
        self.proj = p.computeProjectionMatrixFOV(fov=55, aspect=self.w / self.h, nearVal=0.1, farVal=2000.0)
        self.count = 0
        try:
            from PIL import Image  # noqa: F401
            self._pil = True
        except ImportError:
            self._pil = False
            print("[warn] Pillow not installed: frames saved as .npy", file=sys.stderr)

    def capture(self):
        if self.world.step_count % self.interval:
            return
        tpos, _ = self.target.pose(); spos, _ = self.servicer.pose()
        mid = ((tpos + spos) / 2).tolist()
        view = p.computeViewMatrixFromYawPitchRoll(mid, self.distance, self.yaw, self.pitch, 0, upAxisIndex=2)
        renderer = p.ER_BULLET_HARDWARE_OPENGL if self.world.gui else p.ER_TINY_RENDERER
        img = p.getCameraImage(self.w, self.h, view, self.proj, renderer=renderer,
                               lightDirection=list(self.world.cfg["scene"]["sun_direction"]), shadow=0)
        rgba = np.reshape(np.array(img[2], dtype=np.uint8), (self.h, self.w, 4))
        depth = np.reshape(np.array(img[3], dtype=np.float32), (self.h, self.w))
        rgba[depth >= 0.9999, :3] = 0  # 아무것도 없는 배경은 우주(검정)
        path = os.path.join(self.out_dir, f"frame_{self.count:05d}")
        if self._pil:
            from PIL import Image
            Image.fromarray(rgba[:, :, :3]).save(path + ".png")
        else:
            np.save(path + ".npy", rgba)
        self.count += 1


# =============================================================================
# CaptureSimulation : 단일 시행
# =============================================================================
class CaptureSimulation:
    def __init__(self, method: str, target_kind: str, spin_deg_s: float, spin_mode: str = "principal",
                 nutation_deg: Optional[float] = None, seed: int = 0, gui: bool = False, cfg: Dict = CONFIG,
                 record_frames: Optional[str] = None, record_video: Optional[str] = None,
                 viewer_html: Optional[str] = None, trajectory_json: Optional[str] = None):
        self.cfg = cfg
        self.record_frames = record_frames
        self.record_video = record_video
        self.viewer_html = viewer_html
        self.trajectory_json = trajectory_json
        self.method_name = method
        self.target_kind = target_kind
        self.spin_deg_s = spin_deg_s
        self.spin_mode = spin_mode
        self.nutation_deg = nutation_deg
        self.seed = seed
        self.gui = gui

    @staticmethod
    def rng_normal(rng: np.random.Generator, sigma: float, n: int) -> np.ndarray:
        return rng.normal(0.0, sigma, n) if sigma > 0 else np.zeros(n)

    def run(self) -> Dict:
        cfg = self.cfg
        rng = np.random.default_rng(self.seed)
        VISUAL_DETAIL["enabled"] = bool(self.gui or self.record_frames or self.record_video)
        world = World(cfg, self.gui)
        try:
            method = METHODS[self.method_name](cfg, rng)
            target = Target(cfg, self.target_kind, self.spin_deg_s, self.spin_mode, rng, self.nutation_deg)
            # 몬테카를로 섭동: 랑데부 종료 시점의 상대 위치·지향·잔류 속도 오차
            mc = cfg["monte_carlo"]
            s_pos = method.servicer_position(target)
            s_pos = s_pos + np.array([0.0, *self.rng_normal(rng, mc["relative_position_error_m"], 2)])
            s_orn = random_small_rotation(rng, mc["servicer_pointing_error_deg"])
            servicer = Servicer(cfg, s_pos, method.arm_spec(), orientation=s_orn)
            v_err = self.rng_normal(rng, mc["relative_velocity_error_mps"], 3)
            _, w_t = target.velocity()
            p.resetBaseVelocity(target.body, linearVelocity=v_err.tolist(), angularVelocity=w_t.tolist())
            method.setup(world, servicer, target)
            p.performCollisionDetection()
            if p.getContactPoints(bodyA=target.body):
                method.notes.append("WARNING: initial overlap between target and other bodies")
                print("[warn] initial overlap between target and other bodies", file=sys.stderr)
            # 궤도 환경 (LVLH 상대 역학)
            orbit = None
            if cfg["orbit"]["enabled"]:
                orbit = OrbitalDynamics(cfg)
                orbit.register(target.body, target.mass, target.inertia_diag)
                orbit.register(servicer.body, servicer.mass, Target._box_inertia(servicer.mass, servicer.he))
                for b, m in method.orbit_bodies():
                    orbit.register(b, m, None)
                if cfg["orbit"]["apply_to_nodes"]:
                    for b in method.device_bodies():
                        if b not in (target.body, servicer.body) and b not in [ob for ob, _ in method.orbit_bodies()]:
                            orbit.register(b, p.getDynamicsInfo(b, -1)[0], None)
            scenery = SpaceScenery(world, target, servicer) if self.gui else None
            recorder = FrameRecorder(world, self.record_frames, target, servicer) if self.record_frames else None
            tj = None
            if self.viewer_html or self.trajectory_json:
                tj = TrajectoryRecorder(world, target, servicer, method, cfg["scene"]["viewer_fps"])
            log_id = None
            if self.record_video:
                # PyBullet 의 mp4 녹화는 GUI 연결과 PATH 의 ffmpeg 가 모두 필요하며,
                # 둘 중 하나라도 없으면 아무 말 없이 빈 파일만 남는다. 미리 확인해 알려준다.
                import shutil
                if not self.gui:
                    print("[warn] --record-video 는 --gui 와 함께 써야 한다. 헤드리스라면 --record-frames 를 사용할 것.",
                          file=sys.stderr)
                elif shutil.which("ffmpeg") is None:
                    print("[warn] PATH 에서 ffmpeg 를 찾지 못해 mp4 녹화를 건너뛴다. "
                          "PNG 프레임은 --record-frames, 대화형 재생은 --viewer 를 사용할 것.", file=sys.stderr)
                else:
                    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, self.record_video)
            try:
                result = self._loop(world, method, target, servicer, orbit, scenery, recorder, tj)
            finally:
                if log_id is not None:
                    p.stopStateLogging(log_id)
            if tj is not None:
                data = tj.finish(result)
                if self.trajectory_json:
                    TrajectoryRecorder.write_json(data, self.trajectory_json)
                    result["trajectory_json"] = self.trajectory_json
                if self.viewer_html:
                    TrajectoryRecorder.write_html(data, self.viewer_html)
                    result["viewer_html"] = self.viewer_html
            return result
        finally:
            world.close()

    def _loop(self, world: World, method: CaptureMethod, target: Target, servicer: Servicer,
              orbit: Optional["OrbitalDynamics"] = None, scenery: Optional["SpaceScenery"] = None,
              recorder: Optional["FrameRecorder"] = None, tj: Optional["TrajectoryRecorder"] = None) -> Dict:
        cfg = self.cfg
        thr = cfg["success"]["rel_omega_threshold_deg_s"]
        hold = cfg["success"]["hold_time"]
        max_time = cfg["sim"]["max_time"]
        d_initial = float(np.linalg.norm(servicer.pose()[0] - target.pose()[0]))
        escape_d = cfg["sim"]["escape_distance_factor"] * d_initial

        v_prev, w_prev = target.velocity()
        I = target.inertia_diag
        max_impulse = 0.0
        max_ang_impulse = 0.0
        peak_contact_force = 0.0
        max_att_err = 0.0
        thr_start: Optional[float] = None
        success = False
        success_time: Optional[float] = None
        rel_omega = float("nan")
        hist = {"t": [], "rel_omega_deg_s": [], "servicer_att_err_deg": [], "bond": []} if cfg["sim"]["record_history"] else None

        while world.t < max_time:
            if scenery is not None:
                scenery.poll_input()
                while scenery.paused:            # 일시정지: 물리는 멈추고 카메라/HUD 만 갱신
                    time.sleep(0.03)
                    scenery.poll_input()
                    scenery.update(method, rel_omega, method.bond_intact(), force=True)
            method.pre_step()
            servicer.attitude_control()
            if orbit is not None:
                orbit.apply()
            world.step()
            if VISUAL_OVERLAYS:
                SpacecraftVisuals.sync_overlays()

            # --- 대상 상태 / 충격량 (운동량 변화 기반: 접촉+제약 모두 포함)
            v, w = target.velocity()
            dp = target.mass * (v - v_prev)
            _, tq = target.pose()
            R = quat_to_R(tq)
            dL = R @ (I * (R.T @ (w - w_prev)))
            max_impulse = max(max_impulse, float(np.linalg.norm(dp)))
            max_ang_impulse = max(max_ang_impulse, float(np.linalg.norm(dL)))
            v_prev, w_prev = v, w

            contacts = p.getContactPoints(bodyA=target.body)
            if contacts:
                peak_contact_force = max(peak_contact_force, float(sum(c[9] for c in contacts)))
            method.post_step(contacts)

            # --- 상대 각속도 / 서비서 자세
            _, ws = servicer.velocity()
            rel_omega = math.degrees(float(np.linalg.norm(w - ws)))
            att_err = servicer.attitude_error_deg()
            max_att_err = max(max_att_err, att_err)
            bond = method.bond_intact()
            if scenery is not None:
                scenery.update(method, rel_omega, bond)
            if recorder is not None:
                recorder.capture()
            if tj is not None:
                tj.capture(rel_omega, bond)
            if hist is not None:
                hist["t"].append(world.t); hist["rel_omega_deg_s"].append(rel_omega)
                hist["servicer_att_err_deg"].append(att_err); hist["bond"].append(bond)

            # --- 성공 판정: 임계값 이하 + 결합 유지가 hold 시간 지속
            if bond and rel_omega <= thr:
                if thr_start is None:
                    thr_start = world.t
                elif world.t - thr_start >= hold:
                    success = True; success_time = thr_start
                    break
            else:
                thr_start = None

            # --- 조기 종료
            if method.should_abort():
                break
            if float(np.linalg.norm(servicer.pose()[0] - target.pose()[0])) > escape_d:
                method.notes.append("target escaped")
                break

        fc = method.first_contact_time
        result = {
            "success": success,
            "capture_time": (success_time - fc) if (success and fc is not None) else None,
            "impulse": max_impulse,                       # 대상에 전달된 스텝당 최대 선형 충격량 [N·s]
            "servicer_attitude_error": max_att_err,       # 서비서 자세 변화 최대값 [deg]
            # --- 보조 지표
            "angular_impulse": max_ang_impulse,           # 스텝당 최대 각충격량 [N·m·s]
            "peak_contact_force": peak_contact_force,     # 접촉점 법선력 합 최대 [N]
            "max_interaction_force": method.max_interaction_force,  # 파지/고정 제약에 걸린 최대 힘 [N]
            "final_rel_omega_deg_s": rel_omega,
            "servicer_attitude_error_final": servicer.attitude_error_deg(),
            "first_contact_time": fc,
            "bond_established_time": method.bond_established_time,
            "bond_lost_time": method.bond_lost_time,
            "bond_intact_final": method.bond_intact(),
            "sim_time": world.t,
            "method": self.method_name, "target": self.target_kind, "target_mass": target.mass,
            "spin_deg_s": self.spin_deg_s, "spin_mode": self.spin_mode,
            "nutation_deg": target.nutation_deg if self.spin_mode == "precession" else 0.0,
            "seed": self.seed,
            "notes": "; ".join(method.notes),
        }
        result.update(method.extra_metrics())
        if hist is not None:
            result["history"] = {k: np.array(v) for k, v in hist.items()}
        return result


def run_capture(method: str, target: str, spin_deg_s: float, spin_mode: str = "principal",
                nutation_deg: Optional[float] = None, seed: int = 0, gui: bool = False, cfg: Dict = CONFIG,
                record_frames: Optional[str] = None, record_video: Optional[str] = None,
                viewer_html: Optional[str] = None, trajectory_json: Optional[str] = None) -> Dict:
    """단일 시행 편의 함수. viewer_html 을 주면 three.js 3D 뷰어가 내장된 HTML 을 생성한다."""
    return CaptureSimulation(method, target, spin_deg_s, spin_mode, nutation_deg, seed, gui, cfg,
                             record_frames, record_video, viewer_html, trajectory_json).run()


# =============================================================================
# CLI
# =============================================================================
def _print_result(r: Dict):
    keys = ["method", "target", "spin_deg_s", "spin_mode", "seed", "success", "capture_time", "impulse",
            "servicer_attitude_error", "peak_contact_force", "max_interaction_force", "final_rel_omega_deg_s",
            "bond_intact_final", "first_contact_time", "sim_time", "notes"]
    for k in keys:
        v = r.get(k)
        if isinstance(v, float):
            v = f"{v:.4g}"
        print(f"  {k:28s}: {v}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="회전 우주 파편 포획 시뮬레이터 (PyBullet)")
    ap.add_argument("--method", choices=list(METHODS), default="arm")
    ap.add_argument("--target", choices=list(CONFIG["targets"]), default="medium")
    ap.add_argument("--spin", type=float, default=10.0, help="회전 속도 [deg/s] (0~30)")
    ap.add_argument("--spin-mode", choices=["principal", "precession"], default="principal")
    ap.add_argument("--nutation", type=float, default=None, help="세차 모드 장동각 [deg]")
    ap.add_argument("--spin-axis", choices=["x", "y", "z", "major", "minor"], default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gui", action="store_true", help="GUI 시각화 (기본 헤드리스)")
    ap.add_argument("--max-time", type=float, default=None)
    ap.add_argument("--no-orbit", action="store_true", help="LVLH 궤도 상대 역학(CW 방정식·중력구배) 끄기")
    ap.add_argument("--record-frames", default=None, metavar="DIR", help="PNG 프레임 녹화 폴더 (헤드리스도 가능)")
    ap.add_argument("--record-video", default=None, metavar="FILE.mp4", help="GUI 모드 mp4 녹화 (ffmpeg 필요)")
    ap.add_argument("--viewer", default=None, metavar="FILE.html", help="웹 3D 뷰어 HTML 생성 (헤드리스로 실행 후 브라우저에서 열기)")
    ap.add_argument("--trajectory", default=None, metavar="FILE.json", help="궤적 JSON 저장")
    ap.add_argument("--open", action="store_true", help="--viewer 로 만든 HTML 을 기본 브라우저로 열기")
    ap.add_argument("--sweep", action="store_true", help="방식 x 대상 x 회전속도 전체 조합 실행")
    ap.add_argument("--spins", type=float, nargs="+", default=[0, 5, 10, 20, 30])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--out", default="results.csv")
    args = ap.parse_args(argv)

    if args.spin_axis:
        CONFIG["target_dynamics"]["spin_axis"] = args.spin_axis
    if args.max_time:
        CONFIG["sim"]["max_time"] = args.max_time
    if args.no_orbit:
        CONFIG["orbit"]["enabled"] = False

    if not args.sweep:
        r = run_capture(args.method, args.target, args.spin, args.spin_mode, args.nutation, args.seed, args.gui,
                        record_frames=args.record_frames, record_video=args.record_video,
                        viewer_html=args.viewer, trajectory_json=args.trajectory)
        _print_result(r)
        if args.viewer:
            print(f"  viewer -> {os.path.abspath(args.viewer)}")
            if args.open:
                import webbrowser
                webbrowser.open("file:///" + os.path.abspath(args.viewer).replace("\\", "/"))
        return r

    rows = []
    for method in METHODS:
        for tk in CONFIG["targets"]:
            for spin in args.spins:
                for seed in args.seeds:
                    t0 = time.time()
                    r = run_capture(method, tk, spin, args.spin_mode, args.nutation, seed, False)
                    r.pop("history", None)
                    rows.append(r)
                    print(f"{method:8s} {tk:7s} spin={spin:5.1f} seed={seed} -> success={r['success']} "
                          f"capture_time={r['capture_time']} impulse={r['impulse']:.2f} "
                          f"att_err={r['servicer_attitude_error']:.2f} ({time.time() - t0:.1f}s)")
    keys = [k for k in rows[0].keys()]
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        for r in rows:
            wr.writerow({k: r.get(k) for k in keys})
    print(f"saved {len(rows)} rows -> {args.out}")
    return rows


if __name__ == "__main__":
    main()
