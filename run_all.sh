#!/usr/bin/env bash
# 전체 연구 계산을 순서대로 실행한다 (CPU 경합을 피하려 순차 실행).
#   1) 본 격자 몬테카를로 실험   -> results/
#   2) 수치 검증 (스텝 수렴 등)  -> verification/
#   3) 가정값 민감도 분석        -> sensitivity/
#
# 각 단계는 --resume 을 지원하므로 중단 후 이 스크립트를 다시 실행하면 이어서 진행된다.
#
# 사용법:  bash run_all.sh [워커수]
set -u
PY="${DEBRIS_PY:-C:/Users/User/anaconda3/envs/debris-sim/python.exe}"
W="${1:-10}"
cd "$(dirname "$0")" || exit 1
mkdir -p logs

stamp() { date "+%Y-%m-%d %H:%M:%S"; }

echo "[$(stamp)] === 1/3 격자 몬테카를로 실험 (workers=$W) ==="
"$PY" run_grid_experiment.py --workers "$W" --resume --out results > logs/grid.log 2>&1
echo "[$(stamp)] grid exit=$?"

echo "[$(stamp)] === 2/3 수치 검증 ==="
"$PY" run_verification.py --out verification > logs/verification.log 2>&1
echo "[$(stamp)] verification exit=$?"

echo "[$(stamp)] === 3/3 가정값 민감도 분석 (workers=$W) ==="
"$PY" run_sensitivity.py --workers "$W" --repeats 6 --spins 12 --resume --out sensitivity > logs/sensitivity.log 2>&1
echo "[$(stamp)] sensitivity exit=$?"

echo "[$(stamp)] === 완료. results/ verification/ sensitivity/ 확인 ==="
