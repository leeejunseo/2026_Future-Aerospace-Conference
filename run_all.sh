#!/usr/bin/env bash
# 전체 연구 계산을 순서대로 실행한다 (CPU 경합을 피하려 순차 실행).
#   1) 본 격자 몬테카를로 실험   -> results/
#   2) 수치 검증 (스텝 수렴 등)  -> verification/
#   3) 가정값 민감도 분석        -> sensitivity/
#
# 각 단계는 --resume 을 지원하므로 중단 후 이 스크립트를 다시 실행하면 이어서 진행된다.
# 한 단계를 중단(Ctrl+C 또는 프로세스 종료)하면 다음 단계로 넘어가지 않고 전체가 멈춘다.
#
# 사용법:
#   bash run_all.sh            # 워커 10개로 1~3 단계 전부
#   bash run_all.sh 8          # 워커 8개
#   bash run_all.sh 10 grid    # 특정 단계만 (grid | verify | sens)
set -u
PY="${DEBRIS_PY:-C:/Users/User/anaconda3/envs/debris-sim/python.exe}"
W="${1:-10}"
ONLY="${2:-all}"
cd "$(dirname "$0")" || exit 1
mkdir -p logs

stamp() { date "+%Y-%m-%d %H:%M:%S"; }

# 중단(SIGINT/SIGTERM)되면 자식까지 정리하고 즉시 종료해, 다음 단계로 넘어가지 않도록 한다.
cleanup() { echo "[$(stamp)] 중단됨 - 남은 단계는 실행하지 않는다"; kill 0 2>/dev/null; exit 130; }
trap cleanup INT TERM

run_step() {
    local name="$1" log="$2"; shift 2
    echo "[$(stamp)] === $name 시작 ==="
    "$@" > "$log" 2>&1
    local rc=$?
    echo "[$(stamp)] === $name 종료 (exit=$rc) ==="
    if [ "$rc" -ne 0 ]; then
        echo "[$(stamp)] $name 이 실패했으므로 이후 단계를 중단한다. 로그: $log"
        tail -5 "$log"
        exit "$rc"
    fi
}

if [ "$ONLY" = "all" ] || [ "$ONLY" = "grid" ]; then
    run_step "1/3 격자 몬테카를로 실험 (workers=$W)" logs/grid.log \
        "$PY" run_grid_experiment.py --workers "$W" --resume --out results
fi
if [ "$ONLY" = "all" ] || [ "$ONLY" = "verify" ]; then
    run_step "2/3 수치 검증" logs/verification.log \
        "$PY" run_verification.py --out verification
fi
if [ "$ONLY" = "all" ] || [ "$ONLY" = "sens" ]; then
    run_step "3/3 가정값 민감도 분석 (workers=$W)" logs/sensitivity.log \
        "$PY" run_sensitivity.py --workers "$W" --repeats 6 --spins 12 --resume --out sensitivity
fi

echo "[$(stamp)] === 완료. results/ verification/ sensitivity/ 확인 ==="
