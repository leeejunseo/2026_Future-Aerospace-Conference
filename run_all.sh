#!/usr/bin/env bash
# 전체 연구 계산을 순서대로 실행한다 (CPU 경합을 피하려 순차 실행).
#   1) 본 격자 몬테카를로 실험   -> results/
#   2) 방식 간 짝지은 통계 비교   -> comparison/   (격자 자료만 있으면 되므로 몇 초면 끝난다)
#   3) 수치 검증 (스텝 수렴 등)  -> verification/
#   4) 가정값 민감도 분석        -> sensitivity/
#   5) 회전축 추가 실험 (중형)   -> results_axis/
#
# 각 단계는 --resume 을 지원하므로 중단 후 이 스크립트를 다시 실행하면 이어서 진행된다.
# 한 단계를 중단(Ctrl+C 또는 프로세스 종료)하면 다음 단계로 넘어가지 않고 전체가 멈춘다.
#
# 사용법:
#   bash run_all.sh            # 워커 10개로 1~3 단계 전부
#   bash run_all.sh 8          # 워커 8개
#   bash run_all.sh 10 grid    # 특정 단계만 (grid | compare | verify | sens | axis)
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
    run_step "1/5 격자 몬테카를로 실험 (workers=$W)" logs/grid.log \
        "$PY" run_grid_experiment.py --workers "$W" --resume --out results
fi
if [ "$ONLY" = "all" ] || [ "$ONLY" = "compare" ]; then
    run_step "2/5 방식 간 짝지은 통계 비교" logs/comparison.log \
        "$PY" analyze_compare.py --results results --out comparison
fi
if [ "$ONLY" = "all" ] || [ "$ONLY" = "verify" ]; then
    run_step "3/5 수치 검증" logs/verification.log \
        "$PY" run_verification.py --out verification
fi
if [ "$ONLY" = "all" ] || [ "$ONLY" = "sens" ]; then
    run_step "4/5 가정값 민감도 분석 (workers=$W)" logs/sensitivity.log \
        "$PY" run_sensitivity.py --workers "$W" --repeats 6 --spins 12 --resume --out sensitivity
fi

if [ "$ONLY" = "all" ] || [ "$ONLY" = "axis" ]; then
    # 5) 회전축 추가 실험: 중형 대상을 접근축(x)과 패널축(y)으로 회전시킨다. z 축은 1) 의 자료를 쓴다.
    #    두 축을 워커를 반씩 나눠 동시에 돌려, 중간에 멈춰도 두 축 자료가 같은 반복 수만큼 쌓이게 한다.
    H=$(( W / 2 )); [ "$H" -lt 1 ] && H=1
    echo "[$(stamp)] === 5/5 회전축 추가 실험 시작 (x, y 동시, 축마다 workers=$H) ==="
    pids=()
    for ax in x y; do
        "$PY" run_grid_experiment.py --targets medium --spins $(seq 2 2 30) --spin-axis "$ax" \
            --workers "$H" --resume --out "results_axis/$ax" > "logs/axis_$ax.log" 2>&1 &
        pids+=($!)
    done
    rc=0
    for pid in "${pids[@]}"; do wait "$pid" || rc=$?; done
    echo "[$(stamp)] === 5/5 회전축 추가 실험 종료 (exit=$rc) ==="
    if [ "$rc" -ne 0 ]; then
        echo "[$(stamp)] 회전축 실험이 실패했다. 로그: logs/axis_x.log, logs/axis_y.log"
        exit "$rc"
    fi
    run_step "5/5 회전축별 비교 분석" logs/axis_analysis.log \
        "$PY" analyze_axis.py --out results_axis
fi

echo "[$(stamp)] === 완료. results/ comparison/ verification/ sensitivity/ results_axis/ 확인 ==="
echo "[$(stamp)] 보고서 생성: python make_report.py"
