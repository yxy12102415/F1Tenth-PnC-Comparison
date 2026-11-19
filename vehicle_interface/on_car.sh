#!/usr/bin/env bash
# Use the same Foxy overlays as the commissioned vehicle; keep its installation intact.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ws=${PNC_COMMISSIONED_WORKSPACE:-"$HOME/f1tenth_real_migration_ws_20260829"}
entry="$ws/candidate_cpp_gpu_20260909/offline_env.sh"
usage() {
    cat <<'HELP'
on_car.sh installed-check | inventory | preflight-zero | preflight-run [observer options]
on_car.sh capture zero|run NEW.jsonl [--seconds N]
on_car.sh wait-zero NEW.jsonl [--seconds 1.5] [--timeout 120]
on_car.sh build-shadow | shadow --input CAPTURE --output NEW.jsonl [--methods ...]
on_car.sh nominal-stage --method METHOD | test-nominal-stage | test-guard
All observers are read-only. The guard wire test uses its own localhost ROS domain.
HELP
}
[[ $# -gt 0 ]] || { usage; exit 2; }
action=$1
shift
physical_observer() {
    exec bash "$entry" env ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 \
        "PNC_COMMISSIONED_WORKSPACE=$ws" python3 "$repo/vehicle_interface/preflight.py" "$@"
}
case "$action" in
    installed-check)
        [[ $# == 0 ]] || exit 2
        exec bash "$ws/fitzpatrick_avoidance_dynamic_mpc.sh" check ;;
    inventory) physical_observer --mode inventory "$@" ;;
    preflight-zero) physical_observer --mode zero "$@" ;;
    preflight-run) physical_observer --mode run "$@" ;;
    capture)
        [[ $# -ge 2 && ( "$1" == zero || "$1" == run ) ]] || { usage; exit 2; }
        mode=$1; dest=$2; shift 2
        physical_observer --mode "$mode" --snapshots "$dest" "$@" ;;
    wait-zero)
        exec bash "$entry" env ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 \
            "PNC_COMMISSIONED_WORKSPACE=$ws" python3 "$repo/vehicle_interface/commissioned_zero_capture.py" "$@" ;;
    build-shadow)
        [[ $# == 0 ]] || exit 2
        export ACADOS_INSTALL_DIR=${ACADOS_INSTALL_DIR:-"$ws/reference_sources/joint_mpc_20260926/acados_install"}
        exec bash "$repo/vehicle_interface/algorithm_build.sh" ;;
    shadow)
        export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
        exec python3 "$repo/vehicle_interface/shadow.py" "$@" ;;
    nominal-stage|test-nominal-stage)
        if [[ "$action" == test-nominal-stage ]]; then
            [[ $# == 0 ]] || exit 2
            set -- --dds
            exec_file=test_nominal_stage.py
        else
            exec_file=nominal_stage.py
        fi
        exec bash "$entry" env ROS_DOMAIN_ID=143 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
            python3 "$repo/vehicle_interface/$exec_file" "$@" ;;
    test-guard)
        [[ $# == 0 ]] || exit 2
        exec bash "$entry" env ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 python3 "$repo/vehicle_interface/test_virtual_guard.py" ;;
    -h|--help|help) usage ;;
    *) usage >&2; exit 2 ;;
esac
