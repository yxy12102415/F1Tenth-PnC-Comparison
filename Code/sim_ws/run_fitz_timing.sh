#!/usr/bin/env bash
# Headless paired Fitzpatrick timing experiment. No ROS or vehicle connection.
set -euo pipefail
sim_dir="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$sim_dir/../.." && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
python_bin="${FITZ_PYTHON:-$repo/.deps/sim_env/bin/python}"
if [[ ! -x "$python_bin" ]]; then
    echo 'Set FITZ_PYTHON or run bash tools/setup_sim.sh first.' >&2
    exit 2
fi
"$python_bin" "$sim_dir/tools/generate_takeover_assets.py" Fitzpatrick_sim >/dev/null
bash "$sim_dir/tools/build_fitz_tracking_joint.sh" >/dev/null
exec "$python_bin" "$sim_dir/tools/run_fitz_timing.py" "$@"
