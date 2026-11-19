#!/usr/bin/env bash
# Single-car fixed trajectory comparison in the official, headless Gym.
set -euo pipefail
sim_dir="$(cd "$(dirname "$0")" && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MPLBACKEND=Agg
repo=$(cd "$sim_dir/../.." && pwd)
python_bin="${FITZ_PYTHON:-$repo/.deps/sim_env/bin/python}"
if [[ -z "${FITZ_PYTHON:-}" ]]; then
    if [[ -x "$python_bin" ]]; then export PYTHONNOUSERSITE=1; else python_bin=python3; fi
fi
"$python_bin" "$sim_dir/tools/generate_takeover_assets.py" Fitzpatrick_sim >/dev/null
bash "$sim_dir/tools/build_fitz_tracking_joint.sh" >/dev/null
exec "$python_bin" "$sim_dir/tools/run_fitz_tracking.py" "$@"
