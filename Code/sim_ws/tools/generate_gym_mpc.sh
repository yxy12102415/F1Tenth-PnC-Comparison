#!/usr/bin/env bash
# Separate build-time model generation. Does not touch shipped real-v8 codegen.
set -euo pipefail
cd "$(dirname "$0")/.."
repo_root=$(cd ../.. && pwd)
generator="$repo_root/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/generate_joint_solver.py"
output="$PWD/build/takeover_sim/gym_model"
python="${FITZ_CODEGEN_PYTHON:-$repo_root/.deps/sim_env/bin/python}"
if [ ! -x "$python" ]; then
  echo 'Run tools/setup_sim.sh or supply FITZ_CODEGEN_PYTHON as described in docs/SIM_SETUP.md.' >&2
  exit 2
fi
export TAKEOVER_MODEL_PROFILE=official_gym_native TAKEOVER_GENERATE_ONLY=1 TAKEOVER_CODEGEN_DIR="$output"
export ACADOS_SOURCE_DIR="${FITZ_ACADOS_SOURCE_DIR:-${ACADOS_INSTALL_DIR:-$repo_root/.deps/acados-v0.4.0}}"
export ACADOS_PYTHON_INTERFACE_PATH="$ACADOS_SOURCE_DIR/interfaces/acados_template/acados_template"
export TERA_PATH="$ACADOS_SOURCE_DIR/bin/t_renderer"
stamp="$output/.fitz_codegen_inputs"
fingerprint=$(sha256sum "$generator" tools/generate_gym_mpc.sh "$ACADOS_PYTHON_INTERFACE_PATH/c_templates_tera/acados_solver.in.c" "$TERA_PATH")
if [[ -f "$output/generated_joint/acados_solver_joint_vehicle.c" && -f "$stamp" && "$(cat "$stamp")" == "$fingerprint" ]]; then
  exit 0
fi
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$python" "$generator"
printf '%s\n' "$fingerprint" > "$stamp"
