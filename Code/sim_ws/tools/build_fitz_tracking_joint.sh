#!/usr/bin/env bash
# Native headless build of the standard Gym-model NMPC and CPU MPPI backends.
set -euo pipefail
cd "$(dirname "$0")/.."
repo=$(cd ../.. && pwd)
v8="$repo/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
acados=$(realpath "${ACADOS_INSTALL_DIR:-$repo/.deps/acados-v0.4.0}")
generated="$PWD/build/takeover_sim/gym_model/generated_joint"
build="$PWD/build/fitz_tracking"
mkdir -p "$build"
exec 9>"$build/.build.lock"
flock 9
if [[ ! -f "$acados/lib/libacados.so" ]]; then
  echo 'Build acados v0.4.0 first (see docs/SIM_SETUP.md).' >&2
  exit 2
fi
bash tools/generate_gym_mpc.sh
includes=(-I"$generated" -I"$acados/include" -I"$acados/include/blasfeo/include"
          -I"$acados/include/hpipm/include")
solver="$build/libfitz_tracking_gym_solver.so"
sources=("$generated/acados_solver_joint_vehicle.c" "$generated"/joint_vehicle_{model,cost,constraints}/*.c)
link=(-L"$acados/lib" -Wl,--disable-new-dtags "-Wl,-rpath,$acados/lib" -lacados -lhpipm -lblasfeo)
if [[ ! -f "$solver" || -n "$(find "$generated" -name '*.[ch]' -newer "$solver" -print -quit)" || tools/build_fitz_tracking_joint.sh -nt "$solver" ]]; then
  "${CC:-cc}" -O2 -fPIC -shared "${includes[@]}" "${sources[@]}" "${link[@]}" -lm -o "$solver.tmp"
  mv "$solver.tmp" "$solver"
fi
binary="$build/fitz_tracking_joint"
source=tools/fitz_tracking_joint.cpp
support=("$v8/src/dynamic_mpc.cpp" "$v8/src/mpc_iteration.cpp")
if [[ ! -x "$binary" || "$source" -nt "$binary" || "$solver" -nt "$binary" ||
      -n "$(find "$v8/src/cpp_frenet" src/takeover_sim/include "${support[@]}" -newer "$binary" -print -quit)" ]]; then
  "${CXX:-c++}" -O2 -std=c++14 -DTAKEOVER_OFFICIAL_GYM_MODEL=1 "${includes[@]}" \
    -I/usr/include/eigen3 -I/usr/include/jsoncpp -I"$v8/src/cpp_frenet" -Isrc/takeover_sim/include \
    "$source" "${support[@]}" -L"$build" '-Wl,-rpath,$ORIGIN' -lfitz_tracking_gym_solver \
    "${link[@]}" -ljsoncpp -o "$binary.tmp"
  mv "$binary.tmp" "$binary"
fi
echo "$binary"
