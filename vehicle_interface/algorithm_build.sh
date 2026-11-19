#!/usr/bin/env bash
# Target-native build. Instantiates no ROS node and uses no network.
set -euo pipefail
base=$(cd "$(dirname "$0")/.." && pwd)
v8="$base/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
prefix=$(realpath "${ACADOS_INSTALL_DIR:-$base/.deps/acados-v0.4.0}")
if [[ ! -f "$prefix/lib/libacados.so" ]]; then
    echo 'Set ACADOS_INSTALL_DIR to the target-native acados v0.4.0 installation.' >&2
    exit 2
fi
out="$base/build/vehicle_shadow"
gen="$v8/generated_joint"
mkdir -p "$out"
exec 9>"$out/build.lock"
flock 9
python3 "$v8/export_assets.py"
inc=(-I"$gen" -I"$prefix/include" -I"$prefix/include/hpipm/include" -I"$prefix/include/blasfeo/include")
link=(-L"$prefix/lib" -Wl,--disable-new-dtags "-Wl,-rpath,$prefix/lib" -lacados -lhpipm -lblasfeo)
"${CC:-cc}" -O2 -fPIC -shared "${inc[@]}" "$gen/acados_solver_joint_vehicle.c" \
    "$gen"/joint_vehicle_{model,cost,constraints}/*.c "${link[@]}" -lm -o "$out/physical_solver.so.tmp"
mv "$out/physical_solver.so.tmp" "$out/physical_solver.so"
"${CXX:-g++}" -std=c++14 -O2 -ffp-contract=off "${inc[@]}" -I/usr/include/eigen3 \
    -I/usr/include/jsoncpp -I"$v8/src/cpp_frenet" -I"$base/Code/sim_ws/src/takeover_sim/include" \
    "$base/vehicle_interface/algorithm_native.cpp" "$v8/src/dynamic_mpc.cpp" "$v8/src/mpc_iteration.cpp" \
    -L"$out" '-Wl,-rpath,$ORIGIN' -l:physical_solver.so "${link[@]}" -ljsoncpp -o "$out/algorithm_native.tmp"
mv "$out/algorithm_native.tmp" "$out/algorithm_native"
