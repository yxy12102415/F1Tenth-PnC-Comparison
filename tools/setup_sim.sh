#!/usr/bin/env bash
# Install the Ubuntu 22.04 x86-64 Gym benchmark stack inside this checkout.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
deps="$repo/.deps"
sim_python="$deps/sim_env/bin/python"
acados="$deps/acados-v0.4.0"
gym="$deps/f1tenth_gym"
export PYTHONNOUSERSITE=1
python_bin="${FITZ_SYSTEM_PYTHON:-/usr/bin/python3}"
if [[ "$(uname -sm)" != "Linux x86_64" ]] || ! "$python_bin" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 10))'; then
    echo 'Use Ubuntu 22.04 x86-64 with Python 3.10 for the recorded simulator stack.' >&2
    exit 2
fi
for cmd in git curl cmake make cc c++ flock sha256sum; do
    command -v "$cmd" >/dev/null || { echo "Install $cmd as listed in docs/SIM_SETUP.md" >&2; exit 2; }
done
"$python_bin" -c 'import numpy, scipy, numba, matplotlib, yaml, PIL, OpenGL'
mkdir -p "$deps"
"$python_bin" -m venv --system-site-packages "$deps/sim_env"
"$sim_python" -m pip install 'pip==23.3.2' 'setuptools==65.5.0' 'wheel==0.38.4'
"$sim_python" -m pip install --no-build-isolation --no-deps \
    'gym==0.19.0' 'cloudpickle==1.6.0' 'pyglet==1.4.11' 'future==0.18.2' \
    'osqp==0.6.3' 'qdldl==0.1.9.post1' 'casadi==3.6.6' 'future-fstrings==1.2.0' 'jinja2==3.1.6'

checkout() {
    local url=$1 revision=$2 directory=$3
    if [[ ! -e "$directory" ]]; then
        git init -q "$directory"
        git -C "$directory" remote add origin "$url"
    fi
    if ! git -C "$directory" rev-parse --verify HEAD >/dev/null 2>&1; then
        git -C "$directory" fetch --depth 1 origin "$revision"
        git -C "$directory" checkout -q --detach FETCH_HEAD
    fi
    if [[ "$(git -C "$directory" rev-parse HEAD)" != "$revision" ]]; then
        echo "Expected revision $revision in $directory" >&2
        exit 2
    fi
}
checkout https://github.com/f1tenth/f1tenth_gym.git 4fdb9c7e6fb7c701290f4dc18377d07c1681724f "$gym"
"$sim_python" -m pip install --no-build-isolation --no-deps -e "$gym"
checkout https://github.com/acados/acados.git 43354449192aa51557abda37d1bc17d907077354 "$acados"
git -C "$acados" submodule update --init --depth 1 -- external/blasfeo external/hpipm external/qpoases
cmake -S "$acados" -B "$acados/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$acados" -DBUILD_SHARED_LIBS=ON -DACADOS_WITH_QPOASES=ON
cmake --build "$acados/build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$acados/build"
renderer="$acados/bin/t_renderer"
renderer_sha=390063f34a8e13620564b4a136012270168e1421dd7920a747048749e1d99718
mkdir -p "$acados/bin"
if ! echo "$renderer_sha  $renderer" | sha256sum --check --status; then
    curl -fL --retry 3 https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -o "$renderer.download"
    echo "$renderer_sha  $renderer.download" | sha256sum --check
    chmod +x "$renderer.download"
    mv "$renderer.download" "$renderer"
fi
"$sim_python" -c 'import gym, f110_gym, osqp, casadi'
ACADOS_INSTALL_DIR="$acados" FITZ_ACADOS_SOURCE_DIR="$acados" FITZ_CODEGEN_PYTHON="$sim_python" \
    bash "$repo/Code/sim_ws/tools/build_fitz_tracking_joint.sh"
