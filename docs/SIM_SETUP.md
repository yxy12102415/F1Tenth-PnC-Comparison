# Fitzpatrick simulation setup

The two benchmark launchers run on Ubuntu 22.04 x86-64 with Python 3.10. They use the official single-agent F1TENTH Gym, a CPU C++ compiler, and an acados v0.4.0 native solver. The static-obstacle viewer additionally uses a graphical desktop with Tk. All simulation commands run from the root of this repository.

## Install on a fresh laptop

Install the Ubuntu packages once.

```bash
sudo apt-get update
sudo apt-get install -y git curl cmake build-essential util-linux libeigen3-dev libjsoncpp-dev \
  python3.10-venv python3-numpy python3-scipy python3-numba python3-matplotlib \
  python3-pil python3-pil.imagetk python3-tk python3-opengl libglu1-mesa python3-yaml
bash tools/setup_sim.sh
```

The setup script installs a local Python environment in `.deps/sim_env`, fetches Gym revision `4fdb9c7e6fb7c701290f4dc18377d07c1681724f` and acados revision `43354449192aa51557abda37d1bc17d907077354`, downloads the x86-64 Tera renderer with a SHA-256 check, generates the Gym-model C sources and compiles the native tracking executable. The benchmark launchers find this environment automatically. The simulator owns its own process and uses direct Gym observations. ROS and a vehicle connection are unnecessary for these runs.

## Run a trial

| Operation | Command |
|---|---|
| All five controllers, center start, seed 11 | `bash Code/sim_ws/run_fitz_tracking.sh --cases center --seeds 11` |
| All five planners, historical A+B layout, seed 11 | `bash Code/sim_ws/run_fitz_local.sh --scenarios AB --seeds 11` |
| Live map with a lattice trial and replay controls | `bash Code/sim_ws/run_fitz_local.sh --gui` |
| Reproduce the 30 tracking trials | `bash Code/sim_ws/run_fitz_tracking.sh` |
| Reproduce the 120 local-planning trials | `bash Code/sim_ws/run_fitz_local.sh --seeds 11 29 47` |

Each launcher prints its output directory. Supply `--output` and a new directory path to choose a destination. The full planning schedule includes known failures and exits with status 1 if any scheduled run fails its criteria. One running benchmark per machine gives more interpretable timing measurements. The controller and local-planning guides define the inputs, outputs, acceptance rules and timing. [Tracking](FITZPATRICK_FIXED_TRACKING.md) · [Local planning](FITZPATRICK_LOCAL_PLANNING.md)

An existing installation can use `FITZ_PYTHON` for the Gym Python executable, `ACADOS_INSTALL_DIR` for the acados v0.4.0 installation prefix, `FITZ_ACADOS_SOURCE_DIR` for matching templates and renderer, and `FITZ_CODEGEN_PYTHON` for a Python 3.10 environment containing CasADi 3.6.6, future-fstrings and Jinja2. The release supplement records the original package versions and verification hashes. [Campaign environment](../results/fitzpatrick_local_design_20261120/reproduction/environment_and_checksums.json)
