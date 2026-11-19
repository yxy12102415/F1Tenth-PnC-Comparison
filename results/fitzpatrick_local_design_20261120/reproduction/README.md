# Reproducing the Fitzpatrick static-obstacle campaign

The experiment scheduled all five planners across eight layouts and seeds 11, 29 and 47. The source supplement includes the campaign’s Python runner, native solver inputs and generated C code, native controller, map, waypoints, obstacle geometry, recorded placement summary, and eight rasterized layouts. The environment JSON records the official Gym and acados commits, package versions, machine and compiler specifications, linked binary fingerprints, arguments and numeric thread settings.

All 34 entries in the campaign-start manifest matched at collection time. This standalone export keeps the results, campaign-start manifest, replay outputs, and archived benchmark Python source bytes. It omits a guide for a different experiment, includes only the geometry used by this benchmark from a shared header, and converts three original machine paths in the archived fixture to relative paths. The original archive digest and the affected historical member hashes are retained in `source_export_notes`. The source archive hash and per-member hashes in the environment JSON verify the exported supplement. Desktop timing depends on hardware and load.

## Verify the supplement

From this directory, run the following with Python 3.

```bash
python3 - <<'PY'
import hashlib, json, pathlib, tarfile
m = json.loads(pathlib.Path('environment_and_checksums.json').read_text())
archive = pathlib.Path('SOURCE.tar.gz')
assert hashlib.sha256(archive.read_bytes()).hexdigest() == m['source_archive_sha256']
with tarfile.open(archive, 'r:gz') as t:
    hashes = {p.name: hashlib.sha256(t.extractfile(p).read()).hexdigest() for p in t}
assert hashes == m['source_sha256']
print(f'Verified {len(hashes)} files')
PY
```

## Repeat with the recorded benchmark sources

From the root of this repository, create a temporary second checkout so the archived algorithm versions and initial reporting logic can be used there. Build the simulator according to the laptop setup guide. [Simulation setup](../../../docs/SIM_SETUP.md)

```bash
replay=$(mktemp -d)
git clone --local . "$replay/comparison"
cd "$replay/comparison"
bundle=results/fitzpatrick_local_design_20261120/reproduction/SOURCE.tar.gz
# Restore the campaign-time comparison code and inputs, keeping the standalone installers.
tar -xzf "$bundle" --wildcards \
  'Code/sim_ws/tools/*.py' 'Code/sim_ws/tools/*.cpp' 'Code/sim_ws/src/*' \
  'real_vehicle/*' 'results/fitzpatrick_v8_20260927/two_obstacle_summary.json'
bash tools/setup_sim.sh
bash Code/sim_ws/run_fitz_local.sh --seeds 11 29 47 --output results/reproduced_fitz_local
```

Each trial writes its status, control trace and planning events. A nonzero exit at the end of the completed schedule records at least one unsuccessful trial. To check the recorded commands against the physics model, run the replay tool from the checkout root. The tool writes a new replay-validation JSON in the named campaign directory.

```bash
OPENBLAS_NUM_THREADS=1 .deps/sim_env/bin/python Code/sim_ws/tools/replay_fitz_local.py results/reproduced_fitz_local
```
