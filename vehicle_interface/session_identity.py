"""Read-only identity of the commissioned Fitzpatrick supervisor, on the Jetson itself."""
import hashlib
import os
from pathlib import Path
import pwd

RELEASE = Path("candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8")
BINDIR = RELEASE / "install_cpp/cpp_frenet/lib/cpp_frenet"
SUPERVISOR = RELEASE / "stand_follow.py"
PLANNER_EXE = BINDIR / "cpp_frenet_planner"
CONTROL_EXE = BINDIR / "cpp_frenet_controller"
ASSETS = RELEASE / "runtime_assets.json"
BINARY_DIGEST = "a2d52924871cbd06d912743b0edf5a593533bcf29c76c3dfd2e40ef81f97be92"
# Hashes read from the commissioned Jetson, 2026-11-20. Raw YAML/asset JSON
# bytes on the vehicle differ from path-normalized assets in exported archives.
PINNED = {
    SUPERVISOR: "96d4fc380993da37b5582a033490b3b1e480914040bb55811206b35144965b01",
    RELEASE / "stand_map_io_optimized.py": "31e820b54a29d72078f116cd21807d6f2d914cbbf8748a473c9c587bc3e7bae7",
    RELEASE / "stand_window.py": "9128e1924efc7dedf5cdccfb1a2d43b9d1a61853cd434b20b72096e4264676d5",
    ASSETS: "beb7a301f19b89b24d29ff32f5ae1b84fed2ba42b74c91f60252dbbd7825cd54",
    Path("maps/Fitzpatrick.yaml"): "5dc4e831cf98a4064736ddff34aa124bce5841c8058c15e840612ce9e31ce435",
    Path("maps/Fitzpatrick.pgm"): "ee4113aa1286de05986e8d10dc83678fa0a6f652e43c789edf701c8e0f4ce296",
    Path("config/Fitzpatrick_fixed_start.json"): "d8d112596ac5d05a13e3be619297e6a837c08e032abe514864454ac721469bc1",
    Path("routes/Fitzpatrick_fixed_start_arc_straight_v2.csv"): "759067c229598ed608a4ab47a11c67342836eb727d2b3dd26105fd3118baa83b",
    Path("routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv"): "f01017527f0cc78069d57db0396b63269b4007e7d8e15c6899e2c936612895d4",
    PLANNER_EXE: BINARY_DIGEST,
    CONTROL_EXE: BINARY_DIGEST,
}


def process_stat(directory):
    # /proc/<pid>/stat starts with pid (comm); comm itself can contain spaces or ')'.
    tail = (directory / "stat").read_text().rsplit(")", 1)[1].split()
    return int(tail[1]), int(tail[19])


def cmdline(directory):
    return [os.fsdecode(arg) for arg in (directory / "cmdline").read_bytes().split(b"\0") if arg]


def supervisors(proc):
    found = []
    for directory in proc.iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            argv = cmdline(directory)
            if len(argv) < 2 or not Path(argv[0]).name.startswith("python"):
                continue
            args = argv[1:]
            while args and args[0] in ("-u", "-B", "-E", "-s", "-S", "-I", "-O", "-OO"):
                args = args[1:]
            if not args or args[0].startswith("-"):
                continue
            script = Path(args[0])
            if not script.is_absolute():
                script = directory / "cwd" / script
            script = script.resolve()
            if script.name != "stand_follow.py" or "--offline-self-test" in args[1:]:
                continue
            _, started = process_stat(directory)
            run, zero = "--allow-ground-motion" in args[1:], "--zero-only" in args[1:]
            mode = "zero" if zero and not run else "run" if run and not zero else "unknown"
            found.append({"pid": int(directory.name), "start_ticks": started, "script": str(script), "mode": mode})
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return sorted(found, key=lambda row: row["pid"])


class SessionIdentity:
    def __init__(self, workspace=None, proc=Path("/proc")):
        home = Path(pwd.getpwuid(os.getuid()).pw_dir)
        self.workspace = Path(workspace or os.environ.get("PNC_COMMISSIONED_WORKSPACE",
                                                          home / "f1tenth_real_migration_ws_20260829")).resolve()
        self.proc = proc
        self.cache = {}

    def digest(self, path):
        # Modifications/replacements trigger rehashing on the next poll.
        with path.open("rb") as source:
            stat = os.fstat(source.fileno())
            key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_ctime_ns)
            if key not in self.cache:
                h = hashlib.sha256()
                for block in iter(lambda: source.read(1 << 20), b""):
                    h.update(block)
                self.cache[key] = h.hexdigest()
        return self.cache[key]

    def check(self, mode, pinned_process=None):
        matches = supervisors(self.proc)
        result = {"status": "MISSING_SUPERVISOR", "candidates": matches, "issues": []}
        if len(matches) != 1:
            if matches:
                result["status"] = "MULTIPLE_SUPERVISORS"
            return result
        p = matches[0]
        expected = self.workspace / SUPERVISOR
        if Path(p["script"]) != expected:
            result["status"] = "DIFFERENT_RELEASE"
            return result
        if mode != "inventory" and p["mode"] != mode:
            result["status"] = "DIFFERENT_MODE"
            return result
        if pinned_process and (p["pid"], p["start_ticks"]) != pinned_process:
            result["status"] = "SUPERVISOR_CHANGED"
            return result
        issues = result["issues"]
        result["file_sha256"] = {}
        try:
            directory = self.proc / str(p["pid"])
            if not (directory / "exe").resolve(strict=True).name.startswith("python"):
                issues.append("supervisor is not a Python executable")
            rawenv = set((directory / "environ").read_bytes().split(b"\0"))
            if not {b"ROS_DOMAIN_ID=85", b"ROS_LOCALHOST_ONLY=1"} <= rawenv:
                issues.append("supervisor DDS environment differs")
            for name, digest in PINNED.items():
                observed = self.digest(self.workspace / name)
                result["file_sha256"][str(name)] = observed
                if observed != digest:
                    issues.append("commissioned file differs: " + str(name))
            self.check_children(p["pid"], issues)
        except (OSError, ValueError) as exc:
            issues.append("identity evidence unavailable: " + str(exc))
        result["status"] = "VERIFIED" if not issues else "IDENTITY_UNVERIFIED"
        return result

    def check_children(self, pid, issues):
        # taskset execs each binary as a direct child. Check the executable actually
        # mapped by the process and the asset referenced in its launch arguments.
        found = {}
        for name in (PLANNER_EXE, CONTROL_EXE):
            found[name.name] = []
        for directory in self.proc.iterdir():
            if not directory.name.isdecimal():
                continue
            try:
                parent, _ = process_stat(directory)
                if parent != pid:
                    continue
                argv = cmdline(directory)
                if not argv or Path(argv[0]).name not in found:
                    continue
                name = Path(argv[0]).name
                correct_elf = self.digest(directory / "exe") == BINARY_DIGEST
                asset = [arg.split(":=", 1)[1] for arg in argv if arg.startswith("assets_file:=")]
                correct_asset = len(asset) == 1 and Path(asset[0]).is_absolute() and (
                    Path(asset[0]).resolve() == self.workspace / ASSETS)
                found[name].append(correct_elf and correct_asset)
            except (FileNotFoundError, ProcessLookupError):
                continue
        for name, valid in found.items():
            if valid != [True]:
                issues.append(name + " commissioned child executable/assets binding unverified")
