#!/usr/bin/env python3
"""Check an installed environment without starting a simulation or changing it."""
from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_COMMIT = "f83fbdd4ee1c07e1afa52694963f3a9a889a0f56"


# The pinned upstream declares its unused RL stack as mandatory. Keep every
# other pip-check error visible; do not filter version conflicts or other pins.
OPTIONAL_UPSTREAM_OMISSIONS = {
    f"gym-pybullet-drones 2.2.0 requires {name}, which is not installed."
    for name in ("control", "stable-baselines3", "torch")
}


def unexpected_dependency_issues(output: str, pinned_backend: bool) -> list[str]:
    return [line for line in output.splitlines() if line.strip() and not (
        pinned_backend and line in OPTIONAL_UPSTREAM_OMISSIONS)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", action="store_true", help="Check the core trajectory installation (default)")
    parser.add_argument("--full", action="store_true", help="Also check the pinned PyBullet backend")
    parser.add_argument("--px4", action="store_true", help="Also check Docker/image and Python imports inside the isolated image")
    parser.add_argument("--gui", action="store_true", help="Also check Tk and local X11 authorization prerequisites")
    args = parser.parse_args()
    failures = []

    def check(label, action):
        try:
            detail = action()
            print(f"OK   {label}" + (f": {detail}" if detail else ""))
        except Exception as exc:
            failures.append(label)
            print(f"FAIL {label}: {exc}")

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    def run(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip())
        return result.stdout.strip()

    check("Python >= 3.12", lambda: require(sys.version_info >= (3, 12), sys.version))
    check("Project virtual environment", lambda: require(
        Path(sys.prefix).resolve() == (ROOT / ".venv").resolve() and sys.prefix != sys.base_prefix,
        "Run with .venv/bin/python scripts/doctor.py"))
    for module in ("numpy", "scipy", "matplotlib", "pytest", "swarm_sim.trajectory2d.planning"):
        check(module, lambda module=module: importlib.import_module(module) and None)
    def native_backend():
        from swarm_sim.trajectory2d.native_planning import require_planner
        from swarm_sim.trajectory2d.native_backend import require_execution_native
        require_planner()
        require_execution_native()
    check('Required C++ planning and execution backend', native_backend)
    def dependencies():
        result = subprocess.run([sys.executable, "-I", "-m", "pip", "check"],
                                capture_output=True, text=True, timeout=120)
        if result.returncode == 0:
            return result.stdout.strip()
        try:
            record = json.loads(importlib.metadata.distribution("gym-pybullet-drones").read_text("direct_url.json") or "{}")
            pinned = record.get("vcs_info", {}).get("commit_id") == UPSTREAM_COMMIT
        except importlib.metadata.PackageNotFoundError:
            pinned = False
        issues = unexpected_dependency_issues(result.stdout + result.stderr, pinned)
        if issues or not result.stdout.strip():
            raise RuntimeError("\n".join(issues) or "pip check failed")
        return "Only the pinned backend's intentionally omitted RL dependencies (control, stable-baselines3, torch) are missing"

    check("Dependency consistency", dependencies)
    if args.full:
        for module in ("pybullet", "gym_pybullet_drones.envs.CtrlAviary", "gym_pybullet_drones.control.DSLPIDControl"):
            check(module, lambda module=module: importlib.import_module(module) and None)

        def pinned_backend():
            record = json.loads(importlib.metadata.distribution("gym-pybullet-drones").read_text("direct_url.json") or "{}")
            require(record.get("vcs_info", {}).get("commit_id") == UPSTREAM_COMMIT,
                    "Run bash scripts/setup.sh --full to install the expected upstream commit")
        check("Pinned gym-pybullet-drones commit", pinned_backend)
    if args.gui:
        check("Matplotlib Tk GUI", lambda: importlib.import_module("tkinter") and None)
        check("Local X11/XWayland display", lambda: require(
            bool(os.environ.get("DISPLAY")) and Path("/tmp/.X11-unix").is_dir(), "No local X11 display; use --no-gui"))
        if args.px4:
            check("XAUTHORITY", lambda: require(
                bool(os.environ.get("XAUTHORITY")) and os.access(os.environ["XAUTHORITY"], os.R_OK),
                "Set XAUTHORITY to your readable desktop authorization file; see docs/installation.md"))
    if args.px4:
        check("PX4 host", lambda: require(
            platform.system() == "Linux" and platform.machine() == "x86_64" and sys.version_info[:2] == (3, 12),
            "The mounted PX4 environment requires Linux x86-64 and CPython 3.12"))
        check("MAVSDK 4.1.0", lambda: require(importlib.metadata.version("mavsdk") == "4.1.0", "Run scripts/setup_px4_sitl.sh"))
        check("Docker daemon access", lambda: run(["docker", "info", "--format", "{{.ServerVersion}} "]))

        def container_python():
            from swarm_sim.px4_gazebo import IMAGE
            run(["docker", "image", "inspect", IMAGE])
            return run(["docker", "run", "--rm", "--network", "none",
                        "--mount", f"type=bind,src={ROOT},dst=/workspace,readonly",
                        "--env", "PYTHONPATH=/workspace/src", "--env", "PYTHONDONTWRITEBYTECODE=1",
                        "--env", "MPLCONFIGDIR=/tmp/swarm-matplotlib",
                        "--entrypoint", "/workspace/.venv/bin/python", IMAGE, "-c",
                        "import numpy, scipy, matplotlib, mavsdk, swarm_sim.px4_gazebo; print('Container Python imports succeeded')"])
        check("Pinned image and mounted Python compatibility", container_python)
    print(f"\n{len(failures)} failed check(s)." if failures else "\nAll requested checks passed.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
