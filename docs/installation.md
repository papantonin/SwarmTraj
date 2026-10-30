# Installing SwarmTraj on a new machine

SwarmTraj is a coordinated fixed-wing trajectory planner with a C++20 physical
planner and a Python interface using NumPy/SciPy and Matplotlib. PyBullet
examples and PX4/Gazebo experiments are separate optional backends. Docker,
ROS 2, PX4 and Gazebo are **not required** for trajectory planning.

## Supported starting point

Use **Ubuntu 24.04 LTS, x86-64, CPython 3.12** for the documented installation.
The Python package requires Python >= 3.12; newer versions and other platforms
are not covered by the clean-install checks. The shell launchers target Linux.
On Windows, use an Ubuntu 24.04 WSL2 environment for headless planning; WSL GUI
and PX4 container integration are not validated. Native Windows/macOS are not
currently supported installation paths. A Linux VM is another option.

You need internet access to Ubuntu package servers, PyPI and, for the full
backend, GitHub. PX4 setup also downloads an image from Docker Hub. After setup,
normal simulations use local dependencies. Python dependency ranges are not a
complete lockfile: future resolutions may differ. The PyBullet upstream commit,
MAVSDK version and PX4 image digest are pinned.

## 1. Install host prerequisites

On Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install -y git python3.12 python3.12-venv python3.12-dev python3-tk build-essential cmake
```

The default physical planner requires a C++20 compiler, CMake 3.20+ and Python
development headers. `python3-tk` is needed for the interactive Matplotlib
editor/viewer; headless planning can omit it. For the optional PyBullet backend,
also install the OpenGL libraries (PyBullet may use the compiler above if no
wheel is available):

```bash
sudo apt-get install -y libgl1 libegl1 libglib2.0-0
```

No script installs system packages, changes your Docker permissions or modifies
a ROS workspace. Do not run project setup or examples using `sudo`.

## 2. Obtain the repository

Clone the repository and enter its directory:

```bash
git clone https://github.com/papantonin/SwarmTraj.git
cd SwarmTraj
```

Run the following commands from that directory. Git is also needed by the full
setup to fetch its pinned upstream backend.
Do not copy another machine's `.venv`; create it on the destination machine.
[Python virtual environments are not portable](https://docs.python.org/3.12/library/venv.html).

## 3. Choose an installation

| Path | Command | Includes |
|---|---|---|
| Trajectory planner (recommended first) | `bash scripts/setup.sh --trajectory` | C++20 physical planner, NumPy, SciPy, Matplotlib, project, pytest |
| Full Python examples | `bash scripts/setup.sh --full` | Trajectory tools plus PyBullet and pinned gym-pybullet-drones |
| PX4/Gazebo experiment | Run trajectory setup, then `bash scripts/setup_px4_sitl.sh` | MAVSDK and the pinned Docker runtime |

Running `bash scripts/setup.sh` without arguments preserves the full-install
behavior. Setup creates `.venv` if absent, reuses a valid existing environment,
builds and tests the native planner, and fails on an unsupported Python version.
It does not delete environments. See [C++ planner defaults](planner_defaults.md)
for an explicit Python reference backend or rebuilding an existing installation.
Repeat it after dependency changes; switching from full to trajectory does not
uninstall optional packages. Set `PYTHON=python3.12` before the setup command if
`python3` selects a different interpreter. This only selects a new environment;
an existing `.venv` retains its original interpreter.

```bash
PYTHON=python3.12 bash scripts/setup.sh --trajectory
.venv/bin/python scripts/doctor.py
.venv/bin/python -m pip check
```

The project distribution is named `swarmtraj`. Python code continues to import
`swarm_sim`; the public import paths are preserved. If updating an environment
created before the project rename, first remove the old distribution metadata,
then rerun the desired setup path:

```bash
.venv/bin/python -m pip uninstall swarm-sim
bash scripts/setup.sh --trajectory
```

There is no need to activate the environment when using `.venv/bin/python`.
To use `python` directly, first run `source .venv/bin/activate`.

## 4. Verify the installation

Start with a single-process headless run. It generates plans, reports, telemetry
and a plot in a fresh directory under ignored `results/`:

```bash
.venv/bin/python examples/trajectory_3d.py --no-gui --workers 1
```

For the interactive editor/viewer, run on a graphical desktop:

```bash
.venv/bin/python examples/trajectory_3d.py
```

For the full Python backend:

```bash
.venv/bin/python scripts/doctor.py --full
.venv/bin/python examples/formation.py --duration 1 --no-gui
.venv/bin/python examples/fixed_wing_ring.py --duration 1 --no-gui
```

For contributor checks:

```bash
# Core tests usable with the trajectory installation
./scripts/test.sh tests/test_flight3d.py tests/test_physical_generation.py tests/test_cpp_production.py
# Entire suite requires the full Python installation, but no live PX4 flight
./scripts/test.sh
```

The full backend deliberately omits upstream-declared `control`, `torch` and
`stable-baselines3` packages because these examples do not use the RL stack.
Consequently raw `pip check` reports these three missing upstream requirements
after a full installation. `doctor.py` recognizes only these exact omissions
when the installed upstream commit matches the pinned commit, reports them
explicitly, and still fails on other missing packages or version conflicts.
Do not install the large RL stack just to clear those expected metadata messages.

A successful diagnostic checks imports and dependency metadata. The headless
runs additionally exercise the applications. Neither proves a GUI works on a
different desktop, physical model validity or real-aircraft safety.

## 5. Optional PX4/Gazebo runtime

This fixture requires **Linux x86-64 and CPython 3.12**: it mounts the local
`.venv` read-only and uses it with the image's Python runtime. Use a conventional
Ubuntu system-Python environment. A custom pyenv/Conda interpreter or another
Python minor version may be incompatible even if host imports succeed.

Install Docker Engine using the
[official Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/).
Follow the [Linux post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/)
to make Docker usable by your account, then open a new session if required.
Membership of the `docker` group grants root-level privileges; choose the
installation policy appropriate for your machine. The scripts do not configure
it automatically. A local daemon is required because project files are bind
mounted; a remote Docker context cannot use the local checkout.

```bash
docker info
bash scripts/setup_px4_sitl.sh
.venv/bin/python scripts/doctor.py --px4
# Generate a reference first; no container flight is started here.
.venv/bin/python examples/px4_gazebo.py
# Explicitly start an isolated simulated flight.
.venv/bin/python examples/px4_gazebo.py --run
```

Allow several GB of disk space for the image, dependencies and generated logs;
flight experiments can take many minutes. No host Gazebo, ROS 2, Compose file,
PX4 build or local Docker image build is required. Setup pulls the immutable
image specified in `src/swarm_sim/px4_gazebo.py`; the runtime keeps its network
isolation and never needs flight-controller hardware access.

The diagnostic runs a temporary `--network none` container to verify that the
mounted Python executable and native dependencies actually import. It does not
start PX4 or Gazebo. A successful import check is not a completed-flight result.
See [the PX4 guide](px4_gazebo.md), [shared editor](gazebo_editor.md) and
[four-aircraft experiment](gazebo_fleet.md) for runtime limits and artifacts.

### Optional desktop and GPU

Start headless first. Gazebo GUI requires a local X11/XWayland desktop, `DISPLAY`,
a readable `XAUTHORITY` file and `/tmp/.X11-unix`. Your desktop must supply the
authorization file; it is not always `~/.Xauthority`. Do not use `xhost +` to
work around authorization failures. Diagnose prerequisites with:

```bash
.venv/bin/python scripts/doctor.py --px4 --gui
.venv/bin/python examples/px4_gazebo.py --run --gui --timeout 1200
```

`--gpu` optionally exposes only `/dev/dri/renderD128` for an Intel/AMD Mesa
renderer. It requires user read/write access to that node. NVIDIA passthrough
is not configured. Omit the flag for software rendering. Inspect `ogre2.log`
for the actual renderer; requesting GPU mode does not prove GPU use.

## Troubleshooting

| Symptom | Action |
|---|---|
| Python is older than 3.12 | Install Python 3.12 and select it with `PYTHON`; do not replace the OS Python |
| `ensurepip` / `venv` missing | Install `python3.12-venv` on Ubuntu 24.04 |
| Existing `.venv` is broken or from another machine | Move it aside and rerun setup; do not try to relocate it |
| `ModuleNotFoundError` | Use `.venv/bin/python`; rerun the appropriate setup path |
| Tk missing / no display | Install `python3-tk` for the system Python, or use `--no-gui` |
| PyBullet build fails | Install compiler/development prerequisites above; use trajectory-only setup if that backend is unnecessary |
| GitHub/PyPI/registry download fails | Check connectivity/proxy/CA certificates and retry; do not disable TLS verification |
| Docker permission denied / daemon unavailable | Check `docker info` as your account and Docker's post-install instructions |
| PX4 container Python fails | Use system CPython 3.12 on Ubuntu x86-64, recreate a local environment, rerun PX4 setup/doctor |
| Plot font cache warning | Ensure your user cache directory is writable, or set `MPLCONFIGDIR` to a writable directory |

## Validation scope

GitHub Actions defines fresh Ubuntu 24.04 / Python 3.12 installations for both
Python setup paths, dependency diagnostics, focused regression tests and
headless example runs. Its full-backend job runs the complete test suite.
The optional PX4 image, desktop GUI and GPU are not covered by that workflow.
Check the repository's [Actions page](https://github.com/papantonin/SwarmTraj/actions)
for actual workflow results; a workflow definition alone does not indicate a
successful run.
