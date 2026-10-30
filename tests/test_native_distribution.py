"""Absolute setup.py invocation must still supply relative wheel source paths."""
from pathlib import Path
import runpy
import pytest


@pytest.mark.parametrize('native',[False,True])
def test_required_wheel_source_paths_are_relative_for_absolute_setup_invocation(monkeypatch,native):
    setuptools=pytest.importorskip('setuptools')
    captured=[]
    monkeypatch.setattr(setuptools,'setup',lambda **kwargs:captured.append(kwargs))
    if native:monkeypatch.setenv('SWARM_BUILD_NATIVE','1')
    else:monkeypatch.delenv('SWARM_BUILD_NATIVE',raising=False)
    root=Path(__file__).resolve().parents[1]
    runpy.run_path(str(root/'setup.py'))
    extensions=captured[0]['ext_modules']
    extension,=extensions
    assert 'native/src/mission.cpp' in extension.sources
    assert 'native/src/python_bridge.hpp' in extension.depends
    for path in [*extension.sources,*extension.depends]:
        assert not Path(path).is_absolute()
        assert (root/path).is_file()


def test_wheel_staging_cannot_reintroduce_a_retired_backend(monkeypatch, tmp_path):
    import setuptools
    from setuptools.command.build_py import build_py
    captured = []
    monkeypatch.delenv('SWARM_BUILD_NATIVE', raising=False)
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: captured.append(kwargs))
    runpy.run_path(str(Path(__file__).resolve().parents[1]/'setup.py'))
    command = captured[0]['cmdclass']['build_py'](setuptools.Distribution())
    command.build_lib = str(tmp_path)
    staged = tmp_path/'swarm_sim/trajectory2d'
    staged.mkdir(parents=True)
    stale = staged/'physical_generation.py'
    valid = staged/'planning.py'
    native = staged.parent/'_trajectory_native.so'
    for path in (stale, valid, native):
        path.write_bytes(b'fixture')
    monkeypatch.setattr(build_py, 'run', lambda self: None)
    command.run()
    assert not stale.exists()
    assert valid.read_bytes() == native.read_bytes() == b'fixture'


def test_production_build_cannot_disable_the_native_extension(monkeypatch):
    monkeypatch.setenv('SWARM_BUILD_NATIVE', '0')
    with pytest.raises(RuntimeError, match='pure Python builds are retired'):
        runpy.run_path(str(Path(__file__).resolve().parents[1]/'setup.py'))
