"""Absolute setup.py invocation must still supply relative wheel source paths."""
from pathlib import Path
import runpy
import pytest


@pytest.mark.parametrize('native',[False,True])
def test_optional_wheel_source_paths_are_relative_for_absolute_setup_invocation(monkeypatch,native):
    setuptools=pytest.importorskip('setuptools')
    captured=[]
    monkeypatch.setattr(setuptools,'setup',lambda **kwargs:captured.append(kwargs))
    if native:monkeypatch.setenv('SWARM_BUILD_NATIVE','1')
    else:monkeypatch.delenv('SWARM_BUILD_NATIVE',raising=False)
    root=Path(__file__).resolve().parents[1]
    runpy.run_path(str(root/'setup.py'))
    extensions=captured[0]['ext_modules']
    if not native:
        assert extensions==[]
        return
    extension,=extensions
    assert 'native/src/mission.cpp' in extension.sources
    assert 'native/src/python_bridge.hpp' in extension.depends
    for path in [*extension.sources,*extension.depends]:
        assert not Path(path).is_absolute()
        assert (root/path).is_file()
