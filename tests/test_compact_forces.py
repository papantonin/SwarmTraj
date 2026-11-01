"""Analytical checks of the actual experimental C++ force implementation."""
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    output=tmp_path_factory.mktemp('compact-forces')/'qualification'
    subprocess.run(['.venv/bin/python','scripts/qualify_compact_forces.py','--output',str(output)],check=True,capture_output=True)
    report=json.loads((output/'report.json').read_text())
    assert report['passed'] and len(report['trials'])==24
    return output/'force_probe'


@pytest.mark.parametrize('velocity',[(30.,0.,0.),(30.,3.,-2.),(30.,-3.,2.),(0.,0.,0.)])
def test_brake_is_dissipative_and_adds_no_lift(probe,velocity):
    def force(brake):
        command=[str(probe),'1.225','.12','.045','.18','1.4','20',*map(str,velocity),'0',str(brake)]
        return np.fromstring(subprocess.run(command,check=True,capture_output=True,text=True).stdout,sep=' ')
    off,on=force(0),force(1)
    speed=np.linalg.norm(velocity)
    np.testing.assert_allclose(on[:3]-off[:3],-20*np.array(velocity)/speed if speed else np.zeros(3),atol=1e-12)
    assert on[4]==off[4]
    assert np.dot(on[:3]-off[:3],velocity)<=0


@pytest.mark.parametrize('brake',['-0.01','1.01','nan','inf'])
def test_invalid_brake_fails_even_without_flow(probe,brake):
    result=subprocess.run([str(probe),'1.225','.12','.045','.18','1.4','20','0','0','0','0',brake],capture_output=True)
    assert result.returncode!=0
