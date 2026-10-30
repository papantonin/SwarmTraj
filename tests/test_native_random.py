"""Exact NumPy PCG64 seeding, cached uint32 draws, rejection sampling and resume."""
import importlib.util
import numpy as np
import pytest
from swarm_sim.trajectory2d.native_backend import require_native

pytestmark=pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                            reason='optional C++ extension not built')


def words(entropy):
    result=[]
    for value in entropy:
        while True:
            result.append(value & 0xffffffff);value >>= 32
            if not value:break
    return result


def state_tuple(bitgen):
    s=bitgen.state;a=s['state']['state'];b=s['state']['inc'];mask=(1<<64)-1
    return (a>>64,a&mask,b>>64,b&mask,s['has_uint32'],s['uinteger'])


@pytest.mark.parametrize('entropy',[[7,3],[0,0],[19,1,8,7301],[2**100+13,2**48+7],list(range(20))])
def test_seed_sequence_raw_and_interleaved_numpy_draws(entropy):
    seed=np.random.SeedSequence(entropy);bitgen=np.random.PCG64(seed);rng=np.random.Generator(bitgen)
    script=[('raw',)]*20
    script += [('uniform',-2.,5.),('integer',2),('uniform',5.,9.),('integer',4)]*20
    script += [('integer',n) for n in (1,2**32,2**31+1,2**40+1,2**63-1)]*40
    expected=[]
    for op,*args in script:
        expected.append(int(bitgen.random_raw()) if op=='raw' else
                        float(rng.uniform(*args)) if op=='uniform' else int(rng.integers(args[0])))
    actual,state,generated=require_native()._rng_probe(words(entropy),script)
    assert actual==expected
    assert state==state_tuple(bitgen)
    assert generated==tuple(seed.generate_state(4,np.uint64))


def test_resume_preserves_pending_uint32_across_float_and_raw_draws():
    rng=np.random.default_rng(np.random.SeedSequence([7,3]))
    first=int(rng.integers(3));resume=state_tuple(rng.bit_generator)
    assert resume[4]==1
    actual,state,_=require_native()._rng_probe([7,3],[('integer',3)])
    assert actual==[first] and state==resume
    script=[('uniform',0.,0.),('raw',),('integer',3),('integer',2),('uniform',0.,1.)]
    expected=[float(rng.uniform(0,0)),int(rng.bit_generator.random_raw()),int(rng.integers(3)),int(rng.integers(2)),float(rng.uniform())]
    actual,state,_=require_native()._rng_probe([7,3],script,resume)
    assert actual==expected and state==state_tuple(rng.bit_generator)
