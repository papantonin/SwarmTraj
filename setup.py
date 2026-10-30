"""Optional native wheel build; ordinary installation remains pure Python."""
import hashlib
import os
from pathlib import Path
from setuptools import Extension, setup

extensions = []
if os.environ.get('SWARM_BUILD_NATIVE') == '1':
    root = Path(__file__).parent
    files = ['include/swarmtraj/flight.hpp', 'src/flight.cpp', 'include/swarmtraj/execution.hpp', 'src/execution.cpp', 'src/python.cpp']
    digest = hashlib.sha256(''.join(hashlib.sha256((root/'native'/p).read_bytes()).hexdigest() for p in files).encode()).hexdigest()
    generated = root/'build/native-setuptools'
    generated.mkdir(parents=True, exist_ok=True)
    (generated/'build_info.hpp').write_text(f'#define SWARM_SOURCE_SHA "{digest}"\n')
    flags = ['/std:c++20', '/fp:strict'] if os.name == 'nt' else ['-std=c++20', '-fno-fast-math', '-ffp-contract=off']
    extensions = [Extension('swarm_sim._trajectory_native',
        sources=['native/src/flight.cpp', 'native/src/execution.cpp', 'native/src/python.cpp'],
        depends=['native/include/swarmtraj/flight.hpp', 'native/include/swarmtraj/execution.hpp', 'build/native-setuptools/build_info.hpp'],
        include_dirs=['native/include', 'build/native-setuptools'],
        extra_compile_args=flags, language='c++')]
setup(ext_modules=extensions)
