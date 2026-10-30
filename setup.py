"""Build the required C++20 production engine for every installation."""
import hashlib
import os
from pathlib import Path
from setuptools import Extension, setup
from setuptools.command.build_py import build_py

if os.environ.get('SWARM_BUILD_NATIVE', '1') != '1':
    raise RuntimeError('The C++ production engine is required; pure Python builds are retired.')

root = Path(__file__).parent


class ProductionBuildPy(build_py):
    """Do not ship retired modules left by an earlier build in the same tree."""

    def run(self):
        super().run()
        source = root/'src/swarm_sim'
        staged = Path(self.build_lib)/'swarm_sim'
        for path in staged.rglob('*.py'):
            if not (source/path.relative_to(staged)).is_file():
                path.unlink()


native = root/'native'
sources = sorted(native.glob('src/*.cpp'))
headers = sorted([*native.glob('include/**/*.hpp'), *native.glob('src/*.hpp')])
files = sorted(p.relative_to(native).as_posix() for p in [*sources, *headers])
digest = hashlib.sha256(''.join(hashlib.sha256((root/'native'/p).read_bytes()).hexdigest() for p in files).encode()).hexdigest()
generated = root/'build/native-setuptools'
generated.mkdir(parents=True, exist_ok=True)
(generated/'build_info.hpp').write_text(f'#define SWARM_SOURCE_SHA "{digest}"\n')
flags = ['/std:c++20', '/fp:strict'] if os.name == 'nt' else ['-std=c++20', '-fno-fast-math', '-ffp-contract=off']
extensions = [Extension('swarm_sim._trajectory_native',
    sources=[p.relative_to(root).as_posix() for p in sources],
    depends=[*[p.relative_to(root).as_posix() for p in headers], 'build/native-setuptools/build_info.hpp'],
    include_dirs=['native/include', 'build/native-setuptools'],
    extra_compile_args=flags, language='c++')]
setup(ext_modules=extensions, cmdclass={'build_py': ProductionBuildPy})
