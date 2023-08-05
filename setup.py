from distutils.core import setup
from Cython.Build import cythonize
#from protobuf_distutils.generate_py_protobufs import generate_py_protobufs
from pathlib import Path

proto_files = []
for file in Path('proto').glob('**/*.proto'):
    proto_files.append(file.relative_to('proto').as_posix())

setup(
    ext_modules = cythonize('cython/astar.pyx')
    )