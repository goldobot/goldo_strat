import sys
from importlib.abc import SourceLoader, MetaPathFinder
from importlib.util import spec_from_file_location
from pathlib import Path


class MyMetaFinder(MetaPathFinder):
    def __init__(self):
        self.sequences_path = ''
        self.sequence_modules = {}
        self.inject_globals = {}

    def unload_all(self):
        for k, v in self.sequence_modules.items():
            try:
                del sys.modules[k]
            except Exception:
                pass
        self.sequence_modules = {}

    def find_spec(self, fullname, path, target=None):
        if fullname == 'sequences':
            file_location = Path(__file__).parents[0] / 'sequences/__init__.py'
            return spec_from_file_location(
                fullname,
                str(file_location),
                loader=MyLoader(self, fullname, str(file_location)))
        if fullname.startswith('sequences.'):
            file_location = self.sequences_path / Path(fullname[10:] + '.py')
            return spec_from_file_location(
                fullname,
                str(file_location),
                loader=MyLoader(self, fullname, str(file_location)))
        return None  # we don't know how to import this


class MyLoader(SourceLoader):
    def __init__(self, finder, fullname, path):
        self.fullname = fullname
        self.path = path
        self.finder = finder

    def get_filename(self, fullname):
        return self.path

    def get_data(self, filename):
        """exec_module is already defined for us, we just have to provide a way
        of getting the source code of the module"""
        with open(filename) as f:
            data = f.read()
        return data

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        data = self.get_data(self.path)
        module_dict = vars(module)
        for k, v in self.finder.inject_globals.items():
            module_dict[k] = v
        exec(data, vars(module))
        self.finder.sequence_modules[self.fullname] = module


meta_finder = MyMetaFinder()


def install():
    # insert the path hook ahead of other path hooks
    sys.meta_path.insert(0, meta_finder)
    # clear any loaders that might already be in use by the FileFinder
    # sys.path_importer_cache.clear()
    # invalidate_caches()


install()
