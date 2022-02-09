import unittest


class ImportTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def test_import(self):
        try:
            import voxblox
        except ImportError:
            self.fail("voxblox not properly installed, please run `make`")

    def test_not_empty(self):
        """Checks that the generated python bindings are not empty."""
        import voxblox

        vdir = [x for x in dir(voxblox) if not x.startswith("__")]
        self.assertGreater(len(vdir), 4)

    def test_import_pybind(self):
        try:
            import voxblox.pybind
        except ImportError:
            self.fail("voxblox not properly installed, please run `make`")
