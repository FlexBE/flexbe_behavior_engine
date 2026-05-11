"""pytest configuration: stub out PySide6 when it is not installed.

test_input_action_server.py patches every PySide6 call it makes, so the
tests run correctly with lightweight stubs.  test_input_gui.py exercises
real widget behaviour and requires genuine PySide6, so it is excluded from
collection when only stubs are present.
"""
import sys
import types

try:
    import PySide6  # noqa: F401
    collect_ignore = []
except ImportError:
    collect_ignore = ['test_input_gui.py']

    # ------------------------------------------------------------------ #
    # Minimal PySide6 stubs sufficient for importing input_action_server  #
    # and input_gui without a real Qt installation.                       #
    # ------------------------------------------------------------------ #

    class _Signal:

        def __init__(self, *args):
            pass

        def connect(self, *args, **kwargs):
            pass

        def emit(self, *args):
            pass

    def _Slot(*_args, **_kwargs):
        def decorator(func):
            return func
        return decorator

    class _QObject:

        def __init__(self, *args, **kwargs):
            pass

        def __getattr__(self, name):
            return lambda *a, **kw: None

    class _QThread(_QObject):
        pass

    class _QCoreApplication:

        @staticmethod
        def quit():  # noqa: A003
            pass

    class _QApplication(_QObject):
        pass

    _qtcore = types.ModuleType('PySide6.QtCore')
    _qtcore.Signal = _Signal
    _qtcore.Slot = _Slot
    _qtcore.QThread = _QThread
    _qtcore.QCoreApplication = _QCoreApplication
    _qtcore.Qt = type('Qt', (), {'BlockingQueuedConnection': None})()
    _qtcore.QSize = type('QSize', (_QObject,), {})

    _qtwidgets = types.ModuleType('PySide6.QtWidgets')
    _qtwidgets.QApplication = _QApplication
    for _cls_name in ('QComboBox', 'QLabel', 'QLineEdit', 'QMainWindow',
                      'QPushButton', 'QVBoxLayout', 'QWidget'):
        setattr(_qtwidgets, _cls_name, type(_cls_name, (_QObject,), {}))

    _pyside6 = types.ModuleType('PySide6')
    _pyside6.QtCore = _qtcore
    _pyside6.QtWidgets = _qtwidgets

    sys.modules['PySide6'] = _pyside6
    sys.modules['PySide6.QtCore'] = _qtcore
    sys.modules['PySide6.QtWidgets'] = _qtwidgets
