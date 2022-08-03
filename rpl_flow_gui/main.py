import importlib
import os
import sys

from PyQt5.QtWidgets import QApplication

from widget import NodeEditorWidget

if __name__ == '__main__':
    app = QApplication(sys.argv)

    w = NodeEditorWidget()
    w.show()

    for fn in os.listdir('plugins'):
        if fn.startswith('plugin_') and fn.endswith('.py'):
            plugin = importlib.import_module('plugins.' + fn[:-3])
            plugin.register(w)

    sys.exit(app.exec_())
