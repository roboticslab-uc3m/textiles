import os
import sys


from textiles.unfolding.gui.TextilesEvaluationWidget import TextilesEvaluationWidget
from textiles.common.errors import DependencyNotInstalled
try:
    from PySide import QtGui
except ImportError as e:
    raise DependencyNotInstalled(
        ("{}. (HINT: you need to install PySide 1 for this to work," +
         "check https://github.com/roboticslab-uc3m/textiles for more info.)").format(e))

def main():
    # Create Qt App
    app = QtGui.QApplication(sys.argv)

    # Create the widget and show it
    gui = TextilesEvaluationWidget()
    gui.output_data_path = os.path.expanduser('~/Research/garments-birdsEye-flat-results/output.txt')
    gui.start(os.path.expanduser('~/Research/garments-birdsEye-flat-results'))
    gui.show()

    # Run the app
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
