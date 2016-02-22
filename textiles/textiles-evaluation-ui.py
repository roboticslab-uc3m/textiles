from PySide import QtCore, QtGui
from PySide import QtUiTools
import os
import sys

def load_ui(file_name, where=None):
    """
    Loads a .UI file into the corresponding Qt Python object
    :param file_name: UI file path
    :param where: Use this parameter to load the UI into an existing class (i.e. to override methods)
    :return: loaded UI
    """
    # Create a QtLoader
    loader = QtUiTools.QUiLoader()

    # Open the UI file
    ui_file = QtCore.QFile(file_name)
    ui_file.open(QtCore.QFile.ReadOnly)

    # Load the contents of the file
    ui = loader.load(ui_file, where)

    # Close the file
    ui_file.close()

    return ui

class TextilesEvaluationWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        self.greatResultButton = None
        self.goodResultButton = None
        self.badResultButton = None
        self.infoLabel = None
        self.graphicsView = None
        self.pixmap = QtGui.QPixmap()

        self.setupUI()

    def setupUI(self):
        # Load UI and set it as main layout
        ui_file_path = os.path.join(os.path.realpath(os.path.dirname(__file__)), 'TextilesEvaluationWidget.ui')
        main_widget = load_ui(ui_file_path, self)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(main_widget)
        self.setLayout(layout)

        # Get a reference to the widgets
        self.greatResultButton = self.findChild(QtGui.QPushButton, 'greatResultButton')
        self.goodResultButton = self.findChild(QtGui.QPushButton, 'goodResultButton')
        self.badResultButton = self.findChild(QtGui.QPushButton, 'badResultButton')
        self.infoLabel = self.findChild(QtGui.QLabel, 'infoLabel')
        self.graphicsView = self.findChild(QtGui.QGraphicsView, 'graphicsView')


        # Connect slots / callbacks
        self.greatResultButton.clicked.connect(self.onGreatButtonClicked)
        self.goodResultButton.clicked.connect(self.onGoodButtonClicked)
        self.badResultButton.clicked.connect(self.onBadButtonClicked)

        self.loadImage(os.path.expanduser('~/Research/garments-birdsEye-flat-results/hoodie1-clustering.png'))
        self.updateImage()

    def loadImage(self, filename):
        if filename:
            with open(filename, 'r') as f:
                image = f.read()

            self.pixmap.loadFromData(image, os.path.splitext(filename)[1])
            self.original_rect = self.pixmap.rect()
            self.pixmap = self.pixmap.scaled(480, 339)
            return True
        else:
            return False

    def updateImage(self):
        # Load image and create scene
        scene = QtGui.QGraphicsScene()
        scene.addItem(QtGui.QGraphicsPixmapItem(self.pixmap))
        self.graphicsView.setScene(scene)
        self.graphicsView.show()

    def onGreatButtonClicked(self):
        print "Great!"

    def onGoodButtonClicked(self):
        print "Good!"

    def onBadButtonClicked(self):
        print "Bad!"



if __name__ == '__main__':
    # Create Qt App
    app = QtGui.QApplication(sys.argv)

    # Create the widget and show it
    gui = TextilesEvaluationWidget()
    gui.show()

    # Run the app
    sys.exit(app.exec_())
