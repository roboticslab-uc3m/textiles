from PySide import QtCore, QtGui
from PySide import QtUiTools
import os
import sys
from collections import namedtuple

from utils import load_results_data

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

# Named tuple to store evaluation results
Evaluation = namedtuple('Evaluation', 'name stage1 stage2 stage3')

class TextilesEvaluationWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        # Widgets and UI
        self.greatResultButton = None
        self.goodResultButton = None
        self.badResultButton = None
        self.infoLabel = None
        self.graphicsView = None
        self.pixmap = QtGui.QPixmap()

        # Iteration over input data
        self.input_results = None
        self.input_iterator = None
        self.current_result = None
        self.output_data = []

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

        self.updateImage(os.path.expanduser('~/Research/garments-birdsEye-flat-results/hoodie1-clustering.png'))


    def updateImage(self, filename):
        """
        Set pixmap in widget's graphics view
        """
        # Load image
        if filename:
            with open(filename, 'r') as f:
                image = f.read()

            self.pixmap.loadFromData(image, os.path.splitext(filename)[1])
        else:
            return False

        # create scene
        scene = QtGui.QGraphicsScene()
        scene.addItem(QtGui.QGraphicsPixmapItem(self.pixmap))
        self.graphicsView.setScene(scene)
        self.graphicsView.show()

    def onGreatButtonClicked(self):
        print "Great!"
        self.onButtonClicked()

    def onGoodButtonClicked(self):
        print "Good!"
        self.onButtonClicked()

    def onBadButtonClicked(self):
        print "Bad!"
        self.onButtonClicked()

    def onButtonClicked(self):
        """
        Try to load next image, if there is no image left, load the next item
        """
        try:
            image_file = self.current_result.next()
        except StopIteration, e:
            image_file = self.load_next_result()

        self.updateImage(image_file)

    def load_next_result(self):
        """
        Load the next item in input results
        """
        try:
            self.current_result = iter(self.input_iterator.next())
        except StopIteration, e:
            print "nothing left!"
            QtCore.QCoreApplication.instance().quit()
        else:
            garment_name = self.current_result.next()
            bumpiness_data = self.current_result.next()
            self.infoLabel.setText("Garment: {}\t\t Bumpiness:{}".format(garment_name, bumpiness_data))
            image_file = self.current_result.next()
            return image_file

    def saveDataToFile(self, filename):
        with open(filename, 'a') as f:
            for datum in self.data:
                f.write(datum)
                f.write('\n')

    def start(self, folder):
        """
        Load files in folder and starts the evaluation process
        """
        self.input_results = load_results_data(folder)
        self.input_iterator = iter(self.input_results)
        try:
            self.current_result = iter(self.input_iterator.next())
        except StopIteration, e:
            print "Exit..."
        else:
            garment_name = self.current_result.next()
            bumpiness_data = self.current_result.next()
            self.infoLabel.setText("Garment: {}\t\t Bumpiness:{}".format(garment_name, bumpiness_data))
            image_file = self.current_result.next()

        self.updateImage(image_file)





if __name__ == '__main__':
    # print [i.bumpiness for i in load_results_data(os.path.expanduser('~/Research/garments-birdsEye-flat-results'))]

    # Create Qt App
    app = QtGui.QApplication(sys.argv)

    # Create the widget and show it
    gui = TextilesEvaluationWidget()
    gui.start(os.path.expanduser('~/Research/garments-birdsEye-flat-results'))
    gui.show()

    # Run the app
    sys.exit(app.exec_())
