from collections import namedtuple
from operator import itemgetter

from textiles.unfolding.perception.GarmentUtils import load_results_data
from textiles.common.errors import DependencyNotInstalled
try:
    from PySide import QtCore, QtGui
    from PySide import QtUiTools
except ImportError as e:
    raise DependencyNotInstalled(
        ("{}. (HINT: you need to install PySide 1 for this to work," +
         "check https://github.com/roboticslab-uc3m/textiles for more info.)").format(e))


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

        # Output data related
        self.output_data = []
        self.current_output = None
        self.output_data_path = os.path.abspath('.')

        self.setup_ui()

    def setup_ui(self):
        # Load UI and set it as main layout
        ui_file_path = os.path.join(os.path.realpath(os.path.dirname(__file__)), 'resources',
                                                                                 'TextilesEvaluationWidget.ui')
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
        self.greatResultButton.clicked.connect(self.on_great_button_clicked)
        self.goodResultButton.clicked.connect(self.on_good_button_clicked)
        self.badResultButton.clicked.connect(self.on_bad_button_clicked)

    def update_image(self, filename):
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

    def on_great_button_clicked(self):
        self.current_output.append(1)
        self.on_button_clicked()

    def on_good_button_clicked(self):
        self.current_output.append(0.5)
        self.on_button_clicked()

    def on_bad_button_clicked(self):
        self.current_output.append(0)
        image_file = self.load_next_result()
        self.update_image(image_file)

    def on_button_clicked(self):
        """
        Try to load next image, if there is no image left, load the next item
        """
        try:
            image_file = self.current_result.next()
        except StopIteration as e:
            image_file = self.load_next_result()

        self.update_image(image_file)

    def load_next_result(self):
        """
        Load the next item in input results
        """
        try:
            self.current_result = iter(self.input_iterator.next())
        except StopIteration as e:
            print("nothing left!")
            self.save_data_to_file(self.output_data_path)
            QtCore.QCoreApplication.instance().quit()
        else:
            # Info label setup
            garment_name = self.current_result.next()
            bumpiness_data = self.current_result.next()
            self.infoLabel.setText("Garment: {}\t\t Bumpiness:{}".format(garment_name, bumpiness_data))

            # Create new output entry (and do backup)
            self.current_output = list()
            self.current_output.append(garment_name)
            self.output_data.append(self.current_output)
            self.save_data_to_file(self.output_data_path)

            # Get first image
            image_file = self.current_result.next()
            return image_file

    def save_data_to_file(self, filename):
        with open(filename, 'w') as f:
            for entry in sorted(self.output_data, key=itemgetter(0)):
                try:
                    # Pad right with -1 for each non-present item
                    padded_entry = list(entry)
                    padded_entry.extend([-1]*(4-len(entry)))
                    for item in padded_entry:
                        f.write('{} '.format(item))
                    f.write('\n')
                except TypeError as e:
                    # nothing to write
                    print(e)

    def start(self, folder):
        """
        Load files in folder and starts the evaluation process
        """
        self.input_results = load_results_data(folder)
        self.input_iterator = iter(self.input_results)
        self.update_image(self.load_next_result())