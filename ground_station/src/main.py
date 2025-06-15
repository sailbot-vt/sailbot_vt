import sys
from widgets.groundstation import GroundStationWidget
from widgets.camera_widget.camera import CameraWidget
from widgets.console_output import ConsoleOutputWidget
from icons import get_icons
import constants

from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt5.QtGui import QIcon


class MainWindow(QMainWindow):
    """
    Main window for the ground station application.

    Inherits
    -------
    `QMainWindow`
    """

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SailBussy Ground Station")
        self.setGeometry(constants.WINDOW_BOX)
        self.main_widget = QTabWidget()
        self.setCentralWidget(self.main_widget)
        self.main_widget.addTab(GroundStationWidget(), "Ground Station")
        self.main_widget.addTab(CameraWidget(), "Camera Feed")
        self.main_widget.addTab(ConsoleOutputWidget(), "Console Output")
        self.main_widget.setCurrentIndex(0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("SailBussy Ground Station")
    app.setStyle("Fusion")
    app_icon: QIcon = get_icons().boat
    app.setWindowIcon(app_icon)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
