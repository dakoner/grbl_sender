import sys
#sys.path.append("..")
#import mvsdk
import signal
from PyQt6 import QtWidgets
from main_window import MainWindow


class QApplication(QtWidgets.QApplication):
    def __init__(self, *argv):
        super().__init__(*argv)
        self.main_window = MainWindow()
        self.main_window.show()#showMaximized()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)    
    app.exec()
