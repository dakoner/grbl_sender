from PyQt6 import QtWidgets
from PyQt6.uic import loadUi
import serial_interface_qobject

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        loadUi("grbl_sender.ui", self)
 
        self.serial = serial_interface_qobject.SerialInterface('/dev/ttyUSB0')
        self.serial.posChanged.connect(self.onPosChange)
        self.serial.stateChanged.connect(self.onStateChange)
        self.serial.messageChanged.connect(self.onMessageChanged)

        self.lineEdit.returnPressed.connect(self.onLineEntered)
        self.resetButton.clicked.connect(self.reset)
        self.state = 'None'
        self.m_pos = [-1, -1, -1]

    def onLineEntered(self):
        line = self.lineEdit.text()
        print("line=", line)
        self.serial.write(line + "\n")
        
    def onMessageChanged(self, message):
        self.textEdit.append(message)

    def onPosChange(self, x, y, z, t):
        self.m_pos = [x,y,z]
        self.x_value.display(x)
        self.y_value.display(y)
        self.z_value.display(z)

    def onStateChange(self, state):
        self.state = state
        self.state_value.setText(state)


    def reset(self):
        self.serial.reset()

    def unlock(self):
        self.serial.write("$X\n")

    def home(self):
        self.serial.write("$H\n")

    def cancel(self):
        self.serial.cancel()
