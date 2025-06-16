import numpy as np
import sys
import serial
import time
import threading
from PyQt6 import QtCore

STATUS_TIMEOUT = 0.01


def openSerial(port, baud):
    serialport = serial.serial_for_url(port, do_not_open=True)
    serialport.baudrate = baud
    serialport.parity = serial.PARITY_NONE
    serialport.stopbits = serial.STOPBITS_ONE
    serialport.bytesize = serial.EIGHTBITS
    serialport.dsrdtr = True
    serialport.dtr = True
    
    try:
        serialport.open()
    except serial.SerialException as e:
        sys.stderr.write("Could not open serial port {}: {}\n".format(serialport.name, e))
        raise

    return serialport


class FakeSerial():
    def read(*args):
        return bytes("", 'utf8')        
    
    def readline(*args):
        return bytes('\n', 'utf8')
        
    def write(*args):
        pass
        #print("Write", args)
        
        
class SerialInterface(QtCore.QObject):
    messageChanged = QtCore.pyqtSignal(str)
    stateChanged = QtCore.pyqtSignal(str)
    posChanged = QtCore.pyqtSignal(float, float, float, float)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.status_time = time.time()
        try:
            self.serialport = openSerial(port, baud)
        except:
            self.serialport = FakeSerial()
        self.m_state = None
        self.m_pos = None
        self.startReadThread()
        self.startStatusThread()

    @QtCore.pyqtProperty(str, notify=stateChanged)
    def state(self):
        return self.m_state

    @state.setter
    def state(self, state_):
        if state_ == self.m_state:
            return
        self.m_state = state_
        self.stateChanged.emit(state_)

    @QtCore.pyqtProperty(str, notify=posChanged)
    def pos(self):
        return self.m_pos

    @pos.setter
    def pos(self, pos_):
        if pos_ == self.m_pos:
            return
        self.m_pos = pos_
        self.posChanged.emit(*pos_[:3], time.time())

    def startStatusThread(self):
        self.status_thread = threading.Thread(target=self.get_status)
        self.status_thread.start()
    
    def startReadThread(self):
        self.read_thread = threading.Thread(target=self.read)
        self.read_thread.start()

    def read(self):
        while True:
            self.readline()

    def readline(self):
        message = self.serialport.readline()
        try:
            message = str(message, 'utf8').strip()
        except UnicodeDecodeError:
            print("Failed to decode", message)
            return
        if message == '':
            return
        if message.startswith("<") and message.endswith(">"):
            rest = message[1:-3].split('|')
            new_state = rest[0]
            if new_state != self.state:
                self.state = new_state
            for item in rest:
                if item.startswith("MPos"):
                    new_pos = [float(field) for field in item[5:].split(',')]
                    self.pos = new_pos
        else:
            self.messageChanged.emit(message)

    def get_status(self):
        while True:
            self.serialport.write(b"?")
            time.sleep(STATUS_TIMEOUT)

    def soft_reset(self):
        print("soft reset")
        self.serialport.write(b"\x18") # Ctrl-X

    def cancel(self):
        print("cancel")
        self.serialport.write(bytes([0x85]))

    def reset(self):
        print("reset\r")
        self.serialport.dtr = False
        time.sleep(.5)
        self.serialport.dtr = True

    def write(self, data):
        print("write", data)
        self.serialport.write(bytes(data, "utf-8"))
        print("write done")
        self.serialport.flush()
        print("flush done")