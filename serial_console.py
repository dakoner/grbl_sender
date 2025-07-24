# main.py
import sys
import time
import re
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import QApplication, QMainWindow, QFileDialog, QMessageBox, QVBoxLayout, QWidget
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, QTimer
from PyQt6.uic import loadUi
import pyqtgraph.opengl as gl
import numpy as np

class SerialWorker(QObject):
    """
    Worker thread for handling serial communication.
    """
    received = pyqtSignal(str)
    finished = pyqtSignal()
    file_transfer_progress = pyqtSignal(int)
    file_transfer_complete = pyqtSignal(bool, str) # Success, message

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.is_sending_file = False
        self.file_content = []
        self.file_line_index = 0
        self.gcode_to_send = None
        self.cancel_transfer = False
        self.response_timer = QTimer()
        self.response_timer.setSingleShot(True)
        self.response_timer.timeout.connect(self.handle_timeout)

    def connect(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            self.thread = QThread.create(self.read_from_port)
            self.thread.start()
            return True
        except serial.SerialException as e:
            return False

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.running = False
            if self.thread:
                self.thread.quit()
                self.thread.wait()
            self.serial_port.close()
        self.finished.emit()

    def read_from_port(self):
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.received.emit(line)
                        if self.is_sending_file and 'ok' in line.lower():
                            self.response_timer.stop()
                            self.send_next_file_line()
                except serial.SerialException:
                    self.running = False
                    self.disconnect()
            time.sleep(0.01)

    def send_command(self, command):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write((command + '\n').encode('utf-8'))
            self.received.emit(f">>> {command}")

    def start_file_transfer(self, file_content):
        self.file_content = [line for line in file_content if line.strip() and not line.strip().startswith(';')]
        if not self.file_content:
            self.file_transfer_complete.emit(True, "File is empty or contains only comments.")
            return
            
        self.is_sending_file = True
        self.file_line_index = 0
        self.cancel_transfer = False
        self.send_next_file_line()

    def send_next_file_line(self):
        if self.cancel_transfer:
            self.is_sending_file = False
            self.response_timer.stop()
            self.file_transfer_complete.emit(False, "File transfer cancelled.")
            return

        if self.file_line_index < len(self.file_content):
            line = self.file_content[self.file_line_index]
            self.send_command(line)
            self.response_timer.start(60000) # 60 second timeout for 'ok'
            self.file_transfer_progress.emit(self.file_line_index + 1)
            self.file_line_index += 1
        else:
            self.is_sending_file = False
            self.file_transfer_complete.emit(True, "File transfer complete.")

    def stop_file_transfer(self):
        self.cancel_transfer = True

    def handle_timeout(self):
        self.is_sending_file = False
        self.file_transfer_complete.emit(False, "Transfer failed: No 'ok' received from device (60s timeout).")


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        loadUi('main_window.ui', self)

        self.command_history = []
        self.history_index = -1
        self.serial_worker = None
        self.serial_thread = None
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.request_status)
        
        self.gcode_lines = []
        self.path_points = np.array([])
        self.gcode_plot = None
        self.progress_plot = None

        self.setup_ui()
        self.populate_ports()
        self.populate_baudrates()
        self.setup_3d_viewer()

    def setup_ui(self):
        # Connect signals and slots
        self.connectButton.clicked.connect(self.toggle_connection)
        self.refreshPortsButton.clicked.connect(self.populate_ports)
        self.sendButton.clicked.connect(self.send_manual_command)
        self.commandLineEdit.returnPressed.connect(self.send_manual_command)
        self.commandLineEdit.installEventFilter(self)
        
        self.loadFileButton.clicked.connect(self.load_file)
        self.sendFileButton.clicked.connect(self.send_file)
        self.cancelSendButton.clicked.connect(self.cancel_file_send)
        
        self.homeButton.clicked.connect(lambda: self.send_control_command("$H"))
        self.unlockButton.clicked.connect(lambda: self.send_control_command("$X"))
        self.softResetButton.clicked.connect(lambda: self.send_control_command("\x18"))
        self.hardResetButton.clicked.connect(self.hard_reset)
        self.abortButton.clicked.connect(lambda: self.send_control_command("\x85"))

        self.laserToggleButton.clicked.connect(self.toggle_laser)
        
        # Jogging buttons
        self.xPlusButton.clicked.connect(lambda: self.jog("X"))
        self.xMinusButton.clicked.connect(lambda: self.jog("X", "-"))
        self.yPlusButton.clicked.connect(lambda: self.jog("Y"))
        self.yMinusButton.clicked.connect(lambda: self.jog("Y", "-"))
        self.zUpButton.clicked.connect(lambda: self.jog("Z"))
        self.zDownButton.clicked.connect(lambda: self.jog("Z", "-"))

    def setup_3d_viewer(self):
        self.viewer = gl.GLViewWidget()
        self.viewer.opts['distance'] = 40
        self.viewer_layout = QVBoxLayout(self.viewerFrame)
        self.viewer_layout.addWidget(self.viewer)
        
        grid = gl.GLGridItem()
        self.viewer.addItem(grid)

    def populate_ports(self):
        self.portComboBox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.portComboBox.addItem(port.device)

    def populate_baudrates(self):
        self.baudRateComboBox.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baudRateComboBox.setCurrentText('115200')

    def toggle_connection(self):
        if self.serial_worker is None:
            port = self.portComboBox.currentText()
            baudrate = int(self.baudRateComboBox.currentText())
            if not port:
                QMessageBox.warning(self, "Connection Error", "No serial port selected.")
                return

            self.serial_worker = SerialWorker(port, baudrate)
            if self.serial_worker.connect():
                self.serial_thread = QThread()
                self.serial_worker.moveToThread(self.serial_thread)
                self.serial_thread.started.connect(self.serial_worker.read_from_port)
                self.serial_worker.received.connect(self.update_console)
                self.serial_worker.finished.connect(self.on_serial_disconnected)
                self.serial_worker.file_transfer_progress.connect(self.update_file_progress)
                self.serial_worker.file_transfer_complete.connect(self.on_file_transfer_complete)
                self.serial_thread.start()
                
                self.connectButton.setText("Disconnect")
                self.consoleOutput.append("<<< Connected")
                self.status_timer.start(1000)
            else:
                QMessageBox.critical(self, "Connection Error", f"Failed to connect to {port}")
                self.serial_worker = None

        else:
            if self.laserToggleButton.isChecked():
                self.toggle_laser() # Turn off laser before disconnecting
            self.status_timer.stop()
            self.serial_worker.disconnect()

    def on_serial_disconnected(self):
        self.connectButton.setText("Connect")
        self.consoleOutput.append("<<< Disconnected")
        if self.serial_thread:
            self.serial_thread.quit()
            self.serial_thread.wait()
        self.serial_worker = None
        self.serial_thread = None

    def send_manual_command(self):
        command = self.commandLineEdit.text()
        if command and self.serial_worker:
            self.serial_worker.send_command(command)
            if not self.command_history or self.command_history[-1] != command:
                 self.command_history.append(command)
            self.history_index = len(self.command_history)
            self.commandLineEdit.clear()

    def eventFilter(self, source, event):
        if source is self.commandLineEdit:
            if event.type() == event.Type.KeyPress:
                if event.key() == 16777235:  # Up arrow
                    if self.history_index > 0:
                        self.history_index -= 1
                        self.commandLineEdit.setText(self.command_history[self.history_index])
                    return True
                elif event.key() == 16777237:  # Down arrow
                    if self.history_index < len(self.command_history) -1:
                        self.history_index += 1
                        self.commandLineEdit.setText(self.command_history[self.history_index])
                    else:
                        self.history_index = len(self.command_history)
                        self.commandLineEdit.clear()
                    return True
        return super().eventFilter(source, event)

    @pyqtSlot(str)
    def update_console(self, text):
        self.consoleOutput.append(f"<<< {text}")
        self.consoleOutput.verticalScrollBar().setValue(self.consoleOutput.verticalScrollBar().maximum())
        self.parse_status(text)

    def load_file(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open G-code File", "", "G-code Files (*.gcode *.nc *.tap);;All Files (*)")
        if file_name:
            with open(file_name, 'r') as f:
                self.gcode_lines = f.readlines()
            self.parse_and_draw_gcode()
            self.consoleOutput.append(f">>> Loaded file: {file_name}")
            self.fileProgressBar.setValue(0)

    def send_file(self):
        if not self.serial_worker:
            QMessageBox.warning(self, "File Transfer", "Not connected to a device.")
            return

        if not self.gcode_lines:
            QMessageBox.warning(self, "File Transfer", "No file loaded. Please load a file first.")
            return

        self.serial_worker.start_file_transfer(self.gcode_lines)
        self.status_timer.stop()
        self.cancelSendButton.setEnabled(True)

    def cancel_file_send(self):
        if self.serial_worker and self.serial_worker.is_sending_file:
            self.serial_worker.stop_file_transfer()
            self.cancelSendButton.setEnabled(False)

    def update_file_progress(self, line_num):
        total_lines = len(self.serial_worker.file_content)
        self.fileProgressBar.setValue(int((line_num / total_lines) * 100))
        self.update_progress_plot(line_num)

    def on_file_transfer_complete(self, success, message):
        QMessageBox.information(self, "File Transfer", message)
        self.fileProgressBar.setValue(100 if success else 0)
        if not self.serial_worker.is_sending_file: # Check if another transfer has started
            self.status_timer.start(1000)
        self.cancelSendButton.setEnabled(False)

    def send_control_command(self, command):
        if self.serial_worker:
            self.serial_worker.send_command(command)

    def hard_reset(self):
        if self.serial_worker and self.serial_worker.serial_port:
            self.serial_worker.serial_port.dtr = not self.serial_worker.serial_port.dtr
            time.sleep(0.1)
            self.serial_worker.serial_port.dtr = not self.serial_worker.serial_port.dtr
            self.consoleOutput.append(">>> Hard Reset (DTR toggled)")

    def toggle_laser(self):
        if self.serial_worker:
            if self.laserToggleButton.isChecked():
                power = self.laserPowerSpinBox.value()
                self.send_control_command(f"M3 S{power}")
                self.laserToggleButton.setText("Laser OFF")
            else:
                self.send_control_command("M5")
                self.laserToggleButton.setText("Laser ON")

    def request_status(self):
        if self.serial_worker and not self.serial_worker.is_sending_file:
            self.send_control_command("?")

    def parse_status(self, status_string):
        if status_string.startswith('<') and status_string.endswith('>'):
            parts = status_string[1:-1].split('|')
            self.stateLabel.setText(parts[0])
            for part in parts[1:]:
                if part.startswith("WPos:") or part.startswith("MPos:"):
                    coords_part = part.split(':')[1]
                    coords = coords_part.split(',')
                    if len(coords) >= 3:
                        self.xPosLabel.setText(coords[0])
                        self.yPosLabel.setText(coords[1])
                        self.zPosLabel.setText(coords[2])
                    
    def jog(self, axis, direction="+"):
        if self.serial_worker:
            dist_xy = self.xyDistanceSpinBox.value()
            dist_z = self.zDistanceSpinBox.value()
            feed = self.feedRateSpinBox.value()
            
            distance = dist_xy if axis in "XY" else dist_z
            if direction == "-":
                distance = -distance
                
            command = f"$J=G91 {axis}{distance} F{feed}"
            self.send_control_command(command)

    def _parse_gcode_to_points(self, gcode_lines):
        """Helper function to parse G-code lines into a numpy array of 3D points."""
        points = []
        current_pos = np.array([0., 0., 0.])
        points.append(current_pos.copy())
        absolute_mode = True  # Default to G90 (absolute)
        last_move_command = None # Track G0 or G1

        for i, line in enumerate(gcode_lines):
            clean_line = line.split(';')[0].strip().upper()
            if not clean_line:
                continue

            try:
                # Check for modal commands first
                if re.search(r'\bG90\b', clean_line):
                    absolute_mode = True
                if re.search(r'\bG91\b', clean_line):
                    absolute_mode = False
                
                g0_match = re.search(r'\bG0\b', clean_line)
                g1_match = re.search(r'\bG1\b', clean_line)
                if g0_match or g1_match:
                    last_move_command = 'G0' if g0_match else 'G1'

                # Check for axis coordinates
                x_match = re.search(r'X([-\d.]+)', clean_line)
                y_match = re.search(r'Y([-\d.]+)', clean_line)
                z_match = re.search(r'Z([-\d.]+)', clean_line)

                # If we have a move command and at least one axis, process the move
                if last_move_command and any([x_match, y_match, z_match]):
                    if absolute_mode:
                        new_pos = current_pos.copy()
                        if x_match: new_pos[0] = float(x_match.group(1))
                        if y_match: new_pos[1] = float(y_match.group(1))
                        if z_match: new_pos[2] = float(z_match.group(1))
                    else:  # Relative mode
                        move = np.array([0., 0., 0.])
                        if x_match: move[0] = float(x_match.group(1))
                        if y_match: move[1] = float(y_match.group(1))
                        if z_match: move[2] = float(z_match.group(1))
                        new_pos = current_pos + move
                    
                    if not np.array_equal(new_pos, current_pos):
                        points.append(new_pos)
                        current_pos = new_pos

            except (ValueError, IndexError) as e:
                raise ValueError(f"Error parsing G-code on line {i + 1}:\n\n"
                                 f"Line content: '{line.strip()}'\n\n"
                                 f"Error: {e}")
        return np.array(points)

    def parse_and_draw_gcode(self):
        if self.gcode_plot:
            self.viewer.removeItem(self.gcode_plot)
        if self.progress_plot:
            self.viewer.removeItem(self.progress_plot)

        try:
            self.path_points = self._parse_gcode_to_points(self.gcode_lines)
        except ValueError as e:
            QMessageBox.critical(self, "G-code Parsing Error", str(e))
            self.gcode_lines = []
            self.path_points = np.array([])
            return
        
        if len(self.path_points) > 1:
            self.gcode_plot = gl.GLLinePlotItem(pos=self.path_points, color=(0.5, 0.5, 1, 1), width=2, antialias=True)
            self.viewer.addItem(self.gcode_plot)
        else:
            self.gcode_plot = None

    def update_progress_plot(self, sent_line_count):
        if self.progress_plot:
            self.viewer.removeItem(self.progress_plot)
        
        sent_lines = self.serial_worker.file_content[:sent_line_count]
        
        try:
            progress_points = self._parse_gcode_to_points(sent_lines)
            if len(progress_points) > 1:
                self.progress_plot = gl.GLLinePlotItem(pos=progress_points, color=(1, 0, 0, 1), width=3, antialias=True)
                self.viewer.addItem(self.progress_plot)
        except ValueError:
            # This should not happen if the initial parse was successful, but it's a safe fallback.
            pass

    def closeEvent(self, event):
        if self.serial_worker:
            self.toggle_connection()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())
