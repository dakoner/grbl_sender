import sys
import platform
from PyQt6.QtWidgets import QApplication, QMainWindow, QTextEdit, QLineEdit, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QLabel, QComboBox, QFileDialog
from PyQt6.QtCore import QThread, pyqtSignal, QObject, QTimer
import serial
import serial.tools.list_ports

class SerialWorker(QObject):
    """
    Worker thread for handling serial port reading.
    This is important to prevent the GUI from freezing while waiting for data.
    """
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._is_running = True

    def run(self):
        """
        Reads data from the serial port and emits it.
        """
        while self._is_running and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # Read data and decode it, replacing errors
                    line = self.ser.readline().decode('utf-8', 'replace').strip()
                    if line:
                        self.data_received.emit(line)
            except serial.SerialException as e:
                self.error_occurred.emit(f"Serial Error: {e}")
                self._is_running = False
        self.finished.emit()

    def stop(self):
        """
        Stops the reading loop.
        """
        self._is_running = False


class SerialConsole(QMainWindow):
    """
    Main application window for the serial console.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 Serial Console")
        self.setGeometry(100, 100, 800, 500) # Increased window size

        # --- UI Elements ---
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setStyleSheet("background-color: #2E2E2E; color: #D2D2D2; font-family: 'Courier New';")

        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command and press Enter...")
        self.command_input.returnPressed.connect(self.send_command)
        self.command_input.setEnabled(False)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        self.send_file_button = QPushButton("Send File")
        self.send_file_button.clicked.connect(self.select_and_send_file)
        self.send_file_button.setEnabled(False)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel_file_send)
        self.cancel_button.setVisible(False)

        self.port_selector = QComboBox()
        self.populate_serial_ports()

        self.baud_selector = QComboBox()
        self.baud_selector.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600'])
        self.baud_selector.setCurrentText('115200')

        # --- Layout ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        connection_layout = QHBoxLayout()
        connection_layout.addWidget(QLabel("Serial Port:"))
        connection_layout.addWidget(self.port_selector)
        connection_layout.addWidget(QLabel("Baud Rate:"))
        connection_layout.addWidget(self.baud_selector)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.send_file_button)
        connection_layout.addWidget(self.cancel_button)

        main_layout.addLayout(connection_layout)
        main_layout.addWidget(self.console_output)
        main_layout.addWidget(self.command_input)

        # --- Serial Port Handling ---
        self.serial_port = None
        self.serial_thread = None
        self.worker = None

        # --- File Sending State ---
        self.is_sending_file = False
        self.file_lines = []
        self.current_line_index = 0
        self.ack_timer = QTimer(self)
        self.ack_timer.setSingleShot(True)
        self.ack_timer.timeout.connect(self.handle_ack_timeout)

    def populate_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_selector.clear()
        port_names = [port.device for port in ports]
        self.port_selector.addItems(port_names)
        if platform.system() == "Windows":
            if "COM3" in port_names:
                self.port_selector.setCurrentText("COM3")
            elif port_names:
                self.port_selector.setCurrentIndex(0)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_from_port()
        else:
            self.connect_to_port()

    def connect_to_port(self):
        port_name = self.port_selector.currentText()
        baud_rate_str = self.baud_selector.currentText()
        if not port_name:
            self.log_to_console("No serial port selected.")
            return
        try:
            baud_rate = int(baud_rate_str)
        except ValueError:
            self.log_to_console(f"Invalid baud rate: {baud_rate_str}")
            return
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=1)
            self.log_to_console(f"Connected to {port_name} at {baud_rate} baud")
            self.connect_button.setText("Disconnect")
            self.command_input.setEnabled(True)
            self.send_file_button.setEnabled(True)
            self.port_selector.setEnabled(False)
            self.baud_selector.setEnabled(False)
            self.worker = SerialWorker(self.serial_port)
            self.serial_thread = QThread()
            self.worker.moveToThread(self.serial_thread)
            self.serial_thread.started.connect(self.worker.run)
            self.worker.finished.connect(self.serial_thread.quit)
            self.worker.finished.connect(self.worker.deleteLater)
            self.serial_thread.finished.connect(self.serial_thread.deleteLater)
            self.worker.data_received.connect(self.handle_received_data)
            self.worker.error_occurred.connect(self.handle_serial_error)
            self.serial_thread.start()
        except serial.SerialException as e:
            self.log_to_console(f"Error connecting to {port_name}: {e}")
            self.command_input.setEnabled(False)
            self.send_file_button.setEnabled(False)

    def disconnect_from_port(self):
        if self.is_sending_file:
            self.cancel_file_send()
        if self.worker:
            self.worker.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_to_console(f"Disconnected from {self.serial_port.name}")
        self.connect_button.setText("Connect")
        self.command_input.setEnabled(False)
        self.send_file_button.setEnabled(False)
        self.port_selector.setEnabled(True)
        self.baud_selector.setEnabled(True)
        self.serial_port = None

    def send_command(self):
        if self.serial_port and self.serial_port.is_open and not self.is_sending_file:
            command = self.command_input.text()
            self.log_to_console(f"> {command}")
            self.serial_port.write((command + '\n').encode('utf-8'))
            self.command_input.clear()

    def log_to_console(self, text):
        self.console_output.append(text)

    def handle_received_data(self, text):
        """Handles incoming data and the file sending state machine."""
        self.log_to_console(text)
        if self.is_sending_file and text.strip().lower() == "ok":
            self.ack_timer.stop()
            self.current_line_index += 1
            self.send_next_line()

    def select_and_send_file(self):
        """Opens a file dialog and starts the file sending process."""
        file_path, _ = QFileDialog.getOpenFileName(self, "Select File to Send")
        if not file_path:
            return
        try:
            with open(file_path, 'r') as f:
                self.file_lines = f.readlines()
            if not self.file_lines:
                self.log_to_console("File is empty.")
                return

            self.is_sending_file = True
            self.current_line_index = 0
            self.log_to_console(f"--- Starting to send file: {file_path} ---")
            self.update_ui_for_file_send(True)
            self.send_next_line()
        except Exception as e:
            self.log_to_console(f"Error reading file: {e}")

    def send_next_line(self):
        """Sends the next line from the file or finishes the process."""
        if self.current_line_index < len(self.file_lines):
            line = self.file_lines[self.current_line_index].strip()
            if line: # Only send non-empty lines
                self.log_to_console(f"> [FILE] {line}")
                self.serial_port.write((line + '\n').encode('utf-8'))
                self.ack_timer.start(5000) # 5-second timeout for "ok"
            else: # If line is empty, skip it and send the next one
                self.current_line_index += 1
                self.send_next_line()
        else:
            self.finish_file_send(success=True)

    def finish_file_send(self, success, message=""):
        """Resets the state after file sending is complete or cancelled."""
        if not self.is_sending_file: return
        self.is_sending_file = False
        self.ack_timer.stop()
        if success:
            self.log_to_console("--- File sent successfully ---")
        else:
            self.log_to_console(f"--- File sending failed: {message} ---")
        self.update_ui_for_file_send(False)

    def cancel_file_send(self):
        self.finish_file_send(success=False, message="User cancelled")

    def handle_ack_timeout(self):
        self.finish_file_send(success=False, message="Acknowledgement timeout")

    def update_ui_for_file_send(self, is_sending):
        """Enables/disables UI controls based on file sending state."""
        self.is_sending_file = is_sending
        self.cancel_button.setVisible(is_sending)
        self.command_input.setEnabled(not is_sending)
        self.connect_button.setEnabled(not is_sending)
        self.send_file_button.setEnabled(not is_sending)
        self.port_selector.setEnabled(not is_sending)
        self.baud_selector.setEnabled(not is_sending)

    def handle_serial_error(self, error_message):
        self.log_to_console(f"ERROR: {error_message}")
        self.disconnect_from_port()

    def closeEvent(self, event):
        self.disconnect_from_port()
        event.accept()

if __name__ == '__main__':
    try:
        import serial
    except ImportError:
        print("pyserial is not installed. Please install it using: pip install pyserial")
        sys.exit(1)
    app = QApplication(sys.argv)
    console = SerialConsole()
    console.show()
    sys.exit(app.exec())
