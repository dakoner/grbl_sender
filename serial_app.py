import sys
import time
import threading
import re
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QFileDialog, QLineEdit
)
from PyQt6.QtCore import QObject, pyqtSignal, QThread, QTimer, Qt, pyqtSlot
from PyQt6.QtGui import QKeyEvent
from PyQt6 import uic
import serial
import serial.tools.list_ports

# --- Worker for Non-Blocking File Transfer ---
class FileSenderWorker(QObject):
    """
    Worker thread to handle sending a file in the background with a send-and-wait protocol.
    This prevents the GUI from freezing during the transfer.
    """
    progress_signal = pyqtSignal(int, int)  # current_line, total_lines
    finished_signal = pyqtSignal(str)      # success/failure message
    log_signal = pyqtSignal(str)           # for logging messages to the main console
    position_update_signal = pyqtSignal(str, str, str) # x, y, z as formatted strings

    def __init__(self, serial_port, file_path, ack_timeout=60):
        super().__init__()
        self.serial_port = serial_port
        self.file_path = file_path
        self.ack_timeout_seconds = ack_timeout
        self._is_running = True
        self.lines_to_send = []
        self.total_lines = 0
        self.waiting_for_ack = False
        self.ack_timer = None
        self.lock = threading.RLock() # Use a re-entrant lock

        # G-code parser state
        self.current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.distance_mode = 'G90' # Absolute is the default

    def run(self):
        """Main execution method for the worker."""
        try:
            with open(self.file_path, 'r') as f:
                self.lines_to_send = [line.strip() for line in f if line.strip() and not line.strip().startswith(';')]
            
            if not self.lines_to_send:
                self.finished_signal.emit("File is empty or contains only comments. Nothing to send.")
                return

            self.total_lines = len(self.lines_to_send)
            self.log_signal.emit(f"Starting file transfer of '{self.file_path}' ({self.total_lines} lines).")
            self.log_signal.emit(f"Using send-and-wait protocol. ACK Timeout: {self.ack_timeout_seconds}s")

            # Setup the acknowledgement timeout timer
            self.ack_timer = QTimer()
            self.ack_timer.setSingleShot(True)
            self.ack_timer.timeout.connect(self.timeout_occurred)
            
            # Start sending the first line
            self.send_next_line()

        except Exception as e:
            self.finished_signal.emit(f"Error starting file transfer: {e}")

    def _parse_gcode_line(self, line):
        """Parses a G-code line to update the virtual machine position."""
        # Check for distance mode changes
        if 'G90' in line:
            self.distance_mode = 'G90'
        if 'G91' in line:
            self.distance_mode = 'G91'

        # Check for G92 (Set Position)
        if 'G92' in line:
            x_match = re.search(r'X([-\d\.]+)', line)
            y_match = re.search(r'Y([-\d\.]+)', line)
            z_match = re.search(r'Z([-\d\.]+)', line)
            if x_match: self.current_pos['x'] = float(x_match.group(1))
            if y_match: self.current_pos['y'] = float(y_match.group(1))
            if z_match: self.current_pos['z'] = float(z_match.group(1))
            self.position_update_signal.emit(
                f"{self.current_pos['x']:.3f}",
                f"{self.current_pos['y']:.3f}",
                f"{self.current_pos['z']:.3f}"
            )
            return # G92 is not a motion command

        # Check for motion commands (G0, G1)
        if re.search(r'G[01]\b', line):
            x_match = re.search(r'X([-\d\.]+)', line)
            y_match = re.search(r'Y([-\d\.]+)', line)
            z_match = re.search(r'Z([-\d\.]+)', line)

            if not (x_match or y_match or z_match):
                return # No coordinates in this motion command

            if self.distance_mode == 'G90': # Absolute
                if x_match: self.current_pos['x'] = float(x_match.group(1))
                if y_match: self.current_pos['y'] = float(y_match.group(1))
                if z_match: self.current_pos['z'] = float(z_match.group(1))
            else: # Relative (G91)
                if x_match: self.current_pos['x'] += float(x_match.group(1))
                if y_match: self.current_pos['y'] += float(y_match.group(1))
                if z_match: self.current_pos['z'] += float(z_match.group(1))
            
            self.position_update_signal.emit(
                f"{self.current_pos['x']:.3f}",
                f"{self.current_pos['y']:.3f}",
                f"{self.current_pos['z']:.3f}"
            )

    def send_next_line(self):
        """Sends a single line if not waiting for an ACK."""
        with self.lock:
            if not self._is_running or self.waiting_for_ack:
                return

            if self.lines_to_send:
                line = self.lines_to_send.pop(0)
                try:
                    # Parse the line for position updates BEFORE sending
                    self._parse_gcode_line(line)
                    
                    self.serial_port.write((line + '\n').encode('utf-8'))
                    self.waiting_for_ack = True
                    self.log_signal.emit(f"--> SENT: {line}")
                    self.ack_timer.start(self.ack_timeout_seconds * 1000)
                    
                    lines_sent = self.total_lines - len(self.lines_to_send)
                    self.progress_signal.emit(lines_sent, self.total_lines)

                except Exception as e:
                    self.finished_signal.emit(f"Error sending line: {e}")
                    self.stop()
            else:
                # No more lines to send
                self.finished_signal.emit("File transfer completed successfully.")
                self.stop()

    @pyqtSlot()
    def receive_ack(self):
        """Called by a signal from the main thread when an 'ok' is received."""
        with self.lock:
            if self.waiting_for_ack:
                self.ack_timer.stop() # Stop the timeout for the acknowledged line
                self.waiting_for_ack = False
                self.log_signal.emit("<-- ACK Received (ok)")
                
                # Immediately try to send the next line
                self.send_next_line()

    def timeout_occurred(self):
        """Handles the ACK timeout."""
        with self.lock:
            if self.waiting_for_ack:
                self.finished_signal.emit(f"TIMEOUT: No 'ok' received for {self.ack_timeout_seconds} seconds. Transfer failed.")
                self.stop()

    def stop(self):
        """Stops the transfer and its timer. Should only be called from the worker thread."""
        with self.lock:
            self._is_running = False
            if self.ack_timer:
                self.ack_timer.stop()
    
    @pyqtSlot()
    def request_stop(self):
        """A slot that can be called from the main thread to safely stop the worker."""
        self.stop()
        self.finished_signal.emit("File transfer cancelled by user.")


# --- Custom QLineEdit with Command History ---
class CommandHistoryLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = []
        self.history_index = -1

    def add_to_history(self, command):
        """Adds a command to the history, avoiding duplicates."""
        if command in self.history:
            self.history.remove(command)
        self.history.insert(0, command)
        self.history_index = -1 # Reset index

    def keyPressEvent(self, event: QKeyEvent):
        """Handle up and down arrow keys for history navigation."""
        if event.key() == Qt.Key.Key_Up:
            if self.history_index < len(self.history) - 1:
                self.history_index += 1
                self.setText(self.history[self.history_index])
        elif event.key() == Qt.Key.Key_Down:
            if self.history_index > 0:
                self.history_index -= 1
                self.setText(self.history[self.history_index])
            else:
                self.history_index = -1
                self.clear()
        else:
            super().keyPressEvent(event)


# --- Main Application Window ---
class SerialConsoleApp(QMainWindow):
    ack_received_signal = pyqtSignal()
    request_stop_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        
        # Load the UI from the .ui file
        uic.loadUi('serial_console.ui', self)

        # --- Replace the placeholder QLineEdit with our custom one ---
        # This is a common pattern when using .ui files with custom widgets.
        # 1. Get the properties and layout of the placeholder
        original_command_input = self.command_input
        layout = original_command_input.parent().layout()
        
        # 2. Create an instance of our custom widget
        self.command_input = CommandHistoryLineEdit()
        
        # 3. Copy properties from the placeholder
        self.command_input.setObjectName(original_command_input.objectName())
        self.command_input.setPlaceholderText(original_command_input.placeholderText())
        
        # 4. Replace the placeholder in the layout
        layout.replaceWidget(original_command_input, self.command_input)
        original_command_input.deleteLater()


        # --- Serial Port State ---
        self.serial_port = serial.Serial()
        self.is_connected = False
        self.file_transfer_thread = None
        self.file_sender_worker = None
        self.is_polling_status = False
        self.waiting_for_status_response = False
        self.received_data_buffer = ""
        self.is_laser_on = False

        # --- Connect Signals to Slots ---
        self.connect_button.clicked.connect(self.toggle_connection)
        self.command_input.returnPressed.connect(self.send_command)
        self.hard_reset_button.clicked.connect(self.hard_reset)
        self.soft_reset_button.clicked.connect(self.soft_reset)
        self.abort_button.clicked.connect(self.send_abort)
        self.unlock_button.clicked.connect(self.send_unlock)
        self.laser_toggle_button.clicked.connect(self.toggle_laser)
        self.send_file_button.clicked.connect(self.send_file)
        self.cancel_file_button.clicked.connect(self.cancel_file_transfer)
        self.jog_up_button.clicked.connect(lambda: self.send_jog_command("Y"))
        self.jog_down_button.clicked.connect(lambda: self.send_jog_command("Y-"))
        self.jog_left_button.clicked.connect(lambda: self.send_jog_command("X-"))
        self.jog_right_button.clicked.connect(lambda: self.send_jog_command("X"))
        self.jog_z_up_button.clicked.connect(lambda: self.send_jog_command("Z"))
        self.jog_z_down_button.clicked.connect(lambda: self.send_jog_command("Z-"))

        # --- Initial UI State ---
        self.populate_ports()
        self.baud_selector.setCurrentText("115200")
        self.update_ui_state()

        # --- Timers ---
        self.status_poll_timer = QTimer(self)
        self.status_poll_timer.timeout.connect(self.poll_status)
        
        # This timer will process serial data received by the pyserial thread
        self.serial_read_timer = QTimer(self)
        self.serial_read_timer.timeout.connect(self.read_serial_data)
        self.serial_read_timer.start(20) # Check for new data every 20ms

    def populate_ports(self):
        """Fills the port selector with available serial ports."""
        self.port_selector.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_selector.addItem(port.device)
        if sys.platform == "win32" and "COM3" in [p.device for p in ports]:
            self.port_selector.setCurrentText("COM3")

    def update_ui_state(self):
        """Enables/disables UI elements based on connection state."""
        is_transferring = self.file_transfer_thread is not None and self.file_transfer_thread.isRunning()
        
        self.port_selector.setEnabled(not self.is_connected)
        self.baud_selector.setEnabled(not self.is_connected)
        
        self.command_input.setEnabled(self.is_connected and not is_transferring)
        self.hard_reset_button.setEnabled(self.is_connected and not is_transferring)
        self.soft_reset_button.setEnabled(self.is_connected and not is_transferring)
        self.abort_button.setEnabled(self.is_connected and not is_transferring)
        self.unlock_button.setEnabled(self.is_connected and not is_transferring)
        self.laser_toggle_button.setEnabled(self.is_connected and not is_transferring)
        self.laser_power_input.setEnabled(self.is_connected and not is_transferring)
        self.send_file_button.setEnabled(self.is_connected and not is_transferring)
        self.cancel_file_button.setEnabled(self.is_connected and is_transferring)
        
        # Jogging controls
        self.jog_up_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_down_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_left_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_right_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_z_up_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_z_down_button.setEnabled(self.is_connected and not is_transferring)
        self.jog_xy_dist_input.setEnabled(self.is_connected and not is_transferring)
        self.jog_z_dist_input.setEnabled(self.is_connected and not is_transferring)
        self.jog_feed_rate_input.setEnabled(self.is_connected and not is_transferring)

    def toggle_connection(self):
        """Connects or disconnects the serial port."""
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()
        self.update_ui_state()

    def connect_serial(self):
        """Establishes the serial connection."""
        port = self.port_selector.currentText()
        baud = int(self.baud_selector.currentText())
        if not port:
            self.log_to_console("Error: No serial port selected.")
            return

        try:
            self.serial_port.port = port
            self.serial_port.baudrate = baud
            self.serial_port.timeout = 0.1 # Non-blocking read
            self.serial_port.open()
            self.is_connected = True
            self.connect_button.setText("Disconnect")
            self.log_to_console(f"Connected to {port} at {baud} baud.")
            self.status_poll_timer.start(1000) # Start polling every second
        except serial.SerialException as e:
            self.log_to_console(f"Error connecting: {e}")
            self.is_connected = False

    def disconnect_serial(self):
        """Closes the serial connection."""
        # Safety feature: turn off laser on disconnect
        if self.is_laser_on:
            self.toggle_laser()

        if self.file_transfer_thread and self.file_transfer_thread.isRunning():
            self.cancel_file_transfer()
        
        self.status_poll_timer.stop()
        if self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
        self.connect_button.setText("Connect")
        self.log_to_console("Disconnected.")
        self.clear_status_display()

    def read_serial_data(self):
        """Reads data from serial and processes it."""
        if self.serial_port and self.serial_port.is_open:
            try:
                # Read all available bytes
                data_bytes = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not data_bytes:
                    return

                # Decode and add to buffer
                self.received_data_buffer += data_bytes.decode('utf-8', errors='replace')
                
                # Process complete lines from the buffer
                while '\n' in self.received_data_buffer:
                    line, self.received_data_buffer = self.received_data_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.process_received_line(line)

            except serial.SerialException as e:
                self.log_to_console(f"Serial error: {e}")
                self.disconnect_serial()
                self.update_ui_state()

    def process_received_line(self, line):
        """Handles a single, complete line of received data."""
        is_transferring = self.file_transfer_thread is not None and self.file_transfer_thread.isRunning()
        if is_transferring and line.lower() == 'ok':
            if self.file_sender_worker:
                self.ack_received_signal.emit()
            return # Don't print "ok" to console during transfer

        # Check if this is a status response
        if self.waiting_for_status_response and line.startswith('<') and line.endswith('>'):
            self.parse_and_display_status(line)
            self.waiting_for_status_response = False
            # Don't log status responses to the main console to keep it clean
            return

        # Otherwise, just log to the main console
        self.log_to_console(f"<<< {line}")

    def parse_and_display_status(self, line):
        """Parses a FluidNC status string and updates the UI."""
        # Don't update from machine status if we are sending a file
        is_transferring = self.file_transfer_thread is not None and self.file_transfer_thread.isRunning()
        if is_transferring:
            return

        try:
            # Default values
            state = "Unknown"
            x, y, z = "---", "---", "---"

            # Extract state
            state_match = re.match(r"<(\w+)", line)
            if state_match:
                state = state_match.group(1)

            # Extract MPos
            mpos_match = re.search(r"MPos:([-\d\.]+),([-\d\.]+),([-\d\.]+)", line)
            if mpos_match:
                x = mpos_match.group(1)
                y = mpos_match.group(2)
                z = mpos_match.group(3)

            self.state_display.setText(state)
            self.x_pos_display.setText(x)
            self.y_pos_display.setText(y)
            self.z_pos_display.setText(z)

        except Exception as e:
            # If parsing fails, show the raw line in the state field
            self.state_display.setText(line)
            self.x_pos_display.setText("Error")
            self.y_pos_display.setText("Error")
            self.z_pos_display.setText("Error")
            self.log_to_console(f"Status parsing error: {e}")
            
    def clear_status_display(self):
        """Clears the structured status display fields."""
        self.state_display.clear()
        self.x_pos_display.clear()
        self.y_pos_display.clear()
        self.z_pos_display.clear()

    def log_to_console(self, message):
        """Appends a message to the console output and scrolls to the bottom."""
        self.console_output.append(message)
        self.console_output.verticalScrollBar().setValue(self.console_output.verticalScrollBar().maximum())

    def send_command(self):
        """Sends a command from the input box."""
        if self.is_connected:
            command = self.command_input.text()
            if command:
                try:
                    self.serial_port.write((command + '\n').encode('utf-8'))
                    self.log_to_console(f">>> {command}")
                    self.command_input.add_to_history(command)
                    self.command_input.clear()
                except serial.SerialException as e:
                    self.log_to_console(f"Error writing to port: {e}")

    # --- Control Button Actions ---
    def hard_reset(self):
        if self.is_connected:
            try:
                self.log_to_console("Performing hard reset (DTR toggle)...")
                self.serial_port.setDTR(False)
                time.sleep(0.5)
                self.serial_port.setDTR(True)
                self.log_to_console("Hard reset complete.")
            except serial.SerialException as e:
                self.log_to_console(f"DTR toggle failed: {e}")

    def soft_reset(self):
        if self.is_connected:
            self.log_to_console("Sending soft reset (Ctrl+X)...")
            self.serial_port.write(b'\x18')

    def send_abort(self):
        if self.is_connected:
            self.log_to_console("Sending abort (0x85)...")
            self.serial_port.write(b'\x85')
            
    def send_unlock(self):
        if self.is_connected:
            self.log_to_console("Sending unlock ($X)...")
            self.serial_port.write(b'$X\n')
            
    def toggle_laser(self):
        """Toggles the laser on or off."""
        if not self.is_connected:
            return
            
        if self.is_laser_on:
            # Turn laser off
            command = "M5"
            self.serial_port.write(command.encode('utf-8') + b'\n')
            self.log_to_console(f">>> {command} (Laser Off)")
            self.laser_toggle_button.setText("Laser On")
            self.is_laser_on = False
        else:
            # Turn laser on
            power = self.laser_power_input.value()
            command = f"M3 S{power}"
            self.serial_port.write(command.encode('utf-8') + b'\n')
            self.log_to_console(f">>> {command} (Laser On)")
            self.laser_toggle_button.setText("Laser Off")
            self.is_laser_on = True


    def poll_status(self):
        """Sends '?' to poll device status."""
        if self.is_connected and not self.waiting_for_status_response:
            try:
                self.serial_port.write(b'?')
                self.waiting_for_status_response = True
            except serial.SerialException as e:
                self.log_to_console(f"Status poll failed: {e}")

    # --- File Transfer Actions ---
    def send_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select File to Send")
        if not file_path:
            return

        self.file_transfer_thread = QThread()
        self.file_sender_worker = FileSenderWorker(
            self.serial_port,
            file_path
        )
        self.file_sender_worker.moveToThread(self.file_transfer_thread)

        # Connect signals
        self.file_sender_worker.progress_signal.connect(self.update_file_progress)
        self.file_sender_worker.finished_signal.connect(self.file_transfer_finished)
        self.file_sender_worker.log_signal.connect(self.log_to_console)
        self.file_sender_worker.position_update_signal.connect(self.update_display_from_gcode)
        self.file_transfer_thread.started.connect(self.file_sender_worker.run)
        self.ack_received_signal.connect(self.file_sender_worker.receive_ack)
        self.request_stop_signal.connect(self.file_sender_worker.request_stop)

        self.file_transfer_thread.start()
        self.status_poll_timer.stop() # Pause status polling during transfer
        self.update_ui_state()
        
    @pyqtSlot(str, str, str)
    def update_display_from_gcode(self, x, y, z):
        """Updates the coordinate display based on the G-code parser."""
        self.state_display.setText("Sending")
        self.x_pos_display.setText(x)
        self.y_pos_display.setText(y)
        self.z_pos_display.setText(z)

    def update_file_progress(self, sent, total):
        self.file_progress_label.setText(f"Sending: {sent} / {total} lines")

    def file_transfer_finished(self, message):
        self.log_to_console(message)
        self.file_progress_label.setText("File transfer idle.")
        
        if self.file_transfer_thread:
            # Disconnect the signals to avoid issues on the next transfer
            try:
                self.ack_received_signal.disconnect(self.file_sender_worker.receive_ack)
                self.file_sender_worker.position_update_signal.disconnect(self.update_display_from_gcode)
                self.request_stop_signal.disconnect(self.file_sender_worker.request_stop)
            except TypeError:
                pass # Signal was already disconnected
            self.file_transfer_thread.quit()
            self.file_transfer_thread.wait()
        
        self.file_transfer_thread = None
        self.file_sender_worker = None
        
        if self.is_connected:
            self.status_poll_timer.start() # Resume polling
        self.update_ui_state()

    def cancel_file_transfer(self):
        if self.file_sender_worker:
            self.request_stop_signal.emit()

    # --- Jogging Actions ---
    def send_jog_command(self, axis_dir):
        if not self.is_connected:
            return
        
        axis = axis_dir[0]
        
        try:
            if axis in ('X', 'Y'):
                dist = float(self.jog_xy_dist_input.text())
            else: # Z axis
                dist = float(self.jog_z_dist_input.text())
            
            feed_rate = int(self.jog_feed_rate_input.text())
        except ValueError:
            self.log_to_console("Invalid jog distance or feed rate.")
            return

        if len(axis_dir) > 1 and axis_dir[1] == '-':
            dist = -dist
        
        # Use Grbl jogging syntax: $J=G91 X... F...
        command = f"$J=G91 {axis}{dist:.4f} F{feed_rate}"
        try:
            self.serial_port.write((command + '\n').encode('utf-8'))
            self.log_to_console(f">>> JOG: {command}")
        except serial.SerialException as e:
            self.log_to_console(f"Error sending jog command: {e}")

    def closeEvent(self, event):
        """Ensure clean shutdown."""
        self.disconnect_serial()
        event.accept()


# --- Application Entry Point ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SerialConsoleApp()
    window.show()
    sys.exit(app.exec())
