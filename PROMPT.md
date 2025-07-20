# Final Prompt: PyQt6 CNC Serial Console Application

Write a robust, thread-safe PyQt6 application that implements a graphical serial console for controlling CNC machines like Grbl or FluidNC. The application's user interface should be defined in a Qt Designer `.ui` file, which is then loaded by the main Python script.

The application must have the following features:

---

### **Main Interface**

- A main console output window that shows all data received from the serial port. This window must automatically scroll to the bottom when new lines appear.
- A command input box where a user can type commands.
  - Pressing Enter sends the command, followed by a newline character, to the serial port.
  - The input box must have a command history, allowing the user to cycle through previously sent commands using the up and down arrow keys.

---

### **Connection Management**

- A dropdown menu to select the serial port from a list of available ports on the system. It should default to `COM3` on Windows.
- A dropdown menu to select the baud rate, including common values like `9600`, `57600`, and `115200`. The default should be `115200`.
- A "Connect"/"Disconnect" button to manage the serial connection.

---

### **File Transfer**

- A "Send File" button that opens a file dialog.
- The file transfer must be handled in a non-blocking background thread to keep the UI responsive and must be thread-safe to prevent the application from hanging, crashing, or producing threading errors on cancellation.
- Implement a **send-and-wait protocol** where only one G-code command is in flight at a time. The application sends one line and waits for an "ok" string (case-insensitive) as an acknowledgement before sending the next.
- The file parser should ignore empty lines and lines that start with a semicolon (`;`), as these are comments in G-code.
- There should be a configurable timeout for acknowledgements, set to 60 seconds. If the timeout occurs while waiting for an "ok", the file transfer should fail gracefully.
- A "Cancel" button must be visible and functional during file transfer to abort the process safely.
- During the file transfer, the application must parse the outgoing G-code to provide a real-time display of the machine's target coordinates.
  - It should track the current position based on `G0` and `G1` motion commands.
  - It must correctly interpret `G90` (absolute) and `G91` (relative) distance modes.
  - It must handle `G92` (Set Position) commands to update its internal coordinate system.
  - The calculated X, Y, and Z coordinates should be displayed in their dedicated text boxes.

---

### **Device Control & Commands**

- A "Hard Reset" button that toggles the DTR line off, waits for 0.5 seconds, and then toggles it back on.
- A "Soft Reset" button that sends a Control-X (`0x18`) byte to the serial port.
- An "Abort" button that sends a `0x85` byte to the serial port.
- An "Unlock" button that sends a `$X` command to the serial port.
- A "Home" button that sends a `$H` command to the serial port.

---

### **Laser Control**

- A dedicated "Laser Control" section.
- A power input field (e.g., a `QSpinBox`) that lets the user enter an integer value from 0 to 1023.
- A toggle button that changes its text between "Laser On" and "Laser Off".
  - When clicked to turn the laser on, it sends an `M3 S<power>` command, where `<power>` is the value from the power input field.
  - When clicked to turn the laser off, it sends an `M5` command.
  - As a safety feature, an `M5` command should be automatically sent if the application disconnects from the serial port while the laser is on.

---

### **Status & Jogging**

- A structured, read-only display for the machine's status, with separate text boxes for "State," "X," "Y," and "Z" coordinates.
- The application should periodically send a `?` character (without a newline) to the serial port every second to poll for status.
- The application should parse Grbl/FluidNC-style status responses and update the structured display fields accordingly. The parser's regular expression must be flexible enough to handle both `MPos:` and `WPos:` for the coordinate data (e.g., `<Idle|MPos:10,20,5...>` and `<Idle|WPos:13,3,0...>`). This polling should be paused during file transfers.
- A "Jog" widget with the following controls:
  - Four arrow buttons for X and Y movement (`Y+`, `Y-`, `X-`, `X+`).
  - Two buttons for Z movement ("Up" and "Down").
  - Clicking a jog button should send a Grbl-style jog command (`$J=G91 <axis><distance> F<feed>`). Note the absence of `G0`.
  - A text input for "XY Dist" to set the jog distance for X and Y movements.
  - A separate text input for "Z Dist" to set the jog distance for Z movements.
  - A text input for "Feed" to set the integer feed rate for all jog moves.