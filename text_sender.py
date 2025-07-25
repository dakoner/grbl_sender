import serial
import time
import argparse

def stream_gcode(port, baudrate, gcode_file):
    """
    Streams a G-code file to a serial port and waits for an 'ok' response.

    Args:
        port (str): The serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
        baudrate (int): The baud rate for the serial communication.
        gcode_file (str): The path to the G-code file to be sent.
    """
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"Opened serial port {port} at {baudrate} bps.")
            
            # Allow time for the device to initialize
            time.sleep(2)
            
            # Wake up the device
            ser.write(b"\r\n\r\n")
            time.sleep(2)   # Wait for grbl to initialize
            ser.flushInput()  # Flush startup text in serial input

            # Open the G-code file
            with open(gcode_file, 'r') as f:
                print(f"Sending G-code file: {gcode_file}")
                
                # Stream each line of the G-code file
                for line in f:
                    # Strip all comments and whitespace
                    line = line.strip()
                    
                    if line.startswith(';') or not line:
                        continue # Skip comments and empty lines

                    print(f"Sending: {line}")
                    ser.write((line + '\n').encode('utf-8'))
                    
                    # Wait for the 'ok' response from the device
                    response = ''
                    while 'ok' not in response:
                        response = ser.readline().decode('utf-8').strip()
                        if response:
                            print(f"Received: {response}")

            print("G-code streaming complete.")

    except serial.SerialException as e:
        print(f"Error opening or writing to serial port {port}: {e}")
    except FileNotFoundError:
        print(f"Error: G-code file not found at {gcode_file}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="Send a G-code file to a serial device.")
    parser.add_argument('gcode_file', help="The path to the G-code file.")
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help="The serial port to use. Defaults to /dev/ttyUSB0.")
    parser.add_argument('-b', '--baud', type=int, default=115200, help="The baud rate for the serial connection. Defaults to 115200.")
    
    args = parser.parse_args()

    # Call the main function with the parsed arguments
    stream_gcode(args.port, args.baud, args.gcode_file)
