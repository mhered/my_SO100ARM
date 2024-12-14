import serial
import time

# Serial communication parameters
SERIAL_PORT = "/dev/ttyACM0"  # Replace with your serial port
BAUD_RATE = 1000000  # Default baud rate for Waveshare servo board
TIMEOUT = 0.1  # Communication timeout in seconds

# Function to calculate checksum (last byte of command)
def calculate_checksum(data):
    return (~sum(data) & 0xFF)

# Function to send a command and receive a response
def send_command(ser, command):
    try:
        # Write the command to the serial port
        ser.write(command)
        time.sleep(0.05)  # Small delay to wait for response
        # Read the response
        response = ser.read_all()
        return response
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

# Function to read configuration of a servo
def read_servo_config(ser, servo_id):
    # Construct the command to read configuration
    # Assuming a standard protocol for STS3215 and Waveshare (adjust based on datasheet)
    command = [servo_id, 0x03, 0x00, 0x00, 0x00]  # Example Read Command
    checksum = calculate_checksum(command)
    command.append(checksum)
    command_bytes = bytes(command)
    
    # Send command and read response
    response = send_command(ser, command_bytes)
    if response:
        print(f"Servo {servo_id} Response: {response.hex()}")
    else:
        print(f"No response from Servo {servo_id}")

# Main function
def main():
    # Open serial port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    # Read configuration for a range of servo IDs (1 to N)
    for servo_id in range(1, 7):  # Adjust range as per your setup
        print(f"Reading configuration for Servo ID {servo_id}...")
        read_servo_config(ser, servo_id)

    # Close serial port
    ser.close()
    print("Serial port closed.")

if __name__ == "__main__":
    main()
