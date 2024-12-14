import sys
import os
import argparse
import json

from STservo_sdk import *  # Import STServo SDK library

class STServoController:
    def __init__(self, device_name='/dev/ttyACM0', baudrate=1000000):
        """
        Initializes the STServo controller with the specified port and baudrate.
        """
        self.device_name = device_name
        self.baudrate = baudrate
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = sts(self.port_handler)

    def open_port(self):
        """
        Opens the serial port and sets the baudrate.
        Returns True if successful, otherwise False.
        """
        if not self.port_handler.openPort():
            print("Failed to open the port")
            return False

        if not self.port_handler.setBaudRate(self.baudrate):
            print("Failed to set the baudrate")
            return False

        print(f"Port {self.device_name} opened with baudrate {self.baudrate}")
        return True

    def close_port(self):
        """
        Closes the serial port.
        """
        self.port_handler.closePort()
        print("Port closed.")

    def read_position_and_speed(self, servo_id):
        """
        Reads the present position and speed of the servo with the given ID.
        Returns a tuple: (position, speed) or None if communication fails.
        """
        position, speed, comm_result, error = self.packet_handler.ReadPosSpeed(servo_id)

        if comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(comm_result))
            return None
        if error != 0:
            print(self.packet_handler.getRxPacketError(error))
            return None

        return position, speed

    def ping(self, servo_id):
        """
        Pings the servo with the given ID.
        Returns True if successful, otherwise False.
        """
        model_number, comm_result, error = self.packet_handler.ping(servo_id)

        if comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(comm_result))
            return False

        if error != 0:
            print(self.packet_handler.getRxPacketError(error))
            return False

        print(f"Ping successful: [ID:{servo_id}] Model Number: {model_number}")
        return True

    def write_position_ex(self, servo_id, position, speed, acceleration):
        """
        Write a goal position, moving speed, and acceleration to a servo.

        Args:
            servo_id (int): The ID of the servo to control.
            position (int): The goal position to set (e.g., 0 to 4095).
            speed (int): The moving speed (e.g., 0 to 2400).
            acceleration (int): The moving acceleration (e.g., 0 to 50).

        Returns:
            bool: True if the command was successful, False otherwise.
        """
        comm_result, error = self.packet_handler.WritePosEx(
            servo_id, position, speed, acceleration
        )

        if comm_result != COMM_SUCCESS:
            print(f"Failed to write position to Servo {servo_id}: {self.packet_handler.getTxRxResult(comm_result)}")
            return False
        if error != 0:
            print(f"Error occurred while writing to Servo {servo_id}: {self.packet_handler.getRxPacketError(error)}")
            return False

        print(f"Successfully wrote Position: {position}, Speed: {speed}, Acceleration: {acceleration} to Servo {servo_id}.")
        return True

    def wait_for_keypress(self):
        """
        Waits for a keypress. Returns the key pressed.
        """
        if os.name == 'nt':
            import msvcrt
            return msvcrt.getch().decode()
        else:
            import tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                return sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def load_positions(file_path):
    """
    Load and validate all saved positions from a JSON file.

    Args:
        file_path (str): Path to the JSON file.

    Returns:
        dict: A dictionary containing configurations with integer keys and values.
    """
    try:
        # Load the JSON file
        with open(file_path, "r") as file:
            configurations = json.load(file)
        
        # Validate and ensure keys/values are integers
        positions = {
            key: {int(k): int(v) for k, v in value.items()}
            for key, value in configurations.items()
        }
        return positions
    except FileNotFoundError:
        print("Error: 'config.json' file not found.")
        raise
    except json.JSONDecodeError:
        print("Error: Failed to parse 'config.json'. Please ensure it contains valid JSON.")
        raise
    except ValueError as e:
        print(f"Error: {e}")
        raise





# Main function for argument parsing and running the controller
def main():
    parser = argparse.ArgumentParser(description="Read position and speed of STServos 1-6.")
    parser.add_argument("--port", type=str, default='/dev/ttyACM0', help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Baud rate (default: 1000000)")
    args = parser.parse_args()
    
    # Load and validate 'rest' positions from the JSON file
    
    try:
        positions = load_positions("saved_positions.json")
    except Exception as e:
        print(f"Failed to load configurations: {e}")
        return

    # Initialize the STServo controller
    controller = STServoController(device_name=args.port, baudrate=args.baudrate)

    # Open the port
    if not controller.open_port():
        print("Failed to initialize the port.")
        return

    # Main loop
    try:
        while True:
            # Display menu
            print("\nOptions:")
            menu_message = f"Options: R - Read | " + " | ".join(
                [f"{idx} - {config_name}" for idx, config_name in enumerate(positions.keys(), start=1)]
                ) + " | ESC - Exit"
            print(menu_message)

            # Get user input
            choice = controller.wait_for_keypress().upper()

            if choice == chr(0x1B):  # ESC key
                print("Exiting program.")
                break
            elif choice == "R":
                # Read current position and speed
                print("Reading current position and speed:")
                for servo_id in range(1, 7):
                    result = controller.read_position_and_speed(servo_id)
                    if result:
                        position, speed = result
                        print(f"[ID:{servo_id:03d}] Position: {position}, Speed: {speed}")
                    else:
                        print(f"Failed to read Servo ID {servo_id}.")
            elif choice.isdigit() and 1 <= int(choice) <= len(positions):
                # Write a selected configuration
                config_name = list(positions.keys())[int(choice) - 1]
                print(f"Writing '{config_name}' position to servos:")
                for servo_id, position in positions[config_name].items():
                    success = controller.write_position_ex(
                        servo_id=servo_id,
                        position=position,
                        speed=1000,  # Example speed
                        acceleration=40  # Example acceleration
                    )
                    if not success:
                        print(f"Failed to move Servo {servo_id} to Position {position}")
            else:
                print("Invalid choice. Please select a valid option.")
    finally:
        # Close the port on exit
        controller.close_port()
        print("Port closed.")

if __name__ == "__main__":
    main()

