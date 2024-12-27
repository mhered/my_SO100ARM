import sys
import os
import argparse
import json

from STservo_sdk import *  # Import STServo SDK library

class RobotArmController:
    _DEFAULT_PORT = '/dev/ttyACM0'
    _DEFAULT_BAUDRATE = 1000000
    _DEFAULT_CONFIGFILE = "saved_positions.json"

    def __init__(self, port=_DEFAULT_PORT, baudrate=_DEFAULT_BAUDRATE):
        """
        Initializes the STServo controller with the specified port and baudrate.
        """
        self.port = port
        self.baudrate = baudrate

        self.port_handler = PortHandler(self.port)
        self.packet_handler = sts(self.port_handler)

        if not self.open_port():
            raise ValueError(f"Failed to open the port {self.port} with baudrate {self.baudrate}")

        self.servo_ids = []
        self.named_positions = {}
        self.current_arm_position = {}

    def open_port(self):
        """
        Opens the serial port and sets the baudrate.
        Returns True if successful, otherwise False.
        """
        try:
            port_opened = self.port_handler.openPort()
        except serial.SerialException as e:
            print(f"Failed to open the port: {e}")
            return False

        if not port_opened:
            print("Failed to open the port")
            return False

        if not self.port_handler.setBaudRate(self.baudrate):
            print("Failed to set the baudrate")
            return False

        print(f"Port {self.port} opened with baudrate {self.baudrate}")
        return True

    def close_port(self):
        """
        Closes the serial port.
        """
        self.port_handler.closePort()
        print(f"Port {self.port} closed.")

    def read_position_and_speed_by_id(self, servo_id):
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

    def ping(self, servo_id, verbose = True):
        """
        Pings a servo by ID.
        Returns True if successful, otherwise False.
        """
        if verbose:
            print ("Trying to ping servo with ID: ", servo_id)

        model_number, comm_result, error = self.packet_handler.ping(servo_id)

        if comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(comm_result))
            return False

        if error != 0:
            print(self.packet_handler.getRxPacketError(error))
            return False
             
        if verbose: 
            print(f"Ping successful: [ID:{servo_id}] Model Number: {model_number}")
        return True

    def write_position_by_id(self, servo_id, position, speed, acceleration):
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
    


    def validate(self, positions):
        """
        Validate a dictionary of servo positions.

        Validation steps:
        1. Take the list of servo IDs from the first configuration as the reference.
        2. Ensure all configurations have the exact same list of servo IDs.
        3. Validate that all positions are integers within the range of 0 to 4095.
        4. Ping all servos in the reference list to validate connectivity.

        Args:
            positions (dict): Dictionary of configurations to validate.

        Returns:
            bool: True if all validations pass, False otherwise.
        """

        # Get the first configuration's IDs as reference
        config_names = list(positions.keys())
        if not config_names:
            print("Error: No configurations provided for validation.")
            return False

        reference_name = config_names[0]
        reference_ids = list(positions[reference_name].keys())

        # Validate all configurations
        for config_name, config_positions in positions.items():
            if config_name != reference_name:
                # Check if IDs match the reference
                current_ids = list(config_positions.keys())
                if set(current_ids) != set(reference_ids):
                    print(f"Error: Servo IDs in '{config_name}' do not match reference '{reference_name}'.")
                    print(f"Servo IDs in '{config_name}': {current_ids} vs")
                    print(f"Servo IDs in '{reference_name}': {reference_ids}")   
                    return False

            # Validate positions in the current configuration
            for servo_id, position in config_positions.items():
                if not isinstance(position, int) or not 0 <= position <= 4095:
                    print(f"Position {position} for Servo {servo_id} in '{config_name}' is out of range or invalid.")
                    return False

        # Validate reference IDs by pinging each servo
        for servo_id in reference_ids:
            if not self.ping(int(servo_id), verbose=False):
                print(f"Failed to ping Servo {servo_id} in '{reference_name}'.")
                return False

        print("Validation successful: All configurations are valid.")
        return True
   

    def read_current_arm_position(self, verbose = True):
        """
        Read the current position and speed of all servos in the robot arm.
        """

        success = True

        if verbose:
            print("Reading current position and speed:")

        # Dictionary to store the servo positions
        servo_positions_speeds = {}

        for servo_id in self.servo_ids:
            result = self.read_position_and_speed_by_id(servo_id)
            if result:
                position, speed = result
                servo_positions_speeds[servo_id] = {'position':position, 'speed' : speed} # Save the position and speed to the json_output dictionary
            else:
                print(f"Failed to read Servo ID {servo_id}.")
                success = False
        

        if success:
            current_arm_position = {key : value['position'] for key, value in servo_positions_speeds.items()}
            self.current_arm_position = current_arm_position
            if verbose: 
                for servo_id in self.servo_ids:
                    position = servo_positions_speeds[servo_id]['position']
                    speed = servo_positions_speeds[servo_id]['speed']
                    print(f"[ID:{servo_id:03d}] Position: {position}, Speed: {speed}")

                # Print the JSON snippet, formatted for readability         
                print("\nJSON snippet ready to copy and paste into your file:")
                print(json.dumps({"current_position" : current_arm_position}, indent=2))
            

        
        return success
    
    def write_arm_position(self, target_arm_position, verbose = True, max_speed=1000, max_acceleration=40):
        # Read current position and speed
        if self.read_current_arm_position(verbose=False):

            distances = {}
            # calculate distances for each servo
            for servo_id in self.servo_ids:
                distances[servo_id] = abs(target_arm_position[servo_id] - self.current_arm_position[servo_id])
                
            # calculate speed for each servo so the move is synchronized
            # the servo that covers the longest distance moves at the maximum speed
            # and the rest of servos move at a speed proportional to the distance they need to cover

            # Find the maximum distance to calculate proportional speeds
            max_distance = max(distances.values(), default=0)
            if max_distance == 0:
                # No movement needed. All servos are already at target positions.
                return True

            # Calculate speeds for each servo based on their distances
            servo_speeds = {}
            for servo_id, distance in distances.items():
                # Scale the speed proportionally to the distance
                servo_speeds[servo_id] = max(1, int((distance / max_distance) * max_speed))
                
            # Write the selected configuration
            for servo_id in self.servo_ids:
                success = self.write_position_by_id(
                    servo_id=servo_id,
                    position=target_arm_position[servo_id],
                    speed=servo_speeds[servo_id],  
                    acceleration=max_acceleration  
                )
                if not success:
                    print(f"Failed to move Servo {servo_id} to Position {target_arm_position[servo_id]}")


    def load_positions(self, file_path):
        """
        Load and validate list of named positions from a JSON config file.

        Args:
            file_path (str): Path to the JSON file.

        Updates: self.named_positions and self.servo_ids
            self.named_positions: A dictionary containing configurations with integer keys and values.
            self.servo_ids: A list containing servo IDs
        """
        try:
            # Load the JSON file
            with open(file_path, "r") as file:
                named_positions = json.load(file)
            
            # Validate and ensure keys/values are integers
            named_positions = {
                str(name): {int(servo_id): int(servo_position) for servo_id, servo_position in arm_position.items()}
                for name, arm_position in named_positions.items()
            }

        except FileNotFoundError:
            print(f"Error: {file_path} file not found.")
            raise
        except json.JSONDecodeError:
            print(f"Error: Failed to parse {file_path}. Please ensure it contains valid JSON.")
            raise
        except ValueError as e:
            print(f"Error: {e}")
            raise
        if self.validate(named_positions):
            # Get the list of servo_ids from the first saved position
            self.servo_ids = list(named_positions[next(iter(named_positions)) ].keys())  
            self.named_positions = named_positions
        else:
            raise ValueError(f"Invalid positions in the JSON file {file_path}.")


# Helper function to wait for a keypress
def wait_for_keypress():
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


# Main function for argument parsing and running the controller
def main():
    parser = argparse.ArgumentParser(description="Save and retrieve position of a robotic arm made of STS3216 Servos ID1-6 using the STServo SDK.")
    parser.add_argument("--port", type=str, default='/dev/ttyACM0', help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Baud rate (default: 1000000)")
    parser.add_argument("--configfile", type=str, default="saved_positions.json", help="JSON config file (default: saved_positions.json)")

    args = parser.parse_args()

    # Initialize the RobotArmController    
    try:
        robot_arm = RobotArmController(port=args.port, baudrate=args.baudrate)
    except Exception as e:
        print(f"Failed to open port and set baudrate: {e}")
        return

    # Load and validate saved positions from the JSON file
    try:
        robot_arm.load_positions(args.configfile)
    except Exception as e:
        print(f"Failed to load saved positions: {e}")
        return

    # Main loop
    try:
        while True:
            # Display menu
            menu_message = f"\n| ESC - Exit | R - Read | " + " | ".join(
                [f"{idx-1} - {config_name}" for idx, config_name in enumerate(robot_arm.named_positions.keys(), start=1)]
                ) +  " |"
            print(menu_message)

            # Get user input
            choice = wait_for_keypress().upper()

            if choice == chr(0x1B):  # ESC key
                print("Exiting program.")
                break

            elif choice == "R":
                # Read current position and speed
                robot_arm.read_current_arm_position(verbose=True)

            elif choice.isdigit() and 0 <= int(choice) <= len(robot_arm.named_positions)-1:
                
                config_name = list(robot_arm.named_positions.keys())[int(choice)]
                # Move the arm to the selected configuration
                print(f"Writing ({choice}):'{config_name}' position to servos:")
                robot_arm.write_arm_position(robot_arm.named_positions[config_name], verbose = True)

            else:
                print("Invalid choice. Please select a valid option.")
    finally:
        # Close the port on exit
        robot_arm.close_port()
        print("Port closed.")

if __name__ == "__main__":
    main()

# def sync_read_arm_position(self):
#         """
#         Perform a sync read operation to retrieve the positions of the servos in a robot arm.

#         Returns:
#             dict: A dictionary mapping servo IDs to their current positions.
#         """
#         # Create a GroupSyncRead instance for reading positions
#         group_sync_read = GroupSyncRead(self.packet_handler, STS_PRESENT_POSITION_L, 4)

#         # Add servos to the sync read groupr
#         for servo_id in self.servo_ids:
#             sts_addparam_result = group_sync_read.addParam(servo_id)
#             if not sts_addparam_result:
#                 print(f"[ID:{servo_id:03d}] groupSyncRead addParam failed")
#                 return {}  # Return an empty dictionary if adding fails

#         # Perform the sync read
#         sts_comm_result = group_sync_read.txRxPacket()
#         if sts_comm_result != COMM_SUCCESS:
#             print(f"Sync read failed: {self.packet_handler.getTxRxResult(sts_comm_result)}")
#             return {}

#         # Retrieve the positions of the servos
#         positions = {}
#         for servo_id in self.servo_ids:
#             # Check if data is available for the current servo
#             sts_data_result, sts_error = group_sync_read.isAvailable(servo_id, STS_PRESENT_POSITION_L, 4)
#             if sts_data_result:
#                 # Get the position value
#                 positions[servo_id] = group_sync_read.getData(servo_id, STS_PRESENT_POSITION_L, 2)
#             else:
#                 print(f"[ID:{servo_id:03d}] groupSyncRead getData failed")
#                 continue

#             # Handle potential errors for the current servo
#             if sts_error != 0:
#                 print(f"[ID:{servo_id:03d}] {self.packet_handler.getRxPacketError(sts_error)}")

#         # Clear the parameter storage after reading
#         group_sync_read.clearParam()

#         return positions


#     def sync_write_arm_position(self, target_arm_position, max_speed, max_acceleration):
#         """
#         Move the arm to the target positions with synchronized speeds.

#         Args:
#             target_arm_position (dict): A dictionary where keys are servo IDs and values are target positions.
#             max_speed (int): The maximum speed for the servo that needs to cover the longest distance.
#             acceleration (int): The acceleration for all servos.

#         Returns:
#             bool: True if the movement was initiated successfully, False otherwise.
#         """

#         # get current position
#         current_arm_position = self.sync_read_arm_position(self, list(target_arm_position.keys()))
#         if not current_arm_position:
#             print("Failed to retrieve current arm positions.")
#             return False
        
#         distances = {}
#         # calculate distances for each servo
#         for servo_id, position in target_arm_position.items():

#             if servo_id in current_arm_position:
#                 # calculate distance
#                 distances[servo_id] = abs(position - current_arm_position[servo_id])
#             else:
#                 print(f"Servo ID {servo_id} not found in current position data.")
#                 return False
            
#         # calculate speed for each servo so the move is synchronized
#         # the servo that covers the longest distance moves at the maximum speed
#         # and the rest of servos move at a speed proportional to the distance they need to cover

#         # Find the maximum distance to calculate proportional speeds
#         max_distance = max(distances.values(), default=0)
#         if max_distance == 0:
#             print("No movement needed. All servos are already at target positions.")
#             return True

#         # Calculate speeds for each servo based on their distances
#         servo_speeds = {}
#         for servo_id, distance in distances.items():
#             # Scale the speed proportionally to the distance
#             servo_speeds[servo_id] = max(1, int((distance / max_distance) * max_speed))

#         # Add all servo positions to the sync write buffer
#         for servo_id, position in target_arm_position.items():
#             sts_addparam_result = self.packet_handler.SyncWritePosEx(servo_id, position, servo_speeds[servo_id], max_acceleration)
#             if not sts_addparam_result:
#                 print(f"[ID:{servo_id:03d}] groupSyncWrite addparam failed")
#                 return False  # Return early if adding a parameter fails

#         # Execute the sync write operation
#         sts_comm_result = self.packet_handler.groupSyncWrite.txPacket()
#         if sts_comm_result != COMM_SUCCESS:
#             print(f"Sync write failed: {self.packet_handler.getTxRxResult(sts_comm_result)}")
#             return False

#         # Clear the sync write parameter storage after execution
#         self.packet_handler.groupSyncWrite.clearParam()
#         return True
    