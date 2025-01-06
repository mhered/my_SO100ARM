# Crude Python code to control SO-100ARM

Based on `STservo_sdk` library: https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/STServo_Python.zip - a copy saved locally [here](./STServo_Python.zip)

See: https://www.waveshare.com/product/bus-servo-adapter-a.htm

## Changelog 

* New `crude_ping.py` - a simple ping written with help of ChatGPT

* `ping.py`  - add argument parsing, change parameters ID, port, baudrate. Uses `STservo_sdk` library 

* `read.py` - add argument parsing, change parameters ID, port, baudrate. Uses `STservo_sdk` library 

* `write.py` - modify parameters: ID, port, position values, moving speed and acceleration. Uses `STservo_sdk` library.

* New  `my_STServo.py` - define class `STServo_Controller` that allows reading positions of the robot arm and retrieving them. Uses `STservo_sdk` library 

* New `saved_positions.json` - json config file with positions  rest, stretched, neutral and cool

* `positions.txt` - text output for positions `rest`, `stretched`, `neutral` and `cool`

## 2nd Commit

* modify `sync_read.py` to change parameter `port`. Note it fails with error: `AttributeError: 'sts' object has no attribute 'scs_makeword'`
* modify default JSON config file `saved_positions.json`  to add new positions: `snake`,`jose`,`guille1`, `guille2` 
* New `pick_barrel.json` - JSON config file to simulate picking up a barrel and moving it to another location. Defines positions:`| 0 - rest | 1 - clear_top | 2 - approach_init_pos | 3 - init_pos_open_jaw | 4 - init_pos_closed_jaw | 5 - lift_clear | 6 - approach_pos2 | 7 - pos2_closed_jaw | 8 - pos2_open_jaw |`

* New `saved_positions.json` - test JSON config file with 2 "safe" positions close to each other: `neutral`, `neutral2`
* Rename `my_STServo.py` to `RobotArmController.py`: 
  -  Rename class `STServo_Controller` to `RobotArmController`. 
  -  Add class constants `_DEFAULT_PORT`. `_DEFAULT_BADURATE`, `_DEFAULT_CONFIGFILE`
  -  in `__init__()` method:
     -  Add class variables: `self.servo_ids`, `self.named_positions`, `self.current_arm_position`
     -  rename `self.device_name` to `self.port`
     -  open port at initialization
  -  in `open_port()` method: improve error handling
  -  rename method `read_position_and_speed()` to `read_position_and_speed_by_id()`
  -  in `ping()` method: add `verbose` parameter
  -  rename method `write_position_ex()` to `write_positions_by_id()`
  -  new method `validate()` - ensure a list of arm positions are valid: all positions defined by same the same list of servo IDs, all positions are in range 0-4095 and all servos ping succesfully. 
  -  make auxiliary function `load_positions()` a class method. Save validated list of servos in `self.servo_id`, and loaded arm positions in `self.named_positions`
  -  make class method `wait_for_keypress()` an auxiliary function
  -  new method `read_current_arm_position()` - encapsulates reading all servos of the arm, with verbose parameter, stores current position in `self.current_arm_position`
  -  new method `write_arm_position()` - encapsulates writing positions to all servos of the arm, ensuring all servos are synchronized and maximum speed is below `max_speed`
  -  new methods `sync_read_arm_position()` and `sync_write_arm_position()`, commented out until I figure out what is wrong with sync r/w operations 
