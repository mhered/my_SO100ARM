# Crude Python code to control SO-100ARM

Based on `STservo_sdk` library: https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/STServo_Python.zip - copy saved locally [here](./STServo_Python.zip)

Changelog 

* New `crude_ping.py` - a simple ping written with help of ChatGPT

* `ping.py`  - add argument parsing, chang parameters ID, port, baudrate. Uses `STservo_sdk` library 

* `read.py` - add argument parsing, change parameters ID, port, baudrate,. Uses `STservo_sdk` library 

* `write.py` - modify parameters: ID, port, position values, moving speed and acceleration. Uses `STservo_sdk` library.

* New  `my_STServo.py` - define class `STServo_Controller` that allows reading positions of the robot arm and retrieving them. Uses `STservo_sdk` library 

* New `saved_positions.json` - json config file with positions  rest, stretched, neutral and cool

* `positions.txt` - text output for positions rest, stretched, neutral and cool

