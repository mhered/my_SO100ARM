# Configuring the Servos

The goal of this preliminary step is:

1. reprogram the ST3215 Servos (I used the 12V, 30kg version) so each one has a different ID number
2. and then zero the servos. 

The kit comes with two driver boards:

- [Waveshare 25514 Serial Bus Servo Driver Board](./BOM/Waveshare_25514_Driver_Board.md) referred to in the official BOM
- the [FE-URT-1 board](https://www.feetechrc.com/FE-URT1-C001.html) ships as part of the [ST3215 servo kit](./BOM/STS3215_servo_kit.md) (and is also referred to in this video [Standard Open ARM 100 5DOF Servos - Low cost DIY 3dprinted robot arm for Le Robot by Hugging Face](https://www.youtube.com/watch?v=fy6Jqq_QaGo)) 

Each board allows to follow a slightly different process to complete this step, with each of the two methods having pros and cons

| Method 1 - Waveshare 25514 board                             | Method 2 - FE-URT-1 board                                    |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| Requires installing the lerobot environment                  | Requires windows                                             |
| Provides a basic CLI with very limited options (renaming and zeroing only) | Provides a nice debugging GUI that allows operating the servos and configuring many options |
| Requires that servos are connected to the PC one by one, so it does not work on the assembled robot | Works on the assembled robot with all the servos connected in daisy chain |
| The Waveshare board fits the 3D printed support and (presumably) works with lerobot software | The FE-URT-1 does not fit the 3D printed support, and it is not confirmed that will work with lerobot software |

## Method 1: using the Waveshare 25514 board

### Install LeRobot

Followed instructions here: https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md

Notes:

1. during miniconda installation I got a WARNING that PYTHONPATH environment variable is set and I should check all files in the path are compatible with python version in miniconda (??)

   Workaround - unset PYTHONPATH manually before activating conda:

```bash
$ unset PYTHONPATH
$ conda activate lerobot
```

It is also possible to automate this step modifying the conda environment activation script (not done)

2. After installation conda base environment activates automativally so when I open a terminal I get by default the prepend `(base) $`. I don't like this as I am not sure if the behaviour of my other projects (ROS, C++) may change under conda

   Workaround - prevent auto activation of base with:

```bash
$ conda config --set auto_activate_base false
```

Then I can activate it manually only when needed.

3. To note, the  `conda`  installer installed `ffmpeg`, and removed `opencv` and reinstalled it with `conda`

### Configuring the servos

The video [Assemble and Calibrate SO-100: LeRobot Tutorial #7 by Jess Moss]( https://www.youtube.com/watch?v=FioA2oeFZ5I ) describes servo configuration with the Waveshare board

Starting lerobot:

```bash
$ cd ~/lerobot
$ unset PYTHONPATH
$ conda activate lerobot
```

Detect the port:

```bash
(lerobot)$ python lerobot/scripts/find_motors_bus_port.py
```

Rename the servos:

```bash
(lerobot)$ python lerobot/scripts/configure_motor.py \
  --port /dev/ttyACM0 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 1
```

Note: need to remove gears from the servos of the leader arm! (why buy the 30kg servos then?) 

Note: The calibration script mentioned in the video does not exist.

## Method 2: using the FE-URT-1 driver board

### Resources

* The video [Standard Open ARM 100 5DOF Servos - Low cost DIY 3dprinted robot arm for Le Robot by Hugging Face](https://www.youtube.com/watch?v=fy6Jqq_QaGo) describes the connection and renaming steps, but skips the installation of the software, and is for the 6V version not the 12V one.

###  Step by step guide

1. Local copies of the required software can be found in folder [./Feetech_sfw_win](./Feetech_sfw_win) or downloaded from [Feetech website](https://www.feetechrc.com/software.html). Note both packages are for Windows OS:

   - the FE-URT-1 board driver `CH341SER.EXE`

   * the debugging GUI `FD.EXE` (version FD1.9.8.3) 


2. Install the driver `CH341SER.EXE`
3. Connect the board to a 12V power supply and to the USB. Check switch is set to 5V. Connect the Servo. **Note: Had to connect 12V input to G - V1 terminals, even though they are rated DC6V-8V. If I connect to G-V2 (rated 8V-24V) it would not work!!** Did this by executive decision, I found nothing online.

<img src="/home/mhered/my_SOARM100/assets/board_detail_xs.jpg" style="zoom:50%;" />

2. Check in **Device Manager** under **Ports**: `USB-SERIAL CH340 (COM4)`
3. Install and launch `FD.EXE`
4. In **Debug** tab, set **BaudR** to 1000000, then click **Open** then **Search** to find the servos
5. Click on the detected Servo, go to tab **Programming**, select **ID**, rename to 6 , click **Save**
6. Connect another servo in daisy chain, repeat the process and rename it to 5, then 4, etc until 1.

<img src="/home/mhered/my_SOARM100/assets/renaming_servos_xs.jpg" style="zoom: 33%;" />

7. In **Debug** tab, select the **Sync Write** toggle, set **Goal** to 2048 then click **Set** to zero all servos to the middle point.
