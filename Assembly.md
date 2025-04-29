# Assembly of the Follower Arm

## Sources

The video [Assemble and Calibrate SO-100: LeRobot Tutorial #7 by Jess Moss]( https://www.youtube.com/watch?v=FioA2oeFZ5I ) describes the full process for servo configuration and assembly, even has a step by step animation of the assembly sequence for the current version (8k), which I wish I had seen earlier. 

Another good updated assembly guide by MrRobot:  [SO-Arm100 assembly](https://www.youtube.com/watch?v=2XQxbHTk-ko)

I originally followed the [Build guide for Standard Open ARM 100 5DOF - Low cost DIY 3dprinted robot arm Le Robot Hugging Face](https://www.youtube.com/watch?v=QkIgxTCq3MY), which actually describes the assembly of an older version (5d). The current version (8k) contains more parts and the assembly sequence is slightly different, so I made my own guide below.

The repo includes a STEP file of the current version [here](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STEP). I imported it into FreeCAD, exploded it into parts as explained [here](https://www.youtube.com/watch?v=4UITPBWyehs) and colored them differently for easier identification (see the resulting FreeCAD model [here](./CAD/SO_5DOF_ARM100_08k.FCStd)):  

<img src="/home/mhered/my_SOARM100/assets/follower_arm.png" style="zoom: 33%;" />

## Step by step assembly

Each servo comes with 9x metal screws + 8x self-tapping screws + 1x flanged self-tapping screw. The assembly uses exclusively the screws provided with the servos.

1. On all servos, install the drive wheel (the one with grooves) on the drive axle, trying to align the four holes with the sides of the servo. Secure it in place using one metal screw through the centre. 

2. On all servos (except **Servo_#2**) insert the free wheel (the one without grooves) in the free axle, but do not install the flanged self-tapping screw provided to fix it, as it impairs rotation
3. Insert **Servo_#6** into the **Base** (green) with the drive axle pointing upwards. Route the cable going to the board downwards through the hole in the **Base**.
4. Insert the **Base_Collar** (brown) holding together the **Base**  and **Servo_#6**, taking care to route the cable from **Servo_#6** to **Servo_#5** through the inside of **Base_Collar** without pinching it - there is a dedicated gap - and then clip the cable in the dedicated vertical slot in **Base**. Fix the **Base_Collar** in position with 2 self-tapping screws. 
5. Secure **Servo_#6** to the **Base** with 2 additional self-tapping screws from the top
6. Prepare the **Shoulder** (orange): preload the 4 metal screws that will later be used to attach to the free wheel of **Servo_#6** . Cover them with an additional free wheel (taken from **Servo_#2**) and attach it with a flanged self-tapping screw. There is a dedicated video with instructions for this complicated step: [Assembly Detail](https://www.youtube.com/watch?v=temou_Nr5s8) 
7. Install the **Shoulder** bracing the drive and free wheels of **Servo_#6**. Attach the **Shoulder** to the drive wheel using 4x metal screws, taking care that the **Shoulder** should be pointing forward when the servo is at the central position. Install 4x metal screws to secure the free wheel to the **Shoulder**, rotating it to get access to the holes as needed. 
8. Secure the extra free wheel in place fixing the **Stopper** (purple) to the **Base** with two self-tapping screws
9. Insert **Servo_#5**  into the slot at the top of the **Shoulder** with the drive wheel facing towards the right, 
10. and secure it in place with the **Collar** (blue) taking care not to pinch the cable coming from **Servo_#6**. Secure the set of three parts in place with 4 self-tapping screws.
11. Install the **Lower_Arm** (red) bracing the drive and free wheels of **Servo_#5**. Attach the **Lower_Arm** to the drive wheel of **Servo_#5** using 4 metal screws, taking care that when the servo is at the central position, the **Lower_Arm** should be pointing upwards. Route the cable to **Servo_#4** through the clip on the left side of the **Lower_Arm**. 
12. Insert **Servo_#4**  into the slot at the top of the **Lower_Arm** with the drive wheel facing towards the right,  and secure it in place with 4 self-tapping screws. Route the cable to **Servo_#3** through the groove on the left side of the **Lower_Arm** and secure it with a flanged self-tapping screw. 
13. Install the **Upper_Arm** (white) bracing the drive and free wheels of **Servo_#4**. Attach the **Upper_Arm** to the drive wheel of **Servo_#4** using 4 metal screws, taking care that when the servo is at the central position, the **Upper_Arm** should be point forward. Route the cable to **Servo_#3** through the groove on the left side of the **Upper_Arm** and secure it with a flanged self-tapping screw.
14. Insert **Servo_#3**  into the slot at the top of the **Upper_Arm** with the drive wheel facing towards the right. Insert the **Collar** (gold) taking care not to pinch the cable coming from **Servo_#4** and secure the assembly of three parts in place with 4 self-tapping screws.   
15. Prepare the **Wrist** (pink) by first installing **Servo_#2** inside it, with the drive axle facing forward and no wheels. Push it in until the surface if flush. Secure it with 2 self-tapping screws in the holes provided plus 1 flanged self-tapping screw on the hole of the free axle. Install the drive wheel.
16. Install **Wrist** bracing the drive and free wheels of **Servo_#3** . Attach the **Wrist** to the drive wheel of **Servo_#3** using 4x metal screws, taking care that when the servo is at the central position, the **Wrist** should be pointing forward. Attach the free wheel with 4 metal screws.
17. Attach the **Fixed_Gripper** (light purple) to the drive wheel of **Servo_#2** using 4 metal screws, taking care that when the servo is at its central position, the gripper should be horizontal, with the **Fixed_Gripper** on the right side.
18. Install **Servo_#1** onto **Fixed_Gripper** in the dedicated slot, with the drive wheel pointing upwards and secure it with 6 self-tapping screws.
19. Attach the **Movable_Jaw** (yellow) to the drive wheel of **Servo_#1** using 4 metal screws, taking care that when the servo is at its central position, the **Movable_Jaw** should point forward. Attach the free wheel with 4 metal screws.