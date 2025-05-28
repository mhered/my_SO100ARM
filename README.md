# README.md

* [Bill of Materials](./BOM.md)
* [Configuring the Servos](./Configuring_the_Servos.md)
* [Assembly instructions](./Assembly.md)
* Crude python script to control the arm: [./scripts/README.md](./scripts/README.md)
* [URDF](./URDF.md) - how to model the SO100ARM: creating `my_arm_description` ROS2 package
* [Python, un robot y un patito de goma](./pycamp.md) - my notes on my first attempt at setting up, teleoperating and training the arm to pick up a rubber duckie (project during pycamp2025)
* [Creating custom worlds](./src/my_worlds/README.md) - my exploration on creating custom models and worlds. I am planning to simulate the exercise of picking up the rubber duckie, and also the eco-disaster challenge from piwars2024. 

## To Do

- [ ] merge in changes made to the code and config files, currently stored only in my local copy of `lerobot` repo
- [x] fix and move over here the quick prototype to spawn a duckie made using `simple_gamecity` world from `~/bar_ws`  
  - [x] Fix it, it is not working yet 
  - [x] move it to `src`, 
  - [x] symlink it to `dev_ws` workspace, 
  - [ ] move notes here
  - [ ] commit

## Sources

* The twitter thread that started it all: https://x.com/RemiCadene/status/1825455895561859185

* Playlist of SO-ARM100 videos from The Robot Studio (many are repeated): https://www.youtube.com/playlist?list=PLy7gxZH9jzfR0l8fYH8C1nyEc4pxSBrer

