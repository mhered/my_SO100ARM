# README.md

* [Bill of Materials](./BOM.md)
* [Configuring the Servos](./Configuring_the_Servos.md)
* [Assembly instructions](./Assembly.md)
* Crude python script to control the arm: [./scripts/README.md](./scripts/README.md)
* [URDF](./URDF.md) - how to model the SO100ARM: creating `my_arm_description` ROS2 package
* [Python, un robot y un patito de goma](./pycamp.md) - my notes on my first attempt at setting up, teleoperating and training the arm to pick up a rubber duckie (project during pycamp2025)
* [Simulating the arm and a duckie](./duckie.md) (WIP) - I have future plans to simulate the exercise of picking up the duckie. Note: temporarily prototyped in `~/bar_ws/` (workspace of the Robotics Challenge) check out `~/bar_ws/src/bar_examples/gamecity`I added the `duckie` to `simple_gamecity`, but it is not working well yet.

## To Do

- [ ] merge in changes made to the code and config files, currently stored only in my local copy of `lerobot` repo
- [ ] fix and move over here the quick prototype to spawn a duckie made using `simple_gamecity` world from `~/bar_ws`  
  - [ ] Fix it, it is not workign yet 
  - [ ] move it to `src`, 
  - [ ] symlink it to `dev_ws` workspace, 
  - [ ] move notes her
  - [ ] commit

## Sources

* The twitter thread that started it all: https://x.com/RemiCadene/status/1825455895561859185

* Playlist of SO-ARM100 videos from The Robot Studio (many are repeated): https://www.youtube.com/playlist?list=PLy7gxZH9jzfR0l8fYH8C1nyEc4pxSBrer

