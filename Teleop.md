# Teleop

Install phosphobot (see https://docs.phospho.ai/installation)

```bash
curl -fsSL https://raw.githubusercontent.com/phospho-app/phosphobot/main/install.sh | sudo bash
sudo apt update && sudo apt install --only-upgrade phosphobot
```

Run

```bash
phosphobot run
025-07-04 22:01:24.370 | INFO     | __main__:<module>:4 - Starting phosphobot...
sys.stdout.encoding = utf-8

    ░█▀█░█░█░█▀█░█▀▀░█▀█░█░█░█▀█░█▀▄░█▀█░▀█▀
    ░█▀▀░█▀█░█░█░▀▀█░█▀▀░█▀█░█░█░█▀▄░█░█░░█░
    ░▀░░░▀░▀░▀▀▀░▀▀▀░▀░░░▀░▀░▀▀▀░▀▀░░▀▀▀░░▀░

    phosphobot 0.3.52
    Copyright (c) 2025 phospho https://phospho.ai
            
2025-07-04 22:01:25.050 | WARNING  | __main__:run:294 - Port 80 is unavailable. Trying next...
pybullet build time: Jan 29 2025 23:16:28
INFO:     Started server process [849619]
INFO:     Waiting for application startup.
2025-07-04 22:01:25.861 | DEBUG    | phosphobot.hardware.sim:simulation_init:23 - Simulation: headless mode enabled
2025-07-04 22:01:25.861 | DEBUG    | phosphobot.utils:login_to_hf:216 - Hugging Face token file not found.
2025-07-04 22:01:25.861 | SUCCESS  | phosphobot.app:lifespan:81 - Startup complete. Go to the phosphobot dashboard here: http://192.168.8.194:8020
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8020 (Press CTRL+C to quit)
```

Open dashboard at http://192.168.8.194:8020

Really neat interface!!

It moves both with keyboard and gamepad, BUT motor labelling is wrong! It turns out at some point the criteria to number the motors changed and now it should be 1 for the base and 6 for the  gripper, see [phospho website](https://github.com/phospho-app/phosphobot/tree/main/scripts/feetech): 

> On the SO-100 and SO-101 robot, the ids of the motors start at 1 (base) and go up to 6 (gripper).

 but also iaw [lerobot so100 website](https://huggingface.co/docs/lerobot/so100):

> 'gripper' motor id set to 6

I use the FE.-URT-1 board, driver and Windows GUI as described [here](https://github.com/mhered/my_SO100ARM/blob/main/Configuring_the_Servos.md)  to avoid having to disconnect/disassemble the arm.

See also this video from Ilia: https://www.youtube.com/watch?v=DeBLc2D6bvg
And branch `joystick` of his `lerobot`fork for the code to use a PS4 gamepad for teleop: https://github.com/huggingface/lerobot/compare/main...IliaLarchenko:lerobot:joystick  

The video contains also useful advice for recording good episodes:

- camera locataion: a wrist camera, a forward camera on the shoulder, adnd a top camerra on a tripod
- use a tablecloth
- make sure the leader is out of view
- if you cannot do the task looking only at the camera feed there's not enough info, move the cameras
- record episodes also in extreme positions
- etc.