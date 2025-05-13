# Python, un robot y un patito de goma

Log of my first try at training the SOARM100 with lerobot during the project "Python, un robot y un patito de goma" at Pycamp2025

Todo: 

- [ ] add pics and videos

## Installation lerobot

Source: https://github.com/huggingface/lerobot

Clone the repo:

```bash
$ git clone https://github.com/huggingface/lerobot.git
$ cd lerobot
```

install `miniconda`:

```bash
$ mkdir -p ~/miniconda3
$ wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
$ bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
$ rm ~/miniconda3/miniconda.sh
```

source and initialize `conda`:

```bash
$ source ~/miniconda3/bin/activate
$ conda init --all
```

to prevent conda for launching automatically:

```bash
$ conda config --set auto_activate_base false
```

create a virtual environment with python 3.10, activate it and install `ffmpeg`:

```bash
$ conda create -y -n lerobot python=3.10
$ conda activate lerobot
$ conda install ffmpeg -c conda-forge
```

install dependencies and lerobot:

```bash
$ sudo apt-get install cmake build-essential python3-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config
$ pip install -e .
```

## Running the examples

```bash
$ conda activate lerobot
$ python3 examples/1_load_lerobot_dataset.py 
...
Traceback (most recent call last):
  File "/home/mhered/lerobot/examples/1_load_lerobot_dataset.py", line 103, in <module>
    frames = [dataset[idx][camera_key] for idx in range(from_idx, to_idx)]
  File "/home/mhered/lerobot/examples/1_load_lerobot_dataset.py", line 103, in <listcomp>
    frames = [dataset[idx][camera_key] for idx in range(from_idx, to_idx)]
  File "/home/mhered/lerobot/lerobot/common/datasets/lerobot_dataset.py", line 739, in __getitem__
    video_frames = self._query_videos(query_timestamps, ep_idx)
  File "/home/mhered/lerobot/lerobot/common/datasets/lerobot_dataset.py", line 711, in _query_videos
    frames = decode_video_frames(video_path, query_ts, self.tolerance_s, self.video_backend)
  File "/home/mhered/lerobot/lerobot/common/datasets/video_utils.py", line 65, in decode_video_frames
    return decode_video_frames_torchcodec(video_path, timestamps, tolerance_s)
  File "/home/mhered/lerobot/lerobot/common/datasets/video_utils.py", line 194, in decode_video_frames_torchcodec
    decoder = VideoDecoder(video_path, device=device, seek_mode="approximate")
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torchcodec/decoders/_video_decoder.py", line 98, in __init__
    core.add_video_stream(
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/_ops.py", line 723, in __call__
    return self._op(*args, **kwargs)
ValueError: No valid stream found in input file. Is -1 of the desired media type?

```

This failed

tried to visualize with:

```bash
$ python lerobot/scripts/visualize_dataset.py     --repo-id lerobot/pusht     --episode-index 0
...
Traceback (most recent call last):
  File "/home/mhered/lerobot/lerobot/scripts/visualize_dataset.py", line 292, in <module>
    main()
  File "/home/mhered/lerobot/lerobot/scripts/visualize_dataset.py", line 288, in main
    visualize_dataset(dataset, **vars(args))
  File "/home/mhered/lerobot/lerobot/scripts/visualize_dataset.py", line 147, in visualize_dataset
    for batch in tqdm.tqdm(dataloader, total=len(dataloader)):
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/tqdm/std.py", line 1181, in __iter__
    for obj in iterable:
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/dataloader.py", line 708, in __next__
    data = self._next_data()
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/dataloader.py", line 1480, in _next_data
    return self._process_data(data)
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/dataloader.py", line 1505, in _process_data
    data.reraise()
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/_utils.py", line 733, in reraise
    raise exception
ValueError: Caught ValueError in DataLoader worker process 0.
Original Traceback (most recent call last):
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/_utils/worker.py", line 349, in _worker_loop
    data = fetcher.fetch(index)  # type: ignore[possibly-undefined]
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/_utils/fetch.py", line 52, in fetch
    data = [self.dataset[idx] for idx in possibly_batched_index]
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/utils/data/_utils/fetch.py", line 52, in <listcomp>
    data = [self.dataset[idx] for idx in possibly_batched_index]
  File "/home/mhered/lerobot/lerobot/common/datasets/lerobot_dataset.py", line 739, in __getitem__
    video_frames = self._query_videos(query_timestamps, ep_idx)
  File "/home/mhered/lerobot/lerobot/common/datasets/lerobot_dataset.py", line 711, in _query_videos
    frames = decode_video_frames(video_path, query_ts, self.tolerance_s, self.video_backend)
  File "/home/mhered/lerobot/lerobot/common/datasets/video_utils.py", line 65, in decode_video_frames
    return decode_video_frames_torchcodec(video_path, timestamps, tolerance_s)
  File "/home/mhered/lerobot/lerobot/common/datasets/video_utils.py", line 194, in decode_video_frames_torchcodec
    decoder = VideoDecoder(video_path, device=device, seek_mode="approximate")
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torchcodec/decoders/_video_decoder.py", line 98, in __init__
    core.add_video_stream(
  File "/home/mhered/miniconda3/envs/lerobot/lib/python3.10/site-packages/torch/_ops.py", line 723, in __call__
    return self._op(*args, **kwargs)
ValueError: No valid stream found in input file. Is -1 of the desired media type?
```

this opened a GUI but was empty

found downloaded data in `~/.cache`

Watching this video: [How to Utilize LeRobot, Isaac Sim and ROS2](https://www.youtube.com/watch?v=eO5wMzw9LeQ)

Maybe example 1 failed because I did not install aloha or pusht so I tried

```bash
$ pip install -e ".[aloha, pusht]"
```

Still not working! Complains video file not valid... missing codecs? but VLC opens files... wrong range?

Clearly something is not working properly, but I decide to proceed anyway...

##  Connect the arms

Connect follower first then leader and update ports on `/home/mhered/lerobot/lerobot/common/robot_devices/robots/configs.py`

```bash
...
class So100RobotConfig(ManipulatorRobotConfig):
...
    leader_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port='/dev/ttyACM1',
...
    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port='/dev/ttyACM0',
...
```

## Calibrate

there is a problem with the range of motor 6, crashes when rotating to the left, I had to manually adjust limits on `/home/mhered/lerobot/lerobot/common/robot_devices/motors/feetech.py`:

```python 
LOWER_BOUND_LINEAR = -110 # was -10
```

## Teleop without cameras

```bash
(lerobot)$ python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --robot.cameras='{}' \
  --control.type=teleoperate
```

## Setup cameras

Find camera indexes with:

```bash
(lerobot)$ python lerobot/common/robot_devices/cameras/opencv.py \
    --images-dir outputs/images_from_opencv_cameras
```



In `/home/mhered/lerobot/lerobot/common/robot_devices/robots/configs.py`:

```bash
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "silla": OpenCVCameraConfig(
                camera_index=7,
                fps=30,
                width=640,
                height=480,
            ),
            "cabeza": OpenCVCameraConfig(
                camera_index=10,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )
```



## Teleop with cameras (& visualize)

```bash
(lerobot)$ python lerobot/scripts/control_robot.py --robot.type=so100 --control.type=teleoperate --control.display_data=true
```



## Record a couple of episodes (init repo)

```bash
(lerobot)$ python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a ducky." \
  --control.repo_id=${HF_USER}/so100_test \
  --control.tags='["so100","tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
```

Add more episodes (resume recording):

```bash
(lerobot)$ python lerobot/scripts/control_robot.py   --robot.type=so100   --control.type=record   --control.fps=30   --control.single_task="Grasp a ducky."   --control.repo_id=${HF_USER}/so100_test    --control.warmup_time_s=5   --control.episode_time_s=30   --control.reset_time_s=20   --control.num_episodes=10   --control.push_to_hub=true --control.display_data=true --control.resume=true
```

Why is the gripper stalling?

## Training

```bash
(lerobot) $ python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/so100_test \
  --policy.type=act \
  --output_dir=outputs/train/act_so100_test \
  --job_name=act_so100_test \
  --policy.device=cuda \
  --wandb.enable=true
```

### Setup Weight and Biases

`wandb` already installed

```bash
$ wandb --version
wandb, version 0.19.10
```

Created account in wandb.ai with Google OAuth SPAM, username manoloheredia, organization Duckietown API key in secrets

```bash
(lerobot) $ wandb login
wandb: WARNING Using legacy-service, which is deprecated. If this is unintentional, you can fix it by ensuring you do not call `wandb.require('legacy-service')` and do not set the WANDB_X_REQUIRE_LEGACY_SERVICE environment variable.
wandb: Logging into wandb.ai. (Learn how to deploy a W&B server locally: https://wandb.me/wandb-server)
wandb: You can find your API key in your browser here: https://wandb.ai/authorize?ref=models
wandb: Paste an API key from your profile and hit enter, or press ctrl+c to quit: 
wandb: No netrc file found, creating one.
wandb: Appending key for api.wandb.ai to your netrc file: /home/mhered/.netrc
wandb: Currently logged in as: manoloheredia (manoloheredia-duckietown) to https://api.wandb.ai. Use `wandb login --relogin` to force relogin
(lerobot) mhered@Vader:~/lerobot$ 

```

## Evaluate

```bash
(lerobot) $ python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a duckie" \
  --control.repo_id=${HF_USER}/eval_act_so100_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=30 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/lerobot_final
```

