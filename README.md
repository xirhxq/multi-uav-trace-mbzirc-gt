# multi-uav-trace-mbzirc-gt

This repository contains a demonstration of multi-UAV formation tracking:
- Using MBZIRC 2023 simulator (https://github.com/osrf/mbzirc);
- Using groundtruth position data;
- Using Docker to run.

## Requirements

- Ubuntu
- Docker
- `nvidia-docker2` Docker Image

## Usage

1. Build the Docker image:
    ```bash
    ./docker/build_docker.sh
    ```

2. Run the Docker container:
    ```bash
    ./docker/run_docker.sh
    ```

3. Run the simulation inside Docker container:
    ```bash
    tmuxinator start . -p src/multi-uav-trace-mbzirc-gt/tmuxinator.multi-uav-trace-mbzirc-gt.yml
    ```

4. To end the simulation, press `Ctrl+B` and then `q`. Make sure that you are in EN mode.

5. To exit Docker container, press `Ctrl+D` in terminal.

## Options

### Where to start? Designed prepare points or standard formation points?

In constructor of `Task` class in `src/suav.cpp`:

```cpp
    // ====================
    // if we start from expected formation at time 0
    prepare_point_ = desired_point_;
    
    // if we start from prepare point
    prepare_point_ = Eigen::Vector3d(arr.data()) + global_offset_;
    // ====================
```

You could choose one by commenting the other.

### Wind settings

Wind is set in `tmuxinator.multi-uav-trace-mbzirc-gt.yml` by

```bash
python3 set_sea_state.py --sea-state 2
```

Parameters can be modified according to [official Wiki](https://github.com/osrf/mbzirc/wiki/Configuring-environment-conditions).