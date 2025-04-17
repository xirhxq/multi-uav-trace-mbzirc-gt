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