#!/usr/bin/env bash

#Image name, default orangewood_sim_image
# If you download from docker hub, it will be: orangewoodlabs/orangewood_sim
docker_image="$1"
echo "Docker Image is $1"
xhost +local:docker

echo "Checking for NVIDIA GPU and driver..."

# Function to check for NVIDIA GPU and driver
check_nvidia_driver() {
    if lspci | grep -i nvidia > /dev/null; then
        if nvidia-smi > /dev/null 2>&1; then
            echo "NVIDIA GPU and driver are properly installed."
            return 0
        else
            echo "NVIDIA GPU found, but driver is not installed or not working properly."
            return 1
        fi
    else
        echo "NVIDIA GPU not found."
        return 1
    fi
}

# Build folder name
orangewood_ros_ws="orangewood_ws"

# Run Docker with NVIDIA Runtime
run_docker_nvidia() {
    echo "Enable NVIDIA Runtime"
    docker run -it \
        --rm \
        --name="orangewood_sim" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --runtime=nvidia --gpus all \
        --workdir="/home/$USER/$orangewood_ros_ws" \
        --device="/dev/video0" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/home/$USER:/home/$USER" \
        $docker_image \
        bash
}

# Run Docker without NVIDIA
run_docker() {
    echo "Enable Non-NVIDIA Runtime"
    docker run -it \
        --rm \
        --name="orangewood_sim" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --workdir="/home/$USER" \
        --device="/dev/video0" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/home/$USER:/home/$USER" \
        $docker_image \
        bash
}

# Check for NVIDIA driver and call docker run based on it
if check_nvidia_driver; then
    run_docker_nvidia
else
    run_docker
fi
