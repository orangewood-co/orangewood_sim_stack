#!/usr/bin/env bash

# Installation Reference website: https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04

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

# Function to install Docker
install_docker() {
    echo "Installing Docker on the Host Machine"
    sudo apt update
    sudo apt install -y apt-transport-https ca-certificates curl software-properties-common
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
    apt-cache policy docker-ce
    sudo apt install -y docker-ce git openssh-server openssh-client
    sudo usermod -aG docker ${USER}
    sudo usermod -aG docker ${USER}
    sudo chmod 666 /var/run/docker.sock
    sudo systemctl restart docker

    echo "Docker Installation finished"
}

# Function to install NVIDIA Container Toolkit
install_nvidia_container_toolkit() {
    echo "Setting up NVIDIA Container Toolkit, make sure you have installed NVIDIA Graphics Driver"
    sudo apt install -y curl
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list

    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
}

# Check for NVIDIA driver and call the appropriate functions
if check_nvidia_driver; then
    install_docker
    install_nvidia_container_toolkit
else
    install_docker
fi

