#!/usr/bin/env bash



#Building image from RoboGPT Dockerfile
build=1  #With cache

#Build folder name
build_folder_name="workspaces"


echo "Enter Mode of Docker build, [0: with cache, 1: without cache, default:0]"
read build

#Checking input is empty or not, if is empty, build=0
if [ -z "$build" ]  
then
    build=0
    fi


if [ $build -eq 0 ]
    then
        echo "Building image with cache"
        docker build -t orangewood_sim_image .
else
    echo "Building image with no cache, it will take longer time"
    docker build --no-cache -t orangewood_sim_image .
fi

