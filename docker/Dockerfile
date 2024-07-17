# Use Ubuntu 20.04 latest for building RoboGPT

FROM osrf/ros:noetic-desktop-full

RUN apt update  
RUN apt -y upgrade

ENV DEBIAN_FRONTEND=noninteractive


#Copying RoboGPT dependencies, real sense dependecies and python module dependecies and install
COPY install_main_dependencies.sh /root

#Install General RoboGPT dependencies
RUN sh /root/install_main_dependencies.sh

COPY requirements.txt /root
#Install Python modules for RoboGPT
RUN pip install -r /root/requirements.txt

#RUN rosdep init
RUN rosdep update


COPY sim_entrypoint.sh /sim_entrypoint.sh
RUN chmod +x /sim_entrypoint.sh
ENTRYPOINT ["/sim_entrypoint.sh"]

