#!/usr/bin/env python3

"""
Python example to demonstrate Orangewood SDK APIs
"""
import time
import sys
import signal

from owl_robot_sdk import OwlSimClient

#MoveIt Config Group name

class TestOwlSimClient:
    def __init__(self,group_name,gripper_group):
        self.group_name = group_name

        signal.signal(signal.SIGINT, self.signal_handler)

        print("Application is running. Press Ctrl+C to exit.")

        self.client = OwlSimClient(self.group_name,gripper_group,5,True)



        self.get_sdk_api()
        self.set_sdk_api()



    def signal_handler(self,sig, frame):
        print("Interrupt received, closing application.")
        sys.exit(0)

    #Testing Get APIs of SDK
    def get_sdk_api(self):

        #Robot Running Status
        status = self.client.is_running()
        print ("Robot Running Status:= ",status)

        #Robot Version
        version = self.client.get_version()
        print ("Robot Motion Group:= ", version)


        #Get Value of TCP 
        tcp_pose = self.client.get_tcp()
        print ("Robot TCP position:= ", tcp_pose)


        #Get Value of Joints
        joint_val = self.client.get_joint()
        print ("Robot Joint position:= ", joint_val)


        #Get TCP Position
        tcp_position = self.client.get_tcp_position()
        print ("Robot TCP Position: ", tcp_position)


        #Get TCP Orientation Quaternion
        tcp_orient_quat = self.client.get_tcp_orientation("quat")
        print("Robot Orientation Quaternion:= ", tcp_orient_quat)


        #Get TCP Orientation Euler angle
        tcp_orient_euler = self.client.get_tcp_orientation("euler")
        print("Robot Orientation in Euler:= ", tcp_orient_euler)


    #Set SDK APIs
    def set_sdk_api(self):

        print("Setting Velocity Fraction")
        status = self.client.change_speed_fraction(0.5)
        #Setting to home
        time.sleep(5)
        print ("Setting Home")
        self.client.set_home()
        time.sleep(5)

        print ("Testing Robot move to pose ")
        tcp_position = self.client.get_tcp()
        tcp_position_goal = tcp_position

        tcp_position_goal[0] = tcp_position_goal[0] + 0.2  #Z
        tcp_position_goal[2] = tcp_position_goal[2] - 0.2  #Z

        status=self.client.move_to_pose(tcp_position_goal)
        print ("Robot move to pose:= ",status)
        time.sleep(5)
        self.client.set_home()
        time.sleep(5)

        ######################
        print("Testing Robot Move to Joint")
        current_joint_pose = self.client.get_joint()
        current_joint_pose_goal = current_joint_pose

        current_joint_pose_goal[0] += 0.3
        current_joint_pose_goal[1] += 0.1

        status=self.client.move_to_joint(current_joint_pose)
        time.sleep(5)
        self.client.set_home()
        time.sleep(5)

        #################################
        print ("Robot moving in Translate Mode")

        status = self.client.move_translate(0.2, 0.0, -0.3)
        time.sleep(5)
        print ("Setting Home")
        self.client.set_home()
        time.sleep(5)

        ###########################################
        print ("Robot Moving in waypoint translate mode")
        trajectory = [[0.1,0.1,-0.2],[-0.1,0.1,-0.2],[0.1,0.1,-0.1],[0.2,-0.1,0.2]]

        status = self.client.move_trajectory(trajectory)
        time.sleep(5)
        self.client.set_home()
        time.sleep(5)

        ###########################################
        print ("Robot Moving in up and down")
        self.client.move_down()
        time.sleep(5)
        self.client.move_up()
        time.sleep(5)
        self.client.set_home()
        time.sleep(5)

        ###########################################
        print ("Adding Obstacles to Scene")
        pose = [2,2,2, 0,0,0]

        self.client.add_obstacle("box1",1,1,1,1,pose,"box")
        self.client.add_obstacle("cone1",1,1,1,1,[5,5,5,0,0,0],"cone")
        self.client.add_obstacle("plane1")
        self.client.add_obstacle("mesh1",1,1,1,1,[1,1,1,0,0,0],"mesh","arm.stl")

        print("Get Obstacle List")
        obstacle_list=self.client.get_obstacles_list()
        print("List of Obstacle ", obstacle_list)
        time.sleep(5)

        print("Remove Obstacles")
        self.client.remove_obstacle()

        ######################################################

        print ("Testing Gripper Control with states")
        self.client.set_gripper("open",1,"state")
        get_gripper = self.client.get_gripper_val()
        print(get_gripper)
        time.sleep(2)
        self.client.set_gripper("close",0,"state")
        get_gripper = self.client.get_gripper_val()
        print(get_gripper)

        time.sleep(2)
        print ("Testing Gripper Control with value")
        self.client.set_gripper("close",0.799,"value")

        get_gripper = self.client.get_gripper_val()
        print(get_gripper)

        time.sleep(2)
        self.client.set_gripper("open",0.0,"value")
        get_gripper = self.client.get_gripper_val()
        print(get_gripper)

        time.sleep(2)
        self.client.close()

if __name__ == "__main__":
    testowl = TestOwlSimClient("arm","gripper")


