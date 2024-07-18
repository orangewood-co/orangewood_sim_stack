# API Documentation

## Class `OwlSimClient`

Class acting as a client interface to control a MoveIt! based robot through ROS. It handles initializing, controlling, and interacting with the robot within a simulated environment. This includes managing movement, adding obstacles, and handling the robot's gripper.

### Method `__init__`

Initializes the robot environment, including the MoveIt! commander, ROS node, and publishers. Sets up arm and gripper groups for robot manipulation.

#### Arguments
- `arm_group_name` (str): Name of the arm move group.
- `gripper_group_name` (str): Name of the gripper move group.
- `wait_for_servers` (int): Time to wait for move group servers to be available.
- `gripper_enable` (bool): Flag to decide gripper to enable or not.

### Method `__repr__`

Return the current joint values of robot.

### Method `get_tcp`

Return the current TCP values of robot.

### Method `get_tcp_orientation`

Return the current TCP orientation in euler or quaternion values.

- `mode`: euler
- `mode`: quat

### Method `get_tcp_position`

Return the current TCP position x, y, z.

### Method `move_to_pose`

Request robot server to move to goal pose with desired tool speed in cartesian space.

#### Parameters
- `goalPose` (Pose): Goal pose [x, y, z, rx, ry, rz] robot need to achieve with desired tool speed.
- `toolSpeed` (float): Tool speed with which robot need to move.
- `wait` (bool): True will make the move call synchronous and wait till move is completed.
- `relative` (bool): Move relative to current robot pose.
- `moveType` (int): Type of move plan need to generate for move.

### Method `move_to_joint`

Request robot server to move to goal joint with desired tool speed in joint space.

#### Parameters
- `goalPose` (Joint): Goal joint robot need to achieve with desired tool speed in radians.
- `toolSpeed` (float): Tool speed with which robot need to move.
- `wait` (bool): True will make the move call synchronous and wait till move is completed.
- `relative` (bool): Move relative to current robot pose.

### Method `move_stop`

Request MoveIt to stop the current move.

### Method `move_pause`

Request MoveIt to pause the current move.

### Method `move_resume`

Request MoveIt to resume the paused move.

### Method `move_abort`

Request MoveIt to abort the current move.

### Method `change_speed_fraction`

Change the speed fraction setting for move [0 -> 1].

### Method `enable`

Request to enable the robot. This command is admin privileged, an access code is required.

#### Parameters
- `access_code` (str)

### Method `disable`

Request to disable the robot. This command is admin privileged, an access code is required.

#### Parameters
- `access_code` (str)

### Method `close`

Adds an obstacle to the planning scene based on the specified parameters.

#### Arguments
- `object_type` (str): Type of the obstacle ('plane' or 'sphere').
- `name` (str): Identifier for the obstacle.
- `pose` (list): Position and orientation of the obstacle.
- `frame_id` (str): The reference frame for the obstacle's position.

#### Returns
- `bool`: True if the obstacle was successfully added, False otherwise.

### Method `get_obstacles_list`

Retrieves a list of all obstacles currently present in the planning scene.

#### Returns
- `list`: A list of obstacle names.

### Method `remove_obstacle`

Removes a specified obstacle from the planning scene.

#### Arguments
- `name` (str, optional): Name of the obstacle to remove. Defaults to None.

### Method `set_home`

Sets the robot's position to a predefined home position.

#### Arguments
- `pose` (str): The name of the home position.
- `wait` (bool): Whether to wait for the move to complete.

### Method `set_gripper`

Configures the gripper to a specific state or value.

#### Arguments
- `goal_state` (str): Target state for the gripper ('open' or 'close').
- `goal_value` (int): Target value for the gripper position.
- `mode` (str): Mode of setting the gripper ('state' or 'value').
- `wait` (bool): Whether to wait for the move to complete.
