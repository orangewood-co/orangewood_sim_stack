## Class `OwlSimClient`

Class acting as a client interface to control a MoveIt! based robot through ROS. It handles initializing,
controlling, and interacting with the robot within a simulated environment. This includes managing movement,
adding obstacles, and handling the robot's gripper.

### Methods

#### Method `__init__`

Initializes the robot environment, including the MoveIt! commander, ROS node, and publishers.
Sets up arm and gripper groups for robot manipulation.

**Parameters:**
- `arm_group_name` (default='arm'): Name of the arm move group.
- `gripper_group_name` (default='gripper'): Name of the gripper move group.
- `wait_for_servers` (default=10): Time to wait for move group servers to be available.
- `gripper_enable` (default=True): Flag to decide whether to enable the gripper or not.

#### Method `__repr__`

Provides a string representation of the client's current state.

**Parameters:**
- None

#### Method `__str__`

Provides a simple string representation.

**Parameters:**
- None

#### Method `__enter__`

Handles context management entry.

**Parameters:**
- None

#### Method `__exit__`

Handles context management exit.

**Parameters:**
- `exc_type` (default=None)
- `exc_value` (default=None)
- `traceback` (default=None)

#### Method `is_running`

Checks if the robot's move group is currently available.

**Returns:**
- `bool`: True if the move group is available, False otherwise.


#### Method `get_version`

Returns the current version of the robot software.

**Parameters:**
- None

#### Method `get_joint`

Returns the current joint values of the robot.

**Parameters:**
- None

#### Method `get_tcp`

Returns the current TCP values of the robot.

**Parameters:**
- None

#### Method `get_tcp_orientation`

Returns the current TCP orientation in Euler or Quaternion values.

**Parameters:**
- `mode` (default='quat'): Specify 'euler' for Euler angles or 'quat' for Quaternion.

#### Method `get_tcp_position`

Returns the current TCP position (x, y, z).

**Parameters:**
- None

#### Method `get_version`

Returns the current version of the robot software.

**Parameters:**
- None

#### Method `get_joint`

Returns the current joint values of the robot.

**Parameters:**
- None

#### Method `get_tcp`

Returns the current TCP values of the robot.

**Parameters:**
- None

#### Method `get_tcp_orientation`

Returns the current TCP orientation in Euler or Quaternion values.

**Parameters:**
- `mode` (default='quat'): Specify 'euler' for Euler angles or 'quat' for Quaternion.

#### Method `get_tcp_position`

Returns the current TCP position (x, y, z).

**Parameters:**
- None

#### Method `add_obstacle`

Adds an obstacle to the planning scene based on the specified parameters.

**Parameters:**
- `object_type` (str): Type of the obstacle ('plane' or 'sphere').
- `name` (str): Identifier for the obstacle.
- `pose` (list): Position and orientation of the obstacle.
- `frame_id` (str): The reference frame for the obstacle's position.

**Returns:**
- `bool`: True if the obstacle was successfully added, False otherwise.

#### Method `get_obstacles_list`

Retrieves a list of all obstacles currently present in the planning scene.

**Returns:**
- `list`: A list of obstacle names.

#### Method `remove_obstacle`

Removes a specified obstacle from the planning scene.

**Parameters:**
- `name` (str, optional): Name of the obstacle to remove. Defaults to None.

#### Method `set_home`

Sets the robot's position to a predefined home position.

**Parameters:**
- `pose` (str): The name of the home position.
- `wait` (bool): Whether to wait for the move to complete.

#### Method `set_gripper`

Configures the gripper to a specific state or value.

**Parameters:**
- `goal_state` (str): Target state for the gripper ('open' or 'close').
- `goal_value` (int): Target value for the gripper position.
- `mode` (str): Mode of setting the gripper ('state' or 'value').
- `wait` (bool): Whether to wait for the move to complete.


#### Method `get_gripper`

Get the current gripper state

**Parameters:**

**Returns:**
- `list`: A list of joint values.