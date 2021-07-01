# Path tracker

The ROS node "path_tracker" implements an algorithm for path tracking for autonomous robots that operates in the following way:

- The node receives as input data a path to track as a sequence of point values (<x, y, z, yaw>).

- As a result, the controller generates a periodic output for a motion controller with reference speed <dx, dy, dz> and reference <yaw>

This specification partially follows the recommendations established in [REP 147](http://www.ros.org/reps/rep-0147.html#rep105) for subscribed topics.

# Subscribed topics

- **motion_reference/path** ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))       
Path reference for the controller. The following values from the messages are used <x, y, z, yaw> (the values for <pitch, roll> are discarded)

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

# Published topics

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller. The following values from the message are generated <d_x, d_y, d_z> (the other fields <d_yaw, d_pitch, d_roll> are generated with zero values)

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) 
Pose reference for the controller. Only the value for <yaw> is generated (the other fields <x, y, z, d_pitch, d_roll> are generated with zero values).

# Parameters

- **frequency** (double).        
The frequency of the path tracker in Hertz.

# Configuration file

The controller uses a configuration file in YAML format called `path_tracker.yaml`. This is an example of the content of the configuration file:

        vxy_max:              0.30
        vz_max:               0.15
        precision:            0.25
        path_facing:          true

# Implementation notes

The path tracker uses a state machine with three possible states: (1) MOVING_STRAIGHT (the vehicle is flying straight to the next consecutive point), (2) MOVING_AROUND (the vehicle moves around an intermediate point; this avoids stopping at an intermediate point), (3) ENDING_MOVEMENT (the vehicle reaches the last point of the trajectory to stop). Note that this does not control yaw (which is done in parallel by the PID controller). To control the movement, speed references are dynamically adjusted depending on the current state in the following way:

- State "MOVING_STRAIGHT": Speed reference is established to follow a straight line to reach the next point
- State "MOVING_AROUND": Speed reference is established to move around an intermediate point with a prefixed radius (precision)
- State "ENDING_MOVEMENT": Speed reference is defined to reach the final point and stop

The trajectory to follow is stored in an array of points and an index (called "reference_point_index") is used to indicate what is the next point to reach. Transitions between states are decided based on the distance between the current position of the vehicle and the points of the trajectory. For example, when the vehicle is far from the next point, the state is MOVING_STRAIGHT and when the vehicle is near the next point and it is necessary to modify the direction, the state is MOVING_AROUND.

----
# Contributors

**Code maintainer:** Alberto Rodelgo

**Authors:** Pablo Santofimia (preliminary implementation), Alberto Rodelgo (programming and testing), Martin Molina (specification as a standard process)
