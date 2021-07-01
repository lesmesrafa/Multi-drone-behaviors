# quadrotor_motion_with_pid_control
## behavior_follow_path

**Purpose**: This behavior follows a path while using the path tracker. This behavior does not finish until the robot reach the last point. The argument is a tuple of coordinates.

**Type of behavior:** Goal-based behavior.

**Arguments:** 

| Name    |   Format  |  Example |  
| :-----------| :---------| :--------|
| path: |Tuple of coordinates x, y, z (meters)|path: [[1.23, 2, 0.5],[2.5,3,1],[3.1,3.4,1.5]]|  

### Diagram

![Follow path](https://i.ibb.co/D72vN3S/follow-path-diagram.png)
----
## behavior_keep_hovering_with_pid_control

**Purpose:** The robot keeps hovering. Hovering is a maneuver in which the robot is maintained in nearly motionless flight over a reference point at a constant altitude and on a constant heading. This behavior does not avoid moving obstacles. 

**Type of behavior:** Recurrent behavior.

### Diagram

![Keep hovering](https://i.ibb.co/0mW6GwZ/keep-hovering-diagram.png)
----
## behavior_keep_moving_with_pid_control

**Purpose:** The robot keeps moving at a constant speed in some direction (forward, backward, left,right). If the speed value is not given a default value is considered. This behavior does not avoid obstacles. 

**Type of behavior:** Recurrent behavior.

| Arguments    |   Format  |  Example |  
| :-----------| :---------| :--------|
| direction |{FORWARD, BACKWARD, LEFT, RIGHT}|direction: FORWARD |          
| speed |number (m/sec)|speed: 12|

### Diagram

![Keep Moving](https://i.ibb.co/42zsSrs/keep-moving-diagram.png)
----
## behavior_rotate_with_pid_control

**Purpose:** The robot rotates left or right a certain number of degrees (angle) on the vertical axis (yaw). The number of degrees can be expressed as an absolute value (argument “angle”) or relative to the robot (argument “relative_angle”).

**Type of behavior:** Goal-based behavior.

| Arguments    |   Format  |  Example |  
| :-----------| :---------| :--------|          
| angle |number (degrees)|angle: 90|
| relative_angle |number (degrees)|relative_angle: 90|

### Diagram

![Rotate](https://i.ibb.co/3zXfJ3p/rotate-diagram.png)
----


# Contributors

**Code maintainer:** Alberto Rodelgo Perales

**Authors:** Alberto Rodelgo Perales