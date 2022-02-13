Robot Simulator in ROS
================================

This is a simple robot simulator in ROS, which should avoid collision with the borders. It was done as a part of Research Track I course and written in C++.

Installing
----------------------

The simulator required a World map, which was provided by professor Carmine Recchiuto in the following [repository](https://github.com/CarmineD8/second_assignment). Besides that, user should type `catkin_make` being in catkin workspace.

Exercise
-----------------------------

To run script in the simulator, use `roscore` or `roscore&` (to run it in background) in your terminal at first.
After that, you should run files from downloaded repository:

```bash
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
$ rosrun second_assignment regulator
$ rosrun second_assignment controller
```
Note that `second_assignment` above should be changed on your folder name, where all of downloaded files from this repository are stored on your computer. For example `catkin_ws/src/second_assignment/src/controller.cpp`, where second_assignment is the name of your folder in your catkin workspace.


Task objectives
---------
### Requirements ###

- We can use ROS for controlling the robot
- The robot is endowed with laser scanners
- We want to have a node for the control of the robot, and an additional node which interacts with the user to: `increase/decrease the speed` and `reset the robot’s position`
- Everything should be done in cpp 

### Controlling the robot ###

- You should publish a velocity on the cmd_vel topic
- You can use the /base_scan topic to get information about the surrounding environment.
- In particular, the information about the distance of the obstacles is in the ranges vector.

Hint: divide the ranges vector into subsections, and take the minimum value of each subsection. This will provide
information about the closest obstacles. 

### Additional specifications ###

- The node controlling the robot should also implement the functionality for increasing/decreasing the speed
- The UI node should constantly wait for an input for the user, which can either ask to increment or decrement the velocity, or to put the robot in the initial position.
- The robot may crash if the speed is increased too much. 

Working principle of proposed solution
-----------------------------

The proposed solution is simple and efficient and works almost perfectly. The solution consists of several cases when the robot needs to complete a particular command.

### Control window ###

- UP arrow key = move on 0.25 faster.
- DOWN arrow key = move on 0.25 slower.
- R or r buttons = place robot on initial position.
- Q or q buttons = close program.
- Other buttons are not recognizible

### Controller file ###

- We update our speed each time, meaning that our `new_speed` is equal to `old_speed + 0.25`. Besides that, we compare our `new_speed` with `old_speed` each time, in order to prevent it from going in reverse direction. For example if `new_speed` is negative, then it should stop, meaning that `new_speed` is equal to zero.
- Controller files calculates distance in 3 directions: in `front` of him, in `right` and `left` in order to calculate collision risk. Since robot has vision of 360 degress following restrictions was introduced. `Front` side is angle between `45 and `135`, `Left` side is angle between `0 and 45` and `Right` side is angle between `135 and 180`.
- Taking into account above restrictions, if robot is too close to the borders `threshold = 0.7` and osbtacle is from the `left`, then it should turn to `right` direction. Similarly, if obstacle is from the `right` side, then it should turn to `left`.

### Regulator file ###

- This file is responbile for input commands. It uses `ncurses.h`, which allows to use arrow keys without waiting of pressing `Enter` from the user. Right after user presses `Up arrow key` or `Down arrow key`, robot will react and move with appropriate speed. The same thing is with `Q - quit` and `R - reset`. 
- It publishes results of successful commands. If robot increases/decreases a speed, then it publishes it in the terminal.

Flowchart
-----------------------------
![frr drawio](https://user-images.githubusercontent.com/67557966/151433368-563f0cc9-dc46-4a5e-a2d3-be628b49247f.png)

Possible improvements
-----------------------------
- In case if you will give to much acceleration to the robot and press `Reset` after that, acceleration will remain the same. For the future improvement, it will be better to reset acceleration as well, so robot will be placed at initial position with zero velocity and acceleration.
- Pause feature can be implemeneted as well in case user wants to pause the process.
