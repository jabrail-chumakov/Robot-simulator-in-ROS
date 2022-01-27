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
$ rosrun src regulator
$ rosrun src controller
```
Note that `src` above should be changed on the folder name, where all of downloaded files from repository are stored on your computer. For example `assigment2/src/controller.cpp`, where assigment2 is the name of your folder in your catkin workspace.


Task objectives
---------
```bash
- We can use ROS for controlling the robot
- The robot is endowed with laser scanners
- We want to have a node for the control of the robot, and an additional node which interacts with the user to: (1) increase/decrease the speed and (2) reset the robot’s position
- Everything should be done in cpp 
```

### Motors ###


Working principle of proposed solution
-----------------------------

The proposed solution is simple and efficient and works almost perfectly. The solution consists of 4 cases when the robot needs to complete a particular command.

### Case 1 ###

```If the robot is far away from the nearest golden token or angle to the nearest golden token is more than 90 degrees then the robot moves straight.```

### Case 2 ###

```If the robot detects a silver token on the distance closer than 1.00, then the robot aligns relatively to silver token and moves towards it.```

### Case 3 ###

```If the robot on a distance of 0.4 from the silver token, then the robot grabs it, rotates at 180 degrees, release this silver token and then the robot returns to its initial position where it was before grabbing.```

### Case 4 ###

``` If the robot on the distance of 0.7 from the golden token and golden angle is less than 90 degrees or more than -90 degrees, then three possible outcomes may occur:```

* `First outcome`: If the angle relative to the golden token is more or equal to 25 degrees, then the robot should turn left.
* `Second outcome`: If the angle relative to the golden token is less or equal to -25 degrees, then the robot should turn right.
* `Third outcome`: A special avoiding case is executed which checks where to turn depending on the closest golden token distance. Frequently it occurs when the robot drives to the corner of the road. In such a scenario robot turns to direction, where the distance to the closest golden box is larger in the range between 75 and 105 degrees.

Flowchart
-----------------------------
![flowchart](https://user-images.githubusercontent.com/67557966/141962747-017895b6-90b8-4a16-9245-ac5e221b2cce.jpg)

Video demonstration
-----------------------------

Below you can watch a demonstration of this assignment:

[<img src="https://user-images.githubusercontent.com/67557966/141862028-4c13ba55-2e97-415f-b7d4-bb2fb68122c1.jpg" width="80%">](https://www.youtube.com/watch?v=c6LJVWnKfDc)

Drawbacks and possible improvements
-----------------------------
### First assumptions ###
```While this solution works perfectly, on very rare occasions the robot may move in the clockwise direction. This is due to the fact that when a robot grabs and rotates the same silver token many times, the silver token may be placed right in the corner of the golden borders. After that when the robot turns, in its vision radius, the silver token will appear to be the closest available (since it will be in a radius of less than 1). However, this can be easily fixed by slightly reducing the view radius (angle) of the robot itself in the script.```

### Second assumptions ###
``` In another case, the robot can get to a sunken corner with golden tokens (imperfect corner where 1 golden token is missing). The script was executed for 1 hour, but in my case, the robot never collided into this area. Therefore, it's difficult to evaluate how the robot will behave in this scenario. However, it can be assumed that it all depends on the angle at which the robot will be to the nearest golden token or rely on the special avoiding case.```
