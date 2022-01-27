Robot Simulator in ROS
================================

This is a simple robot simulator in ROS, which should avoid collision with the borders. It was done as a part of Research Track I course and written in C++.

Installing
----------------------

The simulator required a World map, which was provided by professor Carmine Recchiuto in the following [repository](https://github.com/CarmineD8/second_assignment). Besides that, user should type 'catkin_make' being in catkin workspace.

Exercise
-----------------------------

To run script in the simulator, use `run.py`, passing it the file names. 

You can run `assignment.py` file by running following command in your folder:

```bash
$ python run.py assignment.py
```

In case if you will place `assignment.py` file in another folder, you should run following command:

```bash
$ python run.py path_to_your_folder/assignment.py
```

Robot API
---------

The API for controlling a simulated robot is designed to be as similar as possible to the [SR API][sr-api].

### Motors ###

The simulated robot has two motors configured for skid steering, connected to a two-output [Motor Board](https://studentrobotics.org/docs/kit/motor_board). The left motor is connected to output `0` and the right motor to output `1`.

The Motor Board API is identical to [that of the SR API](https://studentrobotics.org/docs/programming/sr/motors/), except that motor boards cannot be addressed by serial number. So, to turn on the spot at one quarter of full power, one might write the following:

```python
R.motors[0].m0.power = 25
R.motors[0].m1.power = -25
```

### The Grabber ###

The robot is equipped with a grabber, capable of picking up a token which is in front of the robot and within 0.4 metres of the robot's centre. To pick up a token, call the `R.grab` method:

```python
success = R.grab()
```

The `R.grab` function returns `True` if a token was successfully picked up, or `False` otherwise. If the robot is already holding a token, it will throw an `AlreadyHoldingSomethingException`.

To drop the token, call the `R.release` method.

Cable-tie flails are not implemented.

### Vision ###

To help the robot find tokens and navigate, each token has markers stuck to it, as does each wall. The `R.see` method returns a list of all the markers the robot can see, as `Marker` objects. The robot can only see markers which it is facing towards.

Each `Marker` object has the following attributes:

* `info`: a `MarkerInfo` object describing the marker itself. Has the following attributes:
  * `code`: the numeric code of the marker.
  * `marker_type`: the type of object the marker is attached to (either `MARKER_TOKEN_GOLD`, `MARKER_TOKEN_SILVER` or `MARKER_ARENA`).
  * `offset`: offset of the numeric code of the marker from the lowest numbered marker of its type. For example, token number 3 has the code 43, but offset 3.
  * `size`: the size that the marker would be in the real game, for compatibility with the SR API.
* `centre`: the location of the marker in polar coordinates, as a `PolarCoord` object. Has the following attributes:
  * `length`: the distance from the centre of the robot to the object (in metres).
  * `rot_y`: rotation about the Y axis in degrees.
* `dist`: an alias for `centre.length`
* `res`: the value of the `res` parameter of `R.see`, for compatibility with the SR API.
* `rot_y`: an alias for `centre.rot_y`
* `timestamp`: the time at which the marker was seen (when `R.see` was called).

For example, the following code lists all of the markers the robot can see:

```python
markers = R.see()
print "I can see", len(markers), "markers:"

for m in markers:
    if m.info.marker_type in (MARKER_TOKEN_GOLD, MARKER_TOKEN_SILVER):
        print " - Token {0} is {1} metres away".format( m.info.offset, m.dist )
    elif m.info.marker_type == MARKER_ARENA:
        print " - Arena marker {0} is {1} metres away".format( m.info.offset, m.dist )
```

[sr-api]: https://studentrobotics.org/docs/programming/sr/

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
