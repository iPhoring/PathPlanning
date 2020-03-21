# Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
![image1](./images/pathplanner.png)
   

### Goals
Goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provide the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The autonomous car should drive as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.



#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Previous path data given to the Planner
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
Note: Return the previous list but with processed points removed, can be used to show how far along
the path has processed since last time.

#### Previous path's end s and d values 
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details
1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Accelerations both in the tangential and normal directions are measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually it's not very long, maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this it's a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Design
Path Planning module for autonomous vechile are composed of mainly 7 sub modules as listed below.
Behavior: The pupose of this sub module is to define a set of paths for the vehicle to follow ex. lane changes, slow dowm
Predictions: This module predict the trajectories of the surrounding objects based on sensor fusion data 
Trajectory: This module generates the best possible trajectory for the autonomous car to follow.
Sensor Fusion: This module is responsible for collection and data points from the sensors
Motion Control: Fin grain controls for moving the vehicle

![image2](./images/PathPlannerDesign.png)

---
### Behavior Planner:
The main target is to adjusting the speed of car to relate our lane and keeping a safe following distance( ex. 30 meters distance from the vehicle in front of us)

![image3](./images/behaviorDesign.png)

### Development 
The intents is to find the location of the car in referene to our car. 
a) Is the car ahead of us 
b) Is the a car left of us
c) Is the car right of us 
This will help us to decide is we need to reduce speed or it is safe to change the lane.
![image4](./images/prediction.png)

--

Once we have determined the object location wrt our car the below code find the speed at which the object/car is moving. This will help to make the decision.
![image5](./images/objectprediction.png )

--

Based on the speed and location the below code design the behavior of the car.
![image6](./images/behavior.png )

This part of the code generates the safe trajector that car should follow to reach the designation following speed limit and avoiding max jerck. 
![image7](./images/trajectory.png)

---
## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Code Style
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
