How to run the Program:

Part1 - implementation and visuallization of non-holonomic
robot in python

Just run astar.py with default values to get the result as the video attached

Part2 - Implementation in Ros
unzip package in side your catkin_ws/src
(refer to the video for clarity)
catkin build
source devel/setup.bash
roslaunch searchbot astar.launch pos:=1 y_pos:=3 yaw:=0
1
(enter the desired inputs)
Default vaules for video
NOTE that maze size is scaled to be 10 x 10
clearance - .1
start - (-1,-3)
orientation - 0
goal - (-4,-4)
(program will generate action set for robot to follow)

NOTE- Since this done via an open loop it is really hard for the
robot to accurately follow directions and the time length has
to b tweaked for each scenario
Same code can't be used for different start and goal co-ordinates
as the action set generated will be right but the path following
needs some sort of controller implementation


