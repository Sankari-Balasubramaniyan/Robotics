
# In each terminal
cd ~/catkin_ws/
source devel/setup.sh

# First terminal
roscore

#Second terminal
cd ~/catkin_ws/
source devel/setup.sh
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

#Third terminal
cd ~/catkin_ws/
python3 PATH
