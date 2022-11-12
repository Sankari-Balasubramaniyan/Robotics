#!/user/bin/env python


# running prior to starting this script
# Simulation :
# roscore
# roslaunch turtlebot3_gazebo turtlebot3_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

# IRL: 
# roscore
# roslaunch turtlebot3_bringup turtlebot3_robot.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
import rospy

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
global navigator


class GoToPose():

    def __init__(self):
        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 30 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
 
    def notice_gesture (self, gesture): 
    	
        if gesture == 'Geste1':
            position = {'x': 1.22, 'y' : 2.56}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        if gesture == 'Geste2':
            position = {'x': 1.22, 'y' : 2.56}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        if gesture == 'Geste3':
            position = {'x': 1.22, 'y' : 2.56}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        if gesture == 'Geste4':
            position = {'x': 1.22, 'y' : 2.56}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        if gesture == 'Geste5':
            position = {'x': 1.22, 'y' : 2.56}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}


        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = self.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)




#when new msg received, callback is invoked with the msg as the first argument
def callback(data):
    global navigator
    rospy.loginfo(rospy.get_caller_id() + "I heard gesture %s", data.data)
    navigator.notice_gesture(data.data)

    
def listener() :
    
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can run simultaneously.
   
   rospy.init_node('nav_test', anonymous=False)
   rospy.Subscriber("gesture", String, callback)
   
   # spin() simply keeps python from exiting until this node is stopped
   
   
if __name__=="__main__":
    listener()
    navigator = GoToPose()
    rospy.spin()
    
    
    
    
    

