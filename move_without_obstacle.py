import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time 

# goal
desired_position = Point()
desired_position.x = 0
desired_position.y = 7
desired_position.z = 0

position = Point()
orientation = 0

precision = math.pi / 90  #orientation precision

status_robot = {'correct_angle' : False, 'correct_movement' : True}

pub = None

def callback_odom(msg):
    global position
    global orientation
    
    position = msg.pose.pose.position
    
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        
    orientation = transformations.euler_from_quaternion(quaternion)[2] #Get the Yaw of euler angle
 
def callback_laser(msg):
    
    #Ignore Obstacles at a distance 10m from the robot
    
    global right
    right =  min(min(msg.ranges[270:310]), 10)
    global front
    front = min(min(msg.ranges[0:20]+msg.ranges[340:359]), 10)
    global left
    left =   min(min(msg.ranges[50:90]), 10)
      
    
def avoid_obstacle(right,front,left):
    global msg 
    msg = Twist()
    
    status = ''
    
    if front > 0.5 and left > 0.5 and right > 0.5:
        status = 'NO OBSTACLE'
        msg.linear.x = 0.6
        msg.angular.z = 0.0
        pub.publish(msg)

    elif front < 0.5 and left > 0.5 and right > 0.5:
        status = 'OBSTACLE IN THE FRONT'
        rospy.loginfo("Turn left")
        move_left()
        move_forward()
       
    elif front > 0.5 and left > 0.5 and right < 0.5:
        status = 'OBSTACLE IN THE RIGHT'
        rospy.loginfo("Turn left")
        move_left()
        move_forward()
        
    elif front > 0.5 and left < 0.5 and right > 0.5:
        status = 'OBSTACLE IN THE LEFT'
        rospy.loginfo("Turn right")
        move_right()
        move_forward()
        
    elif front < 0.5 and left > 0.5 and right < 0.5:
        status = 'OBSTACLE IN THE FRONT AND RIGHT'
        rospy.loginfo("Turn left")
        move_left()
        move_forward()
        
    elif front < 0.5 and left < 0.5 and right > 0.5:
        status = 'OBSTACLE IN THE FRONT AND LEFT'
        rospy.loginfo("Turn right")
        move_right()
        move_forward()
        
    elif front < 0.5 and left < 0.5 and right < 0.5:
        status = 'OBSTACLE IN THE FRONT, LEFT AND RIGHT'
        rospy.spin()
        
    else:
        rospy.loginfo("Unknown obstacle")


def move_forward():
    for i in range(0,40):
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        rate = rospy.Rate(5)
        pub.publish(msg)
        rate.sleep()
    
def move_right():
    for i in range(0,90):
        right_turn()
        pub.publish(msg)
        rate = rospy.Rate(5)
        rate.sleep()
        
def right_turn():
     msg.linear.x = 0.0
     msg.angular.z = -0.1
     
def move_left():
    for i in range(0,90):
        left_turn()
        pub.publish(msg)
        rate = rospy.Rate(5)
        rate.sleep()
        
def left_turn():
    msg.linear.x = 0.0
    msg.angular.z = 0.1
    
    
def fix_orientation(pos):

    global orientation, status_robot, pub, precision
    msg = Twist()
    #pos - desired position
    #position - current position
    #desired_orien = gets the desired angle
    desired_orien = math.atan2(pos.y - position.y, pos.x - position.x)
    error = desired_orien - orientation
    
    #Accepting an error of 0.2 in orientation
    if math.fabs(error) > precision:
        if error > 0:                
            msg.angular.z = 0.7         #adjust towards left
        else:
            msg.angular.z = -0.7        #adjust towards right

    
    pub.publish(msg)

    #Angle adjust, apply linear motion
    if math.fabs(error) <= precision:
        status_robot = {'correct_angle' : True, 'correct_movement' : False}
        
        
def move_straight(pos):
    
    global right, left, front, status_robot, error, orientation
    #length to reach the goal point
    length_to_cover = math.sqrt(pow(pos.y - position.y, 2) + pow(pos.x - position.x, 2))
    
    #Accepting 0.3m error between goal and current bot position
    rospy.loginfo(length_to_cover)
    if length_to_cover > 0.3:
        print('Length to cover to reach goal : [%s]' % length_to_cover)
        avoid_obstacle(right, front, left) #check for obstacle in right, left, front
    else:
        #goal reached
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        pub.publish(msg)
        rospy.loginfo("****Hurray we have reached the destination****")
        return
        
    desired_orien = math.atan2(pos.y - position.y, pos.x - position.x)
    error = desired_orien - orientation 
     
    #change orientation to align to reach the goal
    if math.fabs(error) > precision:
        status_robot = {'correct_angle' : False, 'correct_movement' : True}
        
    

def main():

    global pub
    
    rospy.init_node('move_without_obstacle')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
    sub_laser = rospy.Subscriber('/scan', LaserScan, callback_laser)
    
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        if status_robot['correct_angle'] == False and status_robot['correct_movement'] == True:
            fix_orientation(desired_position)
        elif status_robot['correct_angle'] == True and status_robot['correct_movement'] == False:
            move_straight(desired_position)
        else:
            rospy.loginfo("Error")
            pass
        rate.sleep()
    
if __name__ == '__main__':
    main()
