#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

class Point:
    def __init__(self, x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z


# global variables
## state variables
position_ = Point()
yaw_ = 0
## machine state
state_ = 0
## goal
desired_position_ = Point()
desired_position_.x = -3
desired_position_.y = -3
desired_position_.z = 0
## parameters
yaw_precision_ = math.pi / 90 # 2 degrees
dist_precision_ = 0.3
## publishers
pub = None

# auxiliar functions
def change_state(state):
    global state_
    state_ = state
    rospy.loginfo("State changed to: " + str(state))


# logical functions
def go_straight_ahead(desired_position):
    global position_, yaw_, pub

    desired_yaw = math.atan2(desired_position.y - position_.y, desired_position.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position.y - position_.y,2) + pow(desired_position.x - position_.x,2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.7
        pub.publish(twist_msg)
    else:
        rospy.logerr("Position error: " + str(err_pos))
        change_state(2)
    
    if math.fabs(err_yaw) > yaw_precision_:
        rospy.logerr("Yaw error: " + str(err_yaw))
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0

    pub.publish(twist_msg)

def fix_yam(desired_position):
    global yaw_, yaw_precision_, pub
    
    desired_yaw = math.atan2(desired_position.y - position_.y, desired_position.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        if err_yaw > 0:
            twist_msg.angular.z = -0.7
        else:
            twist_msg.angular.z = 0.7
    
    pub.publish(twist_msg)

    if math.fabs(err_yaw) <= yaw_precision_:
        rospy.loginfo('Yaw error: ' + str(err_yaw))
        change_state(1)

def callback_odom(msg):
    global position_, yaw_

    position_ = msg.pose.pose.position
    
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def main():
    global pub

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yam(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            pass
        else:   
            rospy.logerr('unknown state')
            pass
        rate.sleep()

if __name__ == "__main__":
    main()