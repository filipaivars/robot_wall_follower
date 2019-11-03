#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

pub = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'fix follow the wall - left',
    4: 'fix follow the wall - right'
}


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.3
    
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = -0.7

    return msg

def follow_wall():
    msg = Twist()
    msg.linear.x = 0.5

    return msg

def fix_follow_wall_left():
    # soft turn left to ensure maximum distance to wall
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.5

    return msg

def fix_follow_wall_right():
    # soft turn right to ensure minimum distance to wall
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.5

    return msg

def change_state(state):
    global state_

    if state is not state_:
        rospy.loginfo('Wall follower: ' + str(state) + ' - ' + str(state_dict_[state]))
        state_ = state

def take_action():
    global regions_

    regions = regions_

    d = 1
    d_min = 0.8
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['right'] > d:
        state_description = 'case 3 - fright'
        change_state(4)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] < d and regions['right'] > d_min:
        state_description = 'case 4.1 - right check'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] < d and regions['right'] < d_min:
        state_description = 'case 4.2 - right fix'
        change_state(3)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['right'] < d and regions['right'] > d_min:
        state_description = 'case 5.1 - fright right check'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['right'] < d and regions['right'] < d_min:
        state_description = 'case 5.2 - fright right fix'
        change_state(3)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 7 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 8 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 9 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 10 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    rospy.loginfo(state_description)


def callback_laser(msg):
    global regions_

    regions_ = {
        'right': min(min(msg.ranges[:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'left': min(min(msg.ranges[576:719]), 10)
    }

    take_action()

def main():
    global pub_, state_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('m2wr/laser/scan', LaserScan, callback_laser)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 3:
            msg = fix_follow_wall_left()
        elif state_ == 4:
            msg = fix_follow_wall_right()
        elif state_ == 2:
            msg = follow_wall()
            pass
        else:
            rospy.logerr('Unknown state')
            pass
        pub_.publish(msg)
        rate.sleep()
        

if __name__ == "__main__":
    main()