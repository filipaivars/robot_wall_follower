#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback_laser(msg):
    # 144 = 720 / 5
    regions = [
        min(min(msg.ranges[:143]), 10),
        min(min(msg.ranges[144:287]), 10),
        min(min(msg.ranges[288:421]), 10),
        min(min(msg.ranges[432:575]), 10),
        min(min(msg.ranges[576:719]), 10),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, callback_laser)

    rospy.spin()

if __name__ == "__main__":
    main()