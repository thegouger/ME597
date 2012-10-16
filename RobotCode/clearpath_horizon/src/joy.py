#!/usr/bin/python


# ROS stuff
import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from geometry_msgs.msg import Twist
from clearpath_horizon.msg import Joy, Announce

import os

rospy.init_node('horizon_joy', anonymous=True)

turn_scale = rospy.get_param('~turn_scale')
drive_scale = rospy.get_param('~drive_scale')

cmd_pub = rospy.Publisher('cmd_vel', Twist, latch=False)

announce_pub = rospy.Publisher('/clearpath/announce', Announce, latch=True)
announce_pub.publish(action="new_joy", topic=os.getenv("ROS_NAMESPACE", ""));

UPDATE_HZ = 20

joy = None

def joy_callback(data):
    global joy
    joy = data

    cmd = Twist()
    cmd.linear.x = data.axes[1] * drive_scale
    cmd.angular.z = data.axes[0] * turn_scale
    cmd_pub.publish(cmd)


rospy.Subscriber("joy", Joy, joy_callback)

rate = rospy.Rate(UPDATE_HZ)

while not rospy.is_shutdown():
    rate.sleep()
    if not joy:
        continue

    cmd = Twist()
    cmd.linear.x = joy.axes[1] * drive_scale
    cmd.angular.z = joy.axes[0] * turn_scale
    cmd_pub.publish(cmd)

