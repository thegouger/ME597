#!/usr/bin/python


# ROS stuff
import roslib; roslib.load_manifest('clearpath_horizon')
import rospy

from clearpath_horizon.msg import Announce
from geometry_msgs.msg import Twist, Vector3


# Required Clearpath Modules
from clearpath.horizon import Horizon 
from clearpath.horizon import transports   
import data


# Standard 
import os, sys, getopt, math, logging


# setup verbosity
logger = logging.getLogger('clearpath.horizon')
while len(logger.handlers) > 0: logger.removeHandler(logger.handlers[0])
logger_handler = logging.StreamHandler()
logger_handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
logger.addHandler(logger_handler)

# log level
logger.setLevel(logging.INFO)

# exact serial data
transports.logger.propagate = True
transports.logger.setLevel(logging.WARNING)  


rospy.init_node('horizon_robot')


# Instantiate Horizon
PORT = rospy.get_param('~port', '')

if PORT != '':
    # Serial port specified.
    print "Using specified port: %s " % PORT
    horizon = Horizon(transport = transports.HorizonTransport_Serial, 
                      transport_args = { 'port': PORT },
                      rec_timeout = 5000, send_timeout = 200, store_timeout = 10000)
else:
    # Not specified. Autodetect.
    print "Using Serial Autodetect."
    horizon = Horizon(rec_timeout = 5000, send_timeout = 200, store_timeout = 10000)

horizon.open()

announce_pub = rospy.Publisher('/clearpath/announce', Announce, latch=True)
announce_pub.publish(action="new_robot", topic=os.getenv("ROS_NAMESPACE", ""));

data.DataReceiver(horizon)

VELOCITY_CONTROL = int(rospy.get_param('~velocity_control', '0')) # bool
OUTPUT_CONTROL = int(rospy.get_param('~output_control', '0')) # bool
tx = True
UPDATE_HZ = 8

met_to_hundred_percent = 1.0;
met_per_sec_to_percent = 100.0 / met_to_hundred_percent;
w_to_percent_change = 50.0

def cmd_vel_callback(data):
    global tx
    if tx == True:
        z_angular_velocity = data.angular.z
        x_velocity = data.linear.x        

        if VELOCITY_CONTROL == 1:
	     horizon.set_ackermann_output(z_angular_velocity, x_velocity, 0)
        
        if OUTPUT_CONTROL == 1:
            linear = x_velocity * met_per_sec_to_percent
            diff = z_angular_velocity * w_to_percent_change
            left_percent = linear - diff
            right_percent = linear + diff
            left_percent = max(min(left_percent, 100.0), -100.0)
            right_percent = max(min(right_percent, 100.0), -100.0)
            horizon.set_differential_output(left_percent, right_percent)

        tx = False

rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
rate = rospy.Rate(UPDATE_HZ)

while not rospy.is_shutdown():
    rate.sleep()
    tx = True

