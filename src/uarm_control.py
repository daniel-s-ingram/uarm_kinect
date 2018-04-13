#!/usr/bin/env python
import rospy
import pyuarm
import tf
from std_msgs.msg import Bool

uarm = pyuarm.uarm.UArm()
pump = False

def pump_callback(msg):
	global pump
	pump = msg.data

rospy.init_node('uarm_control', anonymous=True)
listener = tf.TransformListener()
rospy.Subscriber('/uarm/pump', Bool, pump_callback)

while not rospy.is_shutdown():
	try:
		hand_translation = listener.lookupTransform('openni_depth_frame', 'left_hand_1', rospy.Time(0))[0]
		origin = listener.lookupTransform('openni_depth_frame', 'left_shoulder_1', rospy.Time(0))[0]
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	x = (hand_translation[0]-origin[0])*500
	y = (hand_translation[1]-origin[1])*500
	z = (hand_translation[2]-origin[2])*500

	if (-365 < x < 365) and (115 < y < 365) and (-120 < z < 190):
		uarm.set_position(x, y, z, speed=1000)

	uarm.set_pump(pump)

uarm.set_pump(False)
uarm.set_position(0, 0, 0, wait=True)
uarm.disconnect()