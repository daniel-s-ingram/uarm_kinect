#!/usr/bin/env python
import rospy
import pyuarm
import tf

uarm = pyuarm.uarm.UArm()

rospy.init_node('uarm_control', anonymous=True)
listener = tf.TransformListener()

while not rospy.is_shutdown():
	try:
		hand_translation = listener.lookupTransform('openni_depth_frame', 'left_hand_1', rospy.Time(0))[0]
		origin = listener.lookupTransform('openni_depth_frame', 'left_shoulder_1', rospy.Time(0))[0]
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	x = (hand_translation[0]-origin[0])*500
	y = (hand_translation[1]-origin[1])*500
	z = (hand_translation[2]-origin[2])*500

	try:
		uarm.set_position(x, y, z)
	except:
		pass

uarm.disconnect()