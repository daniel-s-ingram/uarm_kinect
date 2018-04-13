#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import tf
import numpy as np
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from math import sqrt, acos, pi

lower = np.array([0,0,0])
upper = np.array([0,0,0])

def update_mask(_):
	global lower, upper
	lower = np.array([cv2.getTrackbarPos("Lower Hue", 'Gesture Detector'), 
					cv2.getTrackbarPos("Lower Saturation", 'Gesture Detector'),
					cv2.getTrackbarPos("Lower Value", 'Gesture Detector')])
	upper = np.array([cv2.getTrackbarPos("Upper Hue", 'Gesture Detector'), 
					cv2.getTrackbarPos("Upper Saturation", 'Gesture Detector'),
					cv2.getTrackbarPos("Upper Value", 'Gesture Detector')])

class GestureDetector:
	def __init__(self):
		cv2.namedWindow('Gesture Detector', cv2.WINDOW_NORMAL)

		cv2.createTrackbar("Lower Hue", 'Gesture Detector', 0, 180, update_mask)
		cv2.createTrackbar("Upper Hue", 'Gesture Detector', 0, 180, update_mask)
		cv2.createTrackbar("Lower Saturation", 'Gesture Detector', 0, 255, update_mask)
		cv2.createTrackbar("Upper Saturation", 'Gesture Detector', 0, 255, update_mask)
		cv2.createTrackbar("Lower Value", 'Gesture Detector', 0, 255, update_mask)
		cv2.createTrackbar("Upper Value", 'Gesture Detector', 0, 255, update_mask)
		#cv2.createTrackbar("Pump Enable", "Camera", 0, 1, nothing)

		#Good default values
		cv2.setTrackbarPos("Lower Hue", 'Gesture Detector', 136)
		cv2.setTrackbarPos("Upper Hue", 'Gesture Detector', 180)
		cv2.setTrackbarPos("Lower Saturation", 'Gesture Detector', 50)
		cv2.setTrackbarPos("Upper Saturation", 'Gesture Detector', 176)
		cv2.setTrackbarPos("Lower Value", 'Gesture Detector', 0)
		cv2.setTrackbarPos("Upper Value", 'Gesture Detector', 135)

		self.bridge = CvBridge()
		self.listener = tf.TransformListener()
		self.kinect_model = image_geometry.PinholeCameraModel()
		
		self.pump_pub = rospy.Publisher('/uarm/pump', Bool, queue_size=10)
		rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.get_camera_info)
		rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)

		self.pump_msg = Bool()

	def image_callback(self, image_msg):
		global lower, upper
		
		color_image = self.bridge.imgmsg_to_cv2(image_msg)
		hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
		hand_mask = np.zeros_like(hsv_image)
		
		try:
			hand_pos = self.listener.lookupTransform('openni_depth_frame', 'left_hand_1', rospy.Time(0))[0]

			x, y = self.kinect_model.project3dToPixel((hand_pos[1], hand_pos[2], hand_pos[0]))
			cv2.circle(hand_mask, (640-int(x),480-int(y)), 75, (255, 255, 255), -1)
			hsv_image = cv2.bitwise_and(hsv_image, hand_mask)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		hsv_mask = cv2.inRange(hsv_image, lower, upper)

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
		mask = cv2.dilate(hsv_mask, kernel, iterations=3)
		mask = cv2.erode(mask, kernel, iterations=5)
		mask = cv2.GaussianBlur(mask, (5, 5), 0)

		#mask = cv2.bitwise_and(hsv_image, mask)

		_, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			maxArea = 0
			contour = 0
			for i in range(len(contours)):
				area = cv2.contourArea(contours[i])
				if area > maxArea:
					maxArea = area
					contour = i
			
			hand = contours[contour]
			hull = cv2.convexHull(hand)
			hullPts = cv2.convexHull(hand, returnPoints=False)
			defects = cv2.convexityDefects(hand, hullPts)

			pump = False
			if defects is not None:
				fingers = 0
				for i in range(defects.shape[0]):  # calculate the angle
					s, e, f, d = defects[i][0]
					start = tuple(hand[s][0])
					end = tuple(hand[e][0])
					far = tuple(hand[f][0])
					a = sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
					b = sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
					c = sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
					angle = acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))
					if angle <= pi / 2:  
						fingers += 1

				if fingers < 4:
					pump = True
				else:
					pump = False
			
			self.pump_msg.data = pump
			self.pump_pub.publish(self.pump_msg)

			cv2.drawContours(color_image, contours, contour, (255, 0, 0), 3)
			cv2.drawContours(color_image, [hull], 0, (0, 0, 255), 3)

		cv2.imshow('Gesture Detector', np.hstack((color_image, cv2.bitwise_and(color_image, color_image, mask=mask))))
		cv2.waitKey(1)

	def get_camera_info(self, msg):
		self.kinect_model.fromCameraInfo(msg)

if __name__ == '__main__':
	rospy.init_node('gesture_detector', anonymous=True)
	gesture_detector = GestureDetector()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down gesture detector.')

	cv2.destroyAllWindows()