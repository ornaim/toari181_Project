#!/usr/bin/env python
import	rospy
import	actionlib
from	move_base_msgs.msg	import	MoveBaseAction,	MoveBaseGoal
import numpy as np
import ctypes
import struct
import roslib
import time
import math
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image, JointState, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import openni2_camera
from actionlib_msgs import *

red_object_loc=[]
FOUND_OBJECT=1;
NO_OBJECT=0;



def moveToGoal():
	global analyze_bool
	global NodeInCallback
	client	=	actionlib.SimpleActionClient('move_base',	MoveBaseAction)		
	client.wait_for_server(rospy.Duration(60))
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	xGoal = 12.003
	yGoal = -0.826
	goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = 0.541
	goal.target_pose.pose.orientation.w = 0.841

	rospy.loginfo("Sending goal location ...")
	client.send_goal(goal)
	resultTime =client.wait_for_result(rospy.Duration(360))
	if(client.get_state() ==  actionlib.GoalStatus.SUCCEEDED):
		rospy.loginfo("You have reached the destination")
		finished=True;
		analyze_bool , NodeInCallback=True , True # find red object 
		return True
	else:
		rospy.loginfo("The robot failed to reach the destination")
		return False


def analyze(img): #Finds Red object.
	# rospy.loginfo("analyze ");
	global NodeInCallback
	global found_red_object
	global red_object_loc 
	global analyze_bool 
	global depth_bool
	global fininsh_analyze
	if analyze_bool:
		cv_image = CvBridge().imgmsg_to_cv2(img,"bgr8")
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		# lower mask (0-10)
		lower_red = np.array([170,70,70])
		upper_red = np.array([180,255,255])
		mask = cv2.inRange(hsv, lower_red, upper_red)
		# upper mask (170-180)

		# join my masks
		mask_sum=cv2.sumElems(mask)
		res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
		if(mask_sum[0]>20000):
			found_red_object=FOUND_OBJECT
			height=res.shape[0]
			width=res.shape[1]
			count=0
			for y in range(0,height):
				for x in range(0,width):
					if mask[y,x] ==255:
						red_object_loc.append(str(x) +","+str(y))
						count+=1
			# print len(red_object_loc)
			# cv2.imshow("mask", mask )
			cv2.startWindowThread()
			cv2.namedWindow("preview")
			cv2.imshow("preview", res)
			depth_bool=True
			fininsh_analyze=True
			analyze_bool=False
		else:
			found_red_object=NO_OBJECT
			depth_bool=True
			fininsh_analyze=True
			analyze_bool=False
			# cv2.waitKey()
		

		# rospy.loginfo("fininsh_command")

def culc_distance(img):
	global NodeInCallback 
	global found_red_object 
	global red_object_loc 
	global fininsh_analyze 
	global depth_bool 
	global analyze_bool
	
	# rospy.loginfo("fininsh_analyze: "+str(fininsh_analyze))
	# 
	if (depth_bool==True and fininsh_analyze==True):
		min_D=10000;
		depth_bool , analyze_bool=False ,False
		# rospy.logerr("culc_distance")
		# rospy.loginfo("handle laizer, and found=="+ str(found_red_object==FOUND_OBJECT));
		if(found_red_object==FOUND_OBJECT):
			cv_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="passthrough")
			height =cv_image.shape[0]
			width=cv_image.shape[1]
			for i in range(0,len(red_object_loc)):
				x , y=red_object_loc[i].split(",")
				if(min_D>cv_image[y,x]):
					min_D=cv_image[y,x]
			print "closest_obj: "+str(min_D)
			NodeInCallback=False
			# rospy.loginfo(": ");

			
		elif(found_red_object==NO_OBJECT):
			print("No red object found.")
			# rospy.loginfo("runnnnnnnnnnnnnnn: ");
			NodeInCallback=False

			return None








if	__name__	==	'__main__':
	global NodeInCallback
	global analyze_bool
	global depth_bool
	global fininsh_analyze
	rospy.init_node('map_navigation',anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	NodeInCallback, analyze_bool , depth_bool, fininsh_analyze= False, False, False ,False
	sub_camera=rospy.Subscriber("/torso_camera/rgb/image_raw", Image, analyze)
	sub_laizer=rospy.Subscriber("/torso_camera/depth_registered/image_raw", Image, culc_distance)

	try:
		rospy.loginfo("inside try")	
		moveToGoal()
		rospy.spin()


	except rospy.ROSInterruptException:
		rospy.loginfo("map_navigation node terminated.")			
