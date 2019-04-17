#!/usr/bin/env python

# Team: Richard Li
#       Khaled Jadi
#       Kanad Bhagwat
#       Neha Gupta

# Import preparation
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import tf
from sensor_msgs.msg import LaserScan

class race_track:
  
  def __init__(self):
    
    # Initialize cv_bridge
    self.bridge = cv_bridge.CvBridge()
    
    # Subscibe from ros topics
    self.sub_odm = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
    self.sub = rospy.Subscriber("front/scan", LaserScan,self.laser_callback)
    self.image_sub = rospy.Subscriber('front/image_raw', Image, self.callback)

  def laser_callback(self,msg):
    global forward_range
    
    # Retrieve range data
    forward_range = msg.ranges[0:360]
    
  def odom_callback(self, msg):
    global twist 
    
    # Retrieve velocity data
    twist = msg.twist.twist
     
  def callback(self,data):
  
    # Convert the ROS image to opencv image
    image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
    
    # Convert the image from bgr to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define red color threshold
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([10, 255, 255])
    
    # Detect color and remove the small bubbles if there is any
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Retrieve current view width and height
    h_view, w_view, d_view = image.shape

    # Find contours for the detected color
    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
    
    # Initialize contour center
    cx = 0
    cy = 0
