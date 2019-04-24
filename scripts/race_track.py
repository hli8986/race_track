#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import tf
from sensor_msgs.msg import LaserScan
from math import atan2






count = 0
class race_track:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.sub_odm = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)

    self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    self.twist = Twist()
    #self.posit=[]



  def odom_callback(self,msg):
    
    position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    q=msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    pose = [position[0], position[1], yaw]
    if pose[0]<5.987:
       self.call_me_to_go(pose)
       #print 'you are at the center'
    else:
       #print 'now the work satarted'
       pass 
       #self.posit.append(pose)
    #print self.posit[-1:]
    
    #print pose
  def call_me_to_go(self,pose):
    #self.twist = Twist()
    goal = Point()
    goal.x = 5.987
    goal.y = 0
    dif_x = goal.x -pose[0]
    dif_y = goal.y -pose[1]
    angle_to_goal = atan2(dif_y, dif_x)
    if pose[0] < 5.9:
        if abs(angle_to_goal - pose[2]) > 0.1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.4
            #print "Location  :" + str(pose[0])
        else:
            self.twist.linear.x = 0.4
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
      
      


  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([ 0, 255,  222])
    upper_red = np.array([0,255 , 230])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #(_,contours,hierarchy)=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    h, w, d = image.shape
    M = cv2.moments(mask)
    try:
        cx, cy =M['m10']/M['m00'],M['m01']/M['m00']
    except ZeroDivisionError:
        cx, cy =h/2,w/2
    cv2.circle(image, (int((cx)), int((cy))), 20, (0,255,0), -1)
    
    err = cx - w/2
    print err
#    print err
    if err==-64 or err > 200:
        self.twist.linear.x = 0
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
    else:
        self.twist.linear.x = 1
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
    #im2, contours, hierarchy = cv2.findContours(mask,1,2)	

    #print str(len(contours))
    #cv2.imshow("mask",mask)
    #cv2.imshow("output", image)
    #cv2.imshow("Yellow",hsv1)
    
    cv2.waitKey(1)

rospy.init_node('follower')
rospy.Rate(10)
follower = race_track()
rospy.spin()
