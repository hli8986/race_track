#!/usr/bin/env python

# Team: Richard Li
#       Khaled Jadi
#       Kanad Bhagwat
#       Neha Gupta

# Import preparation
import rospy, cv2, cv_bridge
import numpy as np
import tf, time
import matplotlib.pyplot as plt 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
from sensor_msgs.msg import LaserScan
from matplotlib.colors import hsv_to_rgb

class race_track:
  
  def __init__(self):
  
    # Initialize cv_bridge
    self.bridge = cv_bridge.CvBridge()
    
    # Subscibe from ros topics
    self.sub_odm = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
    self.sub = rospy.Subscriber("/scan", LaserScan,self.laser_callback)
    self.image_sub = rospy.Subscriber("/ps3a/image_raw", Image, self.callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    rospy.Rate(10)


  def laser_callback(self,msg):
    global forward_range
    
    # Retrieve range data
    forward_range = msg.ranges[320:360]
    
  def odom_callback(self, msg):
    global twist 
    
    # Retrieve velocity data
    twist = msg.twist.twist
     
  def callback(self,data):
  
    # Convert the ROS image to opencv image
    image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')

    # Convert the image from bgr to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define color threshold
    # lower_red = np.array([18, 40, 90])
    # upper_red = np.array([27, 255, 255])
    lower_red = np.array([1,150,200])#([1, 150, 200])#upper
    upper_red = np.array([18,255,255])#([18, 255, 255])#lower
    
    # Testing for color pickig
    # light_orange = (1, 150, 200)
    # dark_orange = (18, 255, 255)
    # lo_square = np.full((10, 10, 3), light_orange, dtype=np.uint8) / 255.0
    # do_square = np.full((10, 10, 3), dark_orange, dtype=np.uint8) / 255.0
    # plt.subplot(1, 2, 1)
    # plt.imshow(hsv_to_rgb(do_square))
    # plt.subplot(1, 2, 2)
    # plt.imshow(hsv_to_rgb(lo_square))
    # plt.show()
    
    # Detect color and remove the small bubbles if there is any
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    print 1.1
    # Retrieve current view width and height
    h_view, w_view, d_view = image.shape

    # Find contours for the detected color
    # contours= cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
    
    # Initialize contour center
    cx = 0
    cy = 0
    print 2
    # Initialize obstacle detecting state
    flag_wall = 0
    print 'set'
    
    # self.twist.linear.x = 0.3
    # self.cmd_vel_pub.publish(self.twist)
    #rospy.sleep(5)
    
    # Update state based on range of 2.5 meters
    for i in forward_range:
	    if i < 1:       
		   flag_wall = 1
                   print "obstacle"
	    
            if i > 1:
		   flag_wall = 0 
                   print "no obstacle"
		   
    		       
    if flag_wall == 0: # Obstacle not detected
              #rospy.sleep(5) 
            # Line following
            # if len(contours) > 0: # If detected color
              
              # Get the biggest contour
              # c = max(contours, key=cv2.contourArea)
              
              # Get moments from the contour
              M = cv2.moments(mask)

              # Define the center of the contour
              if M['m00'] > 0:
	        cx = int(M['m10']/M['m00'])
	        cy = int(M['m01']/M['m00'])
	      
                cv2.circle(image, (cx, cy), 5, (255, 255, 0), -1)

                # Get output from PID linear.x
                # linear_x=PID_linear(h_view/2, cy)
                PID_angular(w_view/2, cx, 1, 0.5, 0, 0, 0.0005)
                print "target acquired"
              
              else: # If color not detected, go forward
               
                # linear_x = PID_linear(0, twist.linear.x) * 1000
                PID_angular(0.4, twist.angular.z, 1, 0, 0, 0, 0.00065)
                print "looking for target"
              
    else: # If obstacle detected   
    
              # Decelerate to stop
              
              linear_x1 = PID_linear(0, twist.linear.x) * 1000
	      PID_angular(0, twist.angular.z, 1, linear_x1, 0, 0, 0.00065)
	      
	      if twist.linear.x <= 0: # If fully stopped
	        
	        for i in forward_range: # Recheck obstacle
	          if  i<2.5: 
	            # Turn
	            linear_x2 = PID_linear(0.5, twist.linear.x) * 1000
                    PID_angular(4.0, twist.angular.z, 1, linear_x2, 0, 0, 0.1)
                    rospy.sleep(0.01)
                    print "Turning"
                    
              # else: # If not fully stopped, stop the robot
                # linear_x3 = PID_linear(0, twist.linear.x) * 1000
	        # PID_angular(0, twist.angular.z, 1, linear_x3, 0, 0, 0.00065)
    
    # Show original image
    cv2.circle(image, (w_view/2, h_view/2), 5, (0, 128, 0), -1)
    cv2.imshow("Viewed_image", image)
    cv2.imshow("Mask", mask)
    cv2.waitKey(3)
    
def PID_angular(setpoint, input_value, l_constant, linear_x, a_constant, angular_z, kp):

    # Publish to cmd_vel topic
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Define current twist message
    current_twist = Twist()

    # Define new twist message
    new_twist = current_twist

    # Start counting time now
    current_time = rospy.Time.now()
    
    # Define last values
    last_time = rospy.Duration(1/1000)
    last_error = 0.0
    
    # Calculate time step
    dt = (current_time - last_time).to_sec()
    
    # Calculate error between setpoint and input value
    error = setpoint - input_value
    
    # Integral error
    i_error = 0.0
    i_error += error * dt
    
    # Derivative error
    d_error = 0.0
    d_error = error - last_error
    
    # PID gains
#    kp = 0.00065
    ki = 0.00000001
    kd = 0.0
    
    # Output value
    output = kp * error + ki * i_error + kd * d_error
    #print output
    
    # define last error and last time
    last_error = error
    last_time = current_time
    
    # Update twist message with output
    if a_constant == 0:
     new_twist.angular.z = current_twist.angular.z + output
#     print new_twist.angular.z
    elif a_constant == 1:
     new_twist.angular.z = angular_z
    
    if l_constant == 0:
      new_twist.linear.x = current_twist.linear.x + linear_x
#      print new_twist.linear.x
    elif l_constant == 1:
      new_twist.linear.x = linear_x
    
    # Publish the new twist message
    twist_pub.publish(new_twist)

def PID_linear(setpoint, input_value):
    
    # Define current twist message
    current_twist = Twist()

    # Define new twist message
    new_twist = current_twist

    # Start counting time now
    current_time = rospy.Time.now()
    
    # Define last values
    last_time = rospy.Duration(1/1000)
    last_error = 0.0
    
    # Calculate time step
    dt = (current_time - last_time).to_sec()
    
    # Calculate error between setpoint and input value
    error = setpoint - input_value
    
    # Integral error
    i_error = 0.0
    i_error += error * dt
    
    # Derivative error
    d_error = 0.0
    d_error = error - last_error
    
    # PID gains
    kp = 0.002
    ki = 0.0000001
    kd = 0.0
    
    # Output value
    output = kp * error + ki * i_error + kd * d_error
    
    # define last error and last time
    last_error = error
    last_time = current_time
    
    return output	      
      
if __name__ == '__main__': 
  
  # Initialize node follower
  rospy.init_node('follower', anonymous=True)
  # Call the class and functions
  follower = race_track()
  
  rospy.spin()
  
