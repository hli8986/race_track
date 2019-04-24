#!/usr/bin/env python
import rospy, sys, select, tty, termios
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

key_mapping = { 'w': [ 0, 1], 'x': [ 0, -1],
                'a': [ 1, 0], 'd': [-1, 0],
                's': [ 0, 0] }

def filter_vel(current_speed, target_speed, previous_time, current_time):

    # Simple PI controller
    dt = (current_time - previous_time).to_sec()
#    dt = 0.1
    error = target_speed - current_speed
    kp = 0.1
    ki = 1
    iv = 0.0
    iv += error * dt
    output = kp * error + ki * iv
    return output

def send_twist():
    global last_twist_send_time, current_twist, last_twist, twist_pub, k
    
    # Start time
    current_time = rospy.Time.now()
    
    # Add current speed with the output from PI controller for each component of velocity
    last_twist.angular.z = current_twist.angular.z + filter_vel(current_twist.angular.z, vels[0], last_twist_send_time, current_time)
    
    last_twist.linear.x = current_twist.linear.x + filter_vel(current_twist.linear.x, vels[1], last_twist_send_time, current_time)
    
    # Update time
    last_twist_send_time = current_time
#    print last_twist.linear.x
    twist_pub.publish(last_twist)

if __name__ == '__main__':
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('filter_speed')
    
    # Solve the problem of enter in linux
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # Initialize variables
    last_twist_send_time = rospy.Time.now()
    vels = [0,0]
    current_twist = Twist()
    last_twist = current_twist
    
    # Rate at 10 Hz
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown(): 
    
      # Read Keys
      if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
        vels = key_mapping[sys.stdin.read(1)]
      
      # Update speed
      send_twist()
      rate.sleep()
      
    # Back to console for keyboard entering
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
