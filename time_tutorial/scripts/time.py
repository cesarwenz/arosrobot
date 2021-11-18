#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

odom = Odometry ()

def callback(data):
    global odom
    odom = data
    printer(odom)
    sub.unregister()

def printer(data):
    rospy.loginfo("I heard x_pos: %s", data.pose.pose.position.x)
    rospy.loginfo("I heard y_pos: %s", data.pose.pose.position.y)

def simple_input():
    rospy.init_node('simple_input', anonymous=True)
    velocity_publisher = rospy.Publisher('/icerobot_velocity_controller/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    #Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
    distance = input("Type your distance:")
    isForward = input("Foward?: ")#True or False
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)
    print('done')

def controller_input(u):
    rospy.init_node('simple_input', anonymous=True)
    velocity_publisher = rospy.Publisher('/icerobot_velocity_controller/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = u(1)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def states():
    rospy.init_node('states', anonymous=True)
    #sub = rospy.Subscriber("/odometry/filtered_map", Odometry, callback)
    msg = rospy.wait_for_message("/odometry/filtered_map", Odometry)
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    th = tf.transformations.euler_from_quaternion(quaternion)[2] 
    states = np.array([x, y, th])
    return states


if __name__ == '__main__':
    try:
        states()
    except rospy.ROSInterruptException:
        pass
