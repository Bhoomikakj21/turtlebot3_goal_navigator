#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# We'll store the robot's current position and orientation here
pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

# List of waypoints (x, y) the robot should visit
waypoints = [(1.0, 0.5), (2.0, 0.0), (2.5, 1.0)]
current_goal_index = 0  # start from the first goal

def odom_callback(msg):
    """
    Callback to update the robot's current pose from /odom topic.
    Converts quaternion orientation to a usable yaw angle.
    """
    global pose
    pose['x'] = msg.pose.pose.position.x
    pose['y'] = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    pose['theta'] = yaw  # orientation in radians

def move_to_waypoints():
    """
    Main loop: move the robot toward each waypoint using proportional control.
    Stops once all goals are reached.
    """
    global current_goal_index
    rospy.init_node('turtlebot3_goal_navigator')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)  # 10 Hz

    vel = Twist()  # message to send velocity commands

    while not rospy.is_shutdown():
        # If all waypoints have been visited, stop and exit
        if current_goal_index >= len(waypoints):
            vel.linear.x = 0
            vel.angular.z = 0
            vel_pub.publish(vel)
            rospy.loginfo(" All goals reached.")
            break

        # Get the next goal coordinates
        goal_x, goal_y = waypoints[current_goal_index]
        dx = goal_x - pose['x']
        dy = goal_y - pose['y']
        distance = math.sqrt(dx**2 + dy**2)

        # Find the angle to the goal and compute the difference from current orientation
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - pose['theta']
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalize

        # Proportional control logic
        vel.linear.x = 0.2 * distance
        vel.angular.z = 1.0 * angle_error

        # If we're close enough to the goal, move to the next
        if distance < 0.15:
            rospy.loginfo(f" Reached goal {current_goal_index + 1}")
            current_goal_index += 1

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    move_to_waypoints()
    