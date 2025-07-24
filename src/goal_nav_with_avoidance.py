#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math

pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
waypoints = [(1.0, 0.5), (2.0, 0.0), (2.5, 1.0)]
current_goal_index = 0
obstacle_near = False

def odom_callback(msg):
    global pose
    pose['x'] = msg.pose.pose.position.x
    pose['y'] = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    pose['theta'] = yaw

def scan_callback(msg):
    global obstacle_near
    front_ranges = msg.ranges[0:10] + msg.ranges[-10:]  # 20 readings from front
    min_front_distance = min(front_ranges)
    obstacle_near = min_front_distance < 0.4  # threshold in meters

def move_to_waypoints():
    global current_goal_index
    rospy.init_node('turtlebot3_goal_navigator')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rate = rospy.Rate(10)

    vel = Twist()

    while not rospy.is_shutdown():
        if current_goal_index >= len(waypoints):
            vel.linear.x = 0
            vel.angular.z = 0
            vel_pub.publish(vel)
            rospy.loginfo(" All goals reached.")
            break

        goal_x, goal_y = waypoints[current_goal_index]
        dx = goal_x - pose['x']
        dy = goal_y - pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - pose['theta']
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Obstacle detected in front
        if obstacle_near:
            rospy.logwarn("Obstacle ahead! Stopping and rotating")
            vel.linear.x = 0.0
            vel.angular.z = 0.5  # rotate in place
        else:
            # Normal waypoint control
            vel.linear.x = 0.2 * distance
            vel.angular.z = 1.0 * angle_error

            if distance < 0.15:
                rospy.loginfo(f"Reached goal {current_goal_index + 1}")
                current_goal_index += 1

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_waypoints()
    except rospy.ROSInterruptException:
        pass
