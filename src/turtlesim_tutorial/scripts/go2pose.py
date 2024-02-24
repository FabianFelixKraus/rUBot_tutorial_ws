#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi


class TurtleBot:

    def __init__(self):
        rospy.init_node('move_turtle', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.start_pose = Pose()
        self.start_pose.x = 5  # Default start x coordinate
        self.start_pose.y = 5  # Default start y coordinate
        self.distance_tolerance = rospy.get_param("~tol")
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def rotate_to_goal(self, goal_pose):
        angle_to_goal = self.steering_angle(goal_pose)
        vel_msg = Twist()

        while abs(angle_to_goal - self.pose.theta) > 0.01:
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y
        distance_tolerance = self.distance_tolerance

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Robot Reached destination")

        # Rotate to face the starting point after reaching the goal
        self.rotate_to_goal(self.start_pose)
        rospy.loginfo("Robot Oriented to face the starting point")


if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass
