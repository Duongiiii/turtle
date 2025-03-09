#!/usr/bin/env python3

import rospy
from goal.srv import GoToGoal, GoToGoalResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

turtle_pose = Pose()

def pose_callback(msg):
    global turtle_pose
    turtle_pose = msg

def handle_go_to_goal(req):
    global turtle_pose
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rate = rospy.Rate(10)
    goal_x, goal_y = req.x, req.y
    tolerance = 0.1
    speed = Twist()

    while not rospy.is_shutdown():
        distance = math.sqrt((goal_x - turtle_pose.x) ** 2 + (goal_y - turtle_pose.y) ** 2)
        angle = math.atan2(goal_y - turtle_pose.y, goal_x - turtle_pose.x)
        speed.linear.x = 1.5 * distance
        speed.angular.z = 4 * (angle - turtle_pose.theta)
        pub.publish(speed)

        if distance < tolerance:
            speed.linear.x = 0
            speed.angular.z = 0
            pub.publish(speed)
            return GoToGoalResponse(success=True)

        rate.sleep()

def goal_server():
    rospy.init_node('goal_server')
    rospy.Service('go_to_goal', GoToGoal, handle_go_to_goal)
    rospy.spin()

if __name__ == "__main__":
    goal_server()
