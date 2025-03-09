#!/usr/bin/env python3
import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

pose1 = None
pose2 = None

def pose_callback1(msg):
    global pose1
    pose1 = msg

def pose_callback2(msg):
    global pose2
    pose2 = msg

def check_collision(pub1, pub2):
    if pose1 and pose2:
        distance = math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)
        rospy.loginfo(f"Distance: {distance}")
        if distance < 0.3:
            rospy.logwarn("Collision detected! Changing direction...")
            stop_turtle(pub1)
            stop_turtle(pub2)
            rospy.sleep(1)
            move_turtle(pub1, -1.0, 0.5)  # Đổi hướng rùa 1
            move_turtle(pub2, -1.0, -0.5) # Đổi hướng rùa 2

def stop_turtle(pub):
    msg = Twist()
    pub.publish(msg)

def move_turtle(pub, speed_x, speed_z):
    msg = Twist()
    msg.linear.x = speed_x
    msg.angular.z = speed_z
    pub.publish(msg)

def main():
    rospy.init_node('collision_checker', anonymous=True)
    
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback2)
    
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        check_collision(pub1, pub2)
        rate.sleep()

if __name__ == "__main__":
    main()
