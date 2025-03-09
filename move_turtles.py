#!/usr/bin/env python3
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Giới hạn khu vực
X_MIN, X_MAX = 1, 9
Y_MIN, Y_MAX = 1, 9
SAFE_DISTANCE = 3
SPEED = 1.0  # Giảm tốc độ di chuyển
TURN_ANGLE = math.pi / 2  # Tăng góc quay để tránh tường nhanh hơn
SLOW_DOWN_DISTANCE = 1.0  # hoảng cách bắt đầu giảm tốc độ khi gần tường

class TurtleMover:
    def __init__(self, name):
        self.name = name
        self.pub = rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(f'/{name}/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        rospy.sleep(1)

    def update_pose(self, data):
        self.pose = data

    def move_randomly(self, other_turtle):
        vel_msg = Twist()

        while not rospy.is_shutdown():
            turn_factor = 1.0  #  Điều chỉnh mức độ quay
            speed = SPEED  #  Tốc độ mặc định

            #  Tính khoảng cách đến tường
            dist_to_left_wall = self.pose.x - X_MIN
            dist_to_right_wall = X_MAX - self.pose.x
            dist_to_bottom_wall = self.pose.y - Y_MIN
            dist_to_top_wall = Y_MAX - self.pose.y

            # Kiểm tra tường và điều chỉnh hướng đi mềm mại
            if dist_to_left_wall < SAFE_DISTANCE:
                rospy.logwarn(f"{self.name} gần tường trái! Quay phải...")
                turn_factor += TURN_ANGLE  #  Quay nhiều hơn khi gần tường
                speed *= max(0.2, dist_to_left_wall / SAFE_DISTANCE)  #  Giảm tốc độ dần

            elif dist_to_right_wall < SAFE_DISTANCE:
                rospy.logwarn(f"{self.name} gần tường phải! Quay trái...")
                turn_factor -= TURN_ANGLE
                speed *= max(0.2, dist_to_right_wall / SAFE_DISTANCE)

            if dist_to_bottom_wall < SAFE_DISTANCE:
                rospy.logwarn(f"{self.name} gần tường dưới! Quay lên...")
                turn_factor += TURN_ANGLE
                speed *= max(0.2, dist_to_bottom_wall / SAFE_DISTANCE)

            elif dist_to_top_wall < SAFE_DISTANCE:
                rospy.logwarn(f"{self.name} gần tường trên! Quay xuống...")
                turn_factor -= TURN_ANGLE
                speed *= max(0.2, dist_to_top_wall / SAFE_DISTANCE)

            # Nếu ở góc, quay nhiều hơn để tránh bị kẹt
            if (dist_to_left_wall < SAFE_DISTANCE or dist_to_right_wall < SAFE_DISTANCE) and \
               (dist_to_bottom_wall < SAFE_DISTANCE or dist_to_top_wall < SAFE_DISTANCE):
                rospy.logwarn(f"{self.name} ở góc! Quay nhiều hơn...")
                turn_factor *= 2.0  #  Quay nhiều hơn khi ở góc

            #  Kiểm tra rùa khác và tránh va chạm
            if self.check_collision(other_turtle):
                rospy.logwarn(f"{self.name} tránh va chạm với {other_turtle.name}!")
                turn_factor += TURN_ANGLE * random.choice([-1, 1])  # Quay ngẫu nhiên để tránh
                speed *= 0.5  #  Giảm tốc độ khi gần rùa khác

            #  Áp dụng góc quay và tốc độ
            vel_msg.linear.x = speed
            vel_msg.angular.z = turn_factor
            self.pub.publish(vel_msg)
            self.rate.sleep()

    def check_collision(self, other_turtle):
        """ Kiểm tra nếu hai rùa gần nhau """
        dist = math.sqrt((self.pose.x - other_turtle.pose.x)**2 + (self.pose.y - other_turtle.pose.y)**2)
        return dist < SAFE_DISTANCE

if __name__ == '__main__':
    rospy.init_node('move_turtles_node', anonymous=True)
    turtle1 = TurtleMover('turtle1')
    turtle2 = TurtleMover('turtle2')

    try:
        rospy.sleep(2)
        from threading import Thread
        Thread(target=turtle1.move_randomly, args=(turtle2,)).start()
        Thread(target=turtle2.move_randomly, args=(turtle1,)).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass