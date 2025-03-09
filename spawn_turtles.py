#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn, SpawnRequest

def spawn_turtle():
    rospy.init_node('spawn_turtle_node', anonymous=True)
    
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(4, 4, 0, 'turtle2')  # Tạo rùa mới 
        rospy.loginfo("Spawned turtle2 successfully!")
    except rospy.ServiceException as e:
        rospy.logwarn("Turtle2 có thể đã tồn tại, tiếp tục chạy...")
    
if __name__ == '__main__':
    spawn_turtle()

