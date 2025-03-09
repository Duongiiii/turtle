#!/usr/bin/env python3
import rospy
from complex_service.srv import AddComplex, AddComplexRequest

def add_complex_client(re1, im1, re2, im2):
    rospy.wait_for_service('addComplex')
    try:
        add_complex = rospy.ServiceProxy('addComplex', AddComplex)
        resp = add_complex(float(re1), float(im1), float(re2), float(im2))
        return resp.re, resp.im
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('add_complex_client')
    re1, im1, re2, im2 = 2, 3, 5, 7
    result = add_complex_client(re1, im1, re2, im2)
    rospy.loginfo(f"Result: {result[0]} + {result[1]}i")
