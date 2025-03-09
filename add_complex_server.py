#!/usr/bin/env python3
import rospy
from complex_service.srv import AddComplex, AddComplexResponse

def handle_add_complex(req):
    result_re = req.re1 + req.re2
    result_im = req.im1 + req.im2
    rospy.loginfo(f"Received: ({req.re1} + {req.im1}i) + ({req.re2} + {req.im2}i) = ({result_re} + {result_im}i)")
    return AddComplexResponse(result_re, result_im)

def add_complex_server():
    rospy.init_node('add_complex_server')
    s = rospy.Service('addComplex', AddComplex, handle_add_complex)
    rospy.loginfo("Ready to add complex numbers.")
    rospy.spin()

if __name__ == "__main__":
    add_complex_server()
