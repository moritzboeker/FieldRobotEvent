#! /usr/bin/env python

import rospy
from geometry_msgs.msg      import PoseStamped
from qr_code_pkg.srv        import *

def get_pose_client():
    rospy.wait_for_service('get_qrPose')
    try:
        client_handle = rospy.ServiceProxy('get_qrPose', GetQrPose)
        response = client_handle()
        return response.pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    get_pose_client()