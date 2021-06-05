#!/usr/bin/env python
from __future__ import print_function

import rospy
import message_filters
from find_object_2d.msg     import ObjectPose
from find_object_2d.msg     import DetectionInfo
from geometry_msgs.msg      import PoseStamped

def listener():
    rospy.init_node('objectsInfo_node', anonymous=True)          

    info_sub = message_filters.Subscriber('/objectsName', DetectionInfo)
    pose_sub = message_filters.Subscriber('/objectsPose', PoseStamped)
    #pose_test = rospy.Subscriber('/objectsPose', PoseStamped, callback_test)
    #name_test = rospy.Subscriber('/objectsInfo', DetectionInfo, callback_info)

    #ts = message_filters.TimeSynchronizer([info_sub, pose_sub], 10)
    ts = message_filters.ApproximateTimeSynchronizer([info_sub, pose_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

def callback(objectsName, objectsPose):
    pub = rospy.Publisher('objectsInfo', ObjectPose, queue_size=10)                       
    msg = ObjectPose()                              
    path = str(objectsName.filePaths)
    subpath = path.split("/")[-1]
    subpath2 = subpath.split(".")[0]
    name = subpath2[0:-1]
    msg.kind = name
    msg.x = objectsPose.pose.position.x     
    msg.y = objectsPose.pose.position.y               

    pub.publish(msg)                              
    rospy.sleep(1)

# def callback_test(data):
#     pub_test =  rospy.Publisher('test_topic', ObjectPose, queue_size=10)
#     value = ObjectPose()
#     value.header.stamp = rospy.Time.now()
#     value.poseX = data.pose.position.x 
#     value.poseY = data.pose.position.y
#     pub_test.publish(value)
#     rospy.sleep(1)

# def callback_info(data):
#     pub_test =  rospy.Publisher('test_topic', ObjectPose, queue_size=10)
#     value = ObjectPose()
#     value.header.stamp = rospy.Time.now()
#     value.objectName = str(data.filePaths)
#     pub_test.publish(value)
#     rospy.sleep(1)

if __name__ == '__main__':
       
    try:
        listener()
    except rospy.ROSInterruptException:
        pass