#!/usr/bin/env python
from __future__ import print_function

import rospy
import message_filters
from find_object_2d.msg     import ObjectPose
from find_object_2d.msg     import DetectionInfo
from geometry_msgs.msg      import PoseStamped
from std_msgs.msg           import String
import numpy as np

poseXold = 0.0
poseYold = 0.0
poseX = 0.0
poseY = 0.0


def listener():
    rospy.init_node('objectsPublisher_node', anonymous=True)          

    info_sub = message_filters.Subscriber('/objectsName', DetectionInfo)
    pose_sub = message_filters.Subscriber('/objectsPose', PoseStamped)
    ts = message_filters.ApproximateTimeSynchronizer([info_sub, pose_sub], 10, 0.5, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

def callback(objectsName, objectsPose):

    global poseX
    global poseY
    global poseXold
    global poseYold

    pub = rospy.Publisher('objectsInfo', ObjectPose, queue_size=10) 
    pub_marker = rospy.Publisher('fre_detections', String, queue_size=10)                      
    msg = ObjectPose()                              
    path = str(objectsName.filePaths)
    # subpath = path.split("/")[-1]
    # subpath2 = subpath.split(".")[0]
    # name = subpath2[0:-1]

    litter = "litter"
    isLitter = path.find(litter)
    weed = "weed"
    isWeed = path.find(weed)

    if isLitter > -1:
        name = "litter"
    elif isWeed > -1:
        name = "weed" 
    else:
        name = "nada"
    
    msg.kind = name
    msg.x = objectsPose.pose.position.x     
    msg.y = objectsPose.pose.position.y


    poseX = msg.x
    poseY = msg.y
    
    dist = np.linalg.norm(np.array((poseX, poseY)) - np.array((poseXold, poseYold)))
    print("distance is:", dist)
    if dist > 0.3:
        poseXold = poseX
        poseYold = poseY
        #print("i was there")
        #print(str(msg.kind))

        pub_marker.publish(str(msg.kind)) 

   
    pub.publish(msg)
    rospy.sleep(1)

if __name__ == '__main__':
       
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
