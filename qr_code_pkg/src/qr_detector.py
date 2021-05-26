#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
import numpy                as np
import pyzbar.pyzbar        as pyzbar

from sensor_msgs.msg        import Image
from sensor_msgs.msg        import CameraInfo
from geometry_msgs.msg      import PoseStamped
from qr_code_pkg.srv        import GetQrPose, GetQrPoseResponse
from cv_bridge              import CvBridge, CvBridgeError
from math                   import pi, atan2, asin

class image_qr:

    def __init__(self):
   
        camera_topic_name = rospy.get_param("~camera_topic_name", default='/usb_cam/image_raw')
        camera_info_topic_name = rospy.get_param("~camera_info_topic_name", default='/usb_cam/camera_info')
        output_topic_name = rospy.get_param("~output_topic_name", default='/qr_viewer')
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
        self.camera_info_sub = rospy.Subscriber(camera_info_topic_name, CameraInfo, self.callback_camera_info)
        self.image_sub = rospy.Subscriber(camera_topic_name, Image, self.callback)
        self.image_pub = rospy.Publisher(output_topic_name, Image, queue_size=1)
        self.pose_pub = rospy.Publisher('qr_pose', PoseStamped, queue_size=1)

        self.bridge = CvBridge()
        self.K = []
        self.D = []
        self.pose_transformed = []

    def callback(self, data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        draw_img = self.run(cv_image, self.K, self.D)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    def callback_camera_info(self, data):
        self.K = np.array(data.K, np.float32).reshape(3,3)
        self.D = np.array(data.D, np.float32)
 
    def draw(self, points, im, qr_type, qr_data):
        # If the points do not form a quad, find convex hull
        if len(points) > 4 : 
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else : 
            hull = points;
        
        # Number of points in the convex hull
        n = len(hull)
    
        # Draw the convext hull
        for j in range(0,n):
            cv2.line(im, hull[j], hull[ (j+1) % n], (255,255,0), 3)
            cv2.putText(im, str(j), hull[j], 0, 1, (0, 255, 255), 2)
        
        # between 0 and 2
        center_x = np.absolute(hull[2][0]+hull[0][0])/2
        center_y = np.absolute(hull[2][1]+hull[0][1])/2
        center = (center_x, center_y)
        cv2.putText(im, qr_data, center, 0, 1, (0, 0, 255), 2)

        hull.append(center)
        image_points = np.array(hull,np.float32)
 
        return im, image_points

    def draw_projection(self, img, corners, imgpts):
            corner = tuple(corners[0])
            #print(corner)
            point0 = np.array(imgpts[0])
            point1 = np.array(imgpts[1])
            point2 = np.array(imgpts[2])
            
            img = cv2.line(img, corner, (point0[0][0],point0[0][1]), (255,0,0), 3)
            img = cv2.line(img, corner, (point1[0][0],point1[0][1]), (0,255,0), 3)
            img = cv2.line(img, corner, (point2[0][0],point2[0][1]), (0,0,255), 3)
            return img

    def rot_params_rv(self,rvecs):
        R     = cv2.Rodrigues(rvecs)[0]
        roll  = 180*atan2(-R[2][1], R[2][2])/pi
        pitch = 180*asin(R[2][0])/pi
        yaw   = 180*atan2(-R[1][0], R[0][0])/pi

        rot_params = [roll,pitch,yaw]
        return rot_params

    def run(self, img, K, D):

        # setting parameter for qr-size
        qr_x = rospy.get_param("/qr_param_x", default=0.8)
        qr_y = rospy.get_param("/qr_param_y", default=0.4)

        # marker size in meter
        objp = np.array([[0, 0, 0.],  
                    [0, qr_x, 0],       
                    [qr_x, qr_x, 0],
                    [qr_x, 0, 0],
                    [qr_y, qr_y, 0]],np.float32)

        # image processing
        gray = np.array(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        thresh = 40
        img_bw = np.array(cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1])

        # Find QR codes
        decodedObjects = pyzbar.decode(img_bw)
       
        for decodedObject in decodedObjects:
            #print('Type : ', decodedObject.type)
            #print('Data : ', decodedObject.data,'\n')

            points = decodedObject.polygon
            img_bw, image_points = self.draw(points, img, decodedObject.type, decodedObject.data) 

            # Find the rotation and translation vectors.
            
            #print(K)
            #print(D)
            #print(image_points)
            #print(objp)
            
            if len(K)>1:
                ret,rvecs, tvecs = cv2.solvePnP(objp, image_points, K, D)    
           
                rot_params = self.rot_params_rv(rvecs)

                # project 3D points to image plane
                axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, K, D)
             
                cv2.putText(img, "x : " + str(round(tvecs[0][0],2)), (0,40), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "y : " + str(round(tvecs[1][0],2)), (0,80), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "z : " + str(round(tvecs[2][0],2)), (0,120), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "roll: " + str(round(rot_params[0],2)), (200,40), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "pitch: " + str(round(rot_params[1],2)), (200,80), 0, 1, (0, 0, 255), 2)
                cv2.putText(img, "yaw: " + str(round(rot_params[2],2)), (200,120), 0, 1, (0, 0, 255), 2)           
                img = self.draw_projection(img,image_points,imgpts)

                # publishing Pose in camera frame
                p = PoseStamped()
                p.header.frame_id = "front_realsense"
                p.header.stamp = rospy.Time.now()

                p.pose.position.x = round(tvecs[0][0],2)
                p.pose.position.y = round(tvecs[1][0],2)
                p.pose.position.z = round(tvecs[2][0],2)
                
                p.pose.orientation.x = round(rot_params[0],2)
                p.pose.orientation.y = round(rot_params[1],2)
                p.pose.orientation.z = round(rot_params[2],2)
                p.pose.orientation.w = 1.0

                self.pose_transformed = self.transform(p)

                self.pose_pub.publish(self.pose_transformed)
                
        return img;

    def transform(self, p):
        transform = self.tf_buffer.lookup_transform('base_link', p.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        self.pose_transformed = tf2_geometry_msgs.do_transform_pose(p, transform)

        return self.pose_transformed;
    
    def handle_get_pose(self, req):
        return GetQrPoseResponse(self.pose_transformed)       
    
def main(args):
    rospy.init_node("qr_detector_node", anonymous=True)
    qr_object = image_qr()
    service = rospy.Service('get_qrPose', GetQrPose, qr_object.handle_get_pose)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
