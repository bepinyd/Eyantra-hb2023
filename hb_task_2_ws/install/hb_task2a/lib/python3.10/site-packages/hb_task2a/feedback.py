#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np
import math
from cv_bridge import CvBridge


# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.subscriber=self.create_subscription(Image,"/camera/image_raw",self.image_callback,10)
        self.publishA=self.create_publisher(Pose2D,"/detected_aruco",10)
        # Subscribe the topic /camera/image_raw
        self.cv_bridge=CvBridge()
     
   

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict)

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i, marker_id in enumerate(ids):
               if marker_id[0] == 1:
                # Get the corners of the marker with the target ID
                corner = corners[i][0]
                centerX = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
                centerY = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
                center = (int(centerX), int(centerY))
                dist1x=corner[1][0]-centerX
                dist1y=corner[1][1]-centerY
                angle=math.atan2(centerY,centerX)-0.79
                pub=Pose2D()
                pub.x=centerX
                pub.y=centerY
                pub.theta=angle
                print(angle)
                self.publishA.publish(pub)
          
        cv2.imshow("ArUco Markers", cv_image)
        cv2.waitKey(1)

        #convert ROS image to opencv image
        #Detect Aruco marker
        # Publish the bot coordinates to the topic  /detected_aruco

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
