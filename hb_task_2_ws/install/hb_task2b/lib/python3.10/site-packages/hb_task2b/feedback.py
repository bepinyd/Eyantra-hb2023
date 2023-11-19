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
        self.publishA=self.create_publisher(Pose2D,"/detected_aruco_1",10)
        self.publishB=self.create_publisher(Pose2D,"/detected_aruco_2",10)
        self.publishC=self.create_publisher(Pose2D,"/detected_aruco_3",10)
        # Subscribe the topic /camera/image_raw
        self.cv_bridge=CvBridge()
        self.index=0
   

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict)
        target_id=1
        if len(ids)>5 :
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            


                                       ############### corner 1 ################
            for i, marker_id in enumerate(ids):
               if marker_id[0] == 1:
                # Get the corners of the marker with the target ID
                corner1 = corners[i]

                centerX1 = (corner1[0][0][0] + corner1[0][1][0] + corner1[0][2][0] + corner1[0][3][0]) / 4
                centerY1 = (corner1[0][0][1] + corner1[0][1][1] + corner1[0][2][1] + corner1[0][3][1]) / 4
                center1 = (int(centerX1), int(centerY1))
                dist1x=corner1[0][0][0]-corner1[0][3][0]
                dist1y=corner1[0][0][1]-corner1[0][3][1]
                angle1=math.atan2(dist1y,dist1x)
            
                pub=Pose2D()
                pub.x=centerX1
                pub.y=centerY1
                pub.theta=angle1
                self.publishA.publish(pub)
                                       ############### corner 2 ################
            for i, marker_id in enumerate(ids):
               if marker_id[0] == 2:
                # Get the corners of the marker with the target ID
                corner2 = corners[i]  
                centerX2 = (corner2[0][0][0] + corner2[0][1][0] + corner2[0][2][0] + corner2[0][3][0]) / 4
                centerY2 = (corner2[0][0][1] + corner2[0][1][1] + corner2[0][2][1] + corner2[0][3][1]) / 4
                center2 = (int(centerX2), int(centerY2))
                dist2x=corner2[0][1][0]-corner2[0][0][0]
                dist2y=corner2[0][1][1]-corner2[0][0][1]
                angle2=math.atan2(dist2y,dist2x)
                
                pub1=Pose2D()
                pub1.x=centerX2
                pub1.y=centerY2
                pub1.theta=angle2
                self.publishB.publish(pub1)
                                       ############### corner 3 ################
            for i, marker_id in enumerate(ids):
               if marker_id[0] == 3:
                # Get the corners of the marker with the target ID
                corner3 = corners[i]  

                centerX3 = (corner3[0][0][0] + corner3[0][1][0] + corner3[0][2][0] + corner3[0][3][0]) / 4
                centerY3 = (corner3[0][0][1] + corner3[0][1][1] + corner3[0][2][1] + corner3[0][3][1]) / 4
                center3 = (int(centerX3), int(centerY3))
                dist3x=corner3[0][1][0]-corner3[0][0][0]
                dist3y=corner3[0][1][1]-corner3[0][0][1]
                angle3=math.atan2(dist3y,dist3x)
                # print(center3)
                pub2=Pose2D()
                pub2.x=centerX3
                pub2.y=centerY3
                pub2.theta=angle3
                self.publishC.publish(pub2)
              
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
