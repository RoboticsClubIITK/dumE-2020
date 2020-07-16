#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils


# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image

bridge = CvBridge()
x, y = 320, 200
def callback(depth):
    # cv_image_tmp = bridge.imgmsg_to_cv2(depth)
    # max_m_for_kinect = 5.0 ## You'll have to check out online for exact value 
    # cv_image_tmp = np.clip(cv_image_tmp,0.01,max_m_for_kinect) ## Filter all values > 5.0 m
    # scaling_factor = 65535/max_m_for_kinect 
    # cv_image_tmp = cv_image_tmp*scaling_factor #scaling the image to [0;65535]
    # depth_cv = np.array(cv_image_tmp,dtype=np.uint16) ## Creating the cv2 image
    
    try:
        depth_cv = bridge.imgmsg_to_cv2(depth, "32FC1")
    except CvBridgeError as e:
        print(e)

    cv2.circle(depth_cv, (x,y), 5, (255,255,255), 1)
    print(float(depth_cv[y,x]))
    cv2.imshow('test', depth_cv)
    cv2.waitKey(3)

def main():
    rospy.init_node('test_depth', anonymous=False)
    rospy.Subscriber('/kinect/depth/image_raw', Image, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException():
        cv2.destroyAllWindows()
        pass

Green:              Green:
 [[0.12459704]       [[0.12459716]        
 [0.0021526 ]        [0.0021526 ]        
 [0.5452702 ]        [0.5452701 ]        
 [1.        ]]       [1.        ]]        
Red:                Red:
 [[0.09115293]       [[0.09115364]        
 [0.00213097]        [0.00213097]        
 [0.3197117 ]        [0.31971154]        
 [1.        ]]       [1.        ]]        
Blue:               Blue:
 [[0.32829738]       [[0.32842216]        
 [0.00505033]        [0.00336724]        
 [0.24735081]        [0.24733089]        
 [1.        ]]       [1.        ]]        
Yellow:             Yellow:
 [[0.48580608]       [[0.48597902]        
 [0.00138012]        [0.00275759]        
 [0.17476091]        [0.17474401]        
 [1.        ]]       [1.        ]]              
