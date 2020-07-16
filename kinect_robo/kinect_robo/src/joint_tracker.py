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

class joints:
    def __init__(self, name, center):
        self.name = name
        self.center = center

class hsv_:
    def __init__(self, name, h_min, s_min, v_min, h_max, s_max, v_max):
        self.name = name
        self.min = np.array([h_min, s_min, v_min])
        self.max = np.array([h_max, s_max, v_max])

class arm_track:
    def __init__(self):
        print("Joint Tracker initialised\n")
        
        # Declare CVBridge instance to take care of image transport in ROS
        self.bridge = CvBridge()
        
        # Declare ROS Nodes and Topics
        rospy.init_node("joint_tracker", anonymous=False)
        self.img_sub = rospy.Subscriber("/kinect/color/image_raw", Image, self.callback)

        # Declare Joint Colours and their HSV Ranges
        self.colours = [hsv_('Green', 31, 86, 6, 64, 255, 255),
                        hsv_('Red', 0, 70, 50, 10, 255, 255),
                        hsv_('Blue', 100, 150, 0, 140, 255, 255),
                        hsv_('Yellow', 20, 100, 100, 30, 255, 255)]
        
        # Declare container for all joint centers
        self.joints = []

    def publish(self):
        # Show tracked Image
        cv2.imshow('Track_Image',self.image_cv)
        cv2.waitKey(3)

        # add publisher for points (centers)
        print('Centers:')
        for joint in self.joints:
            print("\t" + joint.name + ':\t' + str(joint.center))

        # Clear joint container for next iteration
        self.joints = []
    
    def get_centers(self):
        # # To increase chances of successful detection
        # blurred = cv2.GaussianBlur(self.image_cv, (11, 11), 0)
	    
        # Convert BGR image to HSV image
        hsv = cv2.cvtColor(self.image_cv, cv2.COLOR_BGR2HSV)
        
        # Run detection loop for all colours
        for val in self.colours:
            # Create mask corresponding to detected colour
            mask = cv2.inRange(hsv, val.min, val.max)
            # mask = cv2.erode(mask, None, iterations=2)
            # mask = cv2.dilate(mask, None, iterations=2)
            
            # Show Mask image of each colour
            cv2.imshow("Mask_"+val.name, mask) 
            
            # Get Contours(boundary) for the mask
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(self.image_cv, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(self.image_cv, (int(x), int(y)), 5, (255, 255, 255), -1)
                    # Add center of mask to container or you can add center of circle
                    joint = joints(val.name, center)
                    self.joints.append(joint)

    def callback(self, image):
        # Callback function of subscribed topic. 
        # Here images get converted

        #### direct conversion to CV2 ####
        try:
            image_cv = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, _) = image_cv.shape
        if rows > 0 and cols > 0:
            self.image_cv = image_cv
            cv2.imshow('Raw_Image',image_cv)
            cv2.waitKey(3)
        
        self.get_centers()
        self.publish()

def track():
    at = arm_track()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        track()
    except rospy.ROSInterruptException():
        print("Shutting down")
        cv2.destroyAllWindows()
        pass