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
from geometry_msgs.msg import Point
from kinect_robo.msg import ArmPose

# Camera Parameters
FOCAL_LENGTH = 554.254691

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
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_raw", Image, self.depth_update_cb)
        self.color_sub = rospy.Subscriber("/kinect/color/image_raw", Image, self.color_update_cb)
        self.joint_pub = rospy.Publisher("/vx250/input_arm_pose", ArmPose, queue_size=1)

        # Declare Joint colors and their HSV Ranges
        self.colors = [hsv_('Green', 31, 86, 6, 64, 255, 255),
                       hsv_('Red', 0, 70, 50, 10, 255, 255),
                       hsv_('Blue', 100, 150, 0, 140, 255, 255),
                       hsv_('Yellow', 20, 100, 100, 30, 255, 255)]
        
        # Declare container for all joint centers
        self.joints = []

    def publish(self):
        # Show tracked Image
        cv2.imshow('Track_Image',self.color_cv)
        # cv2.imshow('Raw_Depth',self.depth_cv)
        cv2.waitKey(3)

        # add publisher for points (centers)
        arm_pose = ArmPose()
        print('Centers:')
        for joint in self.joints:
            print("\t" + joint.name + ':\t[%0.4f, %0.4f, %0.4f]' %(joint.center[0], joint.center[1], joint.center[2]))
            
            # Add point object to Message
            joint_msg = Point()
            joint_msg.x, joint_msg.y, joint_msg.z = joint.center[0], joint.center[1], joint.center[2]
            
            arm_pose.joint_names.append(joint.name)
            arm_pose.joints.append(joint_msg)

        # Publish Message
        self.joint_pub.publish(arm_pose)

        # Clear joint container for next iteration
        self.joints = []
    
    def get_centers(self):
        # Convert BGR image to HSV image
        hsv = cv2.cvtColor(self.color_cv, cv2.COLOR_BGR2HSV)
        
        # Run detection loop for all colors
        for val in self.colors:
            # Create mask corresponding to detected color
            mask = cv2.inRange(hsv, val.min, val.max)
            
            # Get Contours(boundary) for the mask
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            # Filter contours found
            filtered = []
            M = []
            for contour in cnts:
                m = cv2.moments(contour)
                if m["m00"] > 0:
                    M.append(m)
                    filtered.append(contour)

            if len(filtered) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(filtered, key=cv2.contourArea)
                ((x_c, y_c), radius) = cv2.minEnclosingCircle(c)

                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(self.color_cv, (int(x_c), int(y_c)), int(radius), (0, 255, 255), 2)
                cv2.circle(self.color_cv, (int(x_c), int(y_c)), 2, (255, 255, 255), -1)
            
            X, Y, Z, total_area = 0, 0, 0, 0
            (rows, cols, _) = self.color_cv.shape
            
            # Sum over all contours
            for i in range(len(M)):
                x = int(M[i]["m10"] / (M[i]["m00"] + 1e-7))
                y = int(M[i]["m01"] / (M[i]["m00"] + 1e-7))
                ## Note that indexing of image is different from CV2 
                z = float(self.depth_cv[y,x])
                if z == np.nan:
                    z = 1e5
                X += float((x - cols/2)*z/FOCAL_LENGTH)*M[i]["m00"]
                Y += float((y - rows/2)*z/FOCAL_LENGTH)*M[i]["m00"]
                Z += z * M[i]["m00"]
                total_area += M[i]["m00"] 
            
            # Take average value of entire contour
            X = X/(total_area + 1e-7)
            Y = Y/(total_area + 1e-7)
            Z = Z/(total_area + 1e-7)
            
            # Print Coordinates on Image
            center_str = '(%0.3f, %0.3f, %0.3f)' %(X,Y,Z)
            cv2.putText(self.color_cv, center_str, (int(x_c), int(y_c)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add center found to joints container only if countour is detected
            if Z > 0:
                center = [X,Y,Z]            
                joint = joints(val.name, center)
                self.joints.append(joint)


    def color_update_cb(self, color):
        # Callback function of subscribed topic. 
        # Here images get converted

        #### direct conversion to CV2 ####
        # print(str(color.encoding))
        try:
            color_cv = self.bridge.imgmsg_to_cv2(color, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, _) = color_cv.shape
        if rows > 0 and cols > 0:
            self.color_cv = color_cv
        
        self.get_centers()
        self.publish()

    def depth_update_cb(self, depth):
        # Callback function of subscribed topic. 
        # Here images get converted

        #### direct conversion to CV2 ####
        # print(str(depth.encoding))
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)

        (rows, cols) = depth_cv.shape
        if rows > 0 and cols > 0:
            self.depth_cv = depth_cv

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