#!/usr/bin/env python3

import numpy as np

import rospy

from kinect_robo.msg import ArmPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CAMERA_ORIGIN = [0, -0.2, 3.14159, 1.25, 0, 0.1]      # [r,p,y,x,y,z] (wrt world Frame) 

## Definitions of Transformation Matrices
def Rx(theta):
    rx = np.matrix([[1.0, 0.0,            0.0,           0.0], 
                    [0.0, np.cos(theta), -np.sin(theta), 0.0],
                    [0.0, np.sin(theta),  np.cos(theta), 0.0],
                    [0.0, 0.0,            0.0,           1.0]],
                   dtype=np.float32)
    return rx

def Ry(theta):
    ry = np.matrix([[ np.cos(theta), 0.0, np.sin(theta), 0.0], 
                    [ 0.0,           1.0, 0.0,           0.0],
                    [-np.sin(theta), 0.0, np.cos(theta), 0.0],
                    [ 0.0,           0.0, 0.0,           1.0]],
                   dtype=np.float32)
    return ry

def Rz(theta):
    rz = np.matrix([[np.cos(theta), -np.sin(theta), 0.0, 0.0], 
                    [np.sin(theta),  np.cos(theta), 0.0, 0.0],
                    [0.0,            0.0,           1.0, 0.0],
                    [0.0,            0.0,           0.0, 1.0]],
                   dtype=np.float32)
    return rz

def T(point):
    t = np.matrix([[1.0, 0.0, 0.0, point[0]],
                   [0.0, 1.0, 0.0, point[1]],
                   [0.0, 0.0, 1.0, point[2]],
                   [0.0, 0.0, 0.0, 1.0     ]],
                  dtype=np.float32)
    return t

## Definition of useful classes

class ArmState(dict):
    def __init__(self):
        self = dict()

    def update(self, joint_names, joints):
        for i in range(len(joint_names)):
            self[joint_names[i]] = np.matrix([[joints[i].x], [joints[i].y], [joints[i].z], [1.0]], dtype=np.float32)

    def clear(self):
        keys = [key for key in self]
        for key in keys:
            self.pop(key)

class IK:
    def __init__(self):
        rospy.init_node("IK_node", anonymous=False)
        self.input_sub  = rospy.Subscriber("/vx250/input_arm_pose", ArmPose, self.callback)
        self.angles_pub = rospy.Publisher("vx250/arm_controller/command", JointTrajectory, queue_size=1)

        self.state = ArmState()
        self.prev_state = ArmState()

        self.rate = rospy.Rate(30)


    def callback(self, arm_pose):
        self.state.update(arm_pose.joint_names, arm_pose.joints)

        self.state = toWorldFrame(self.state)
        # _str = ''
        # for key in self.state:
        #     _str = _str + key + ":\n" + str(self.state[key]) + '\n'
        # print(_str + '\n')

        self.findJointAngles()

        self.publishAngles()

        self.prev_state = self.state 
        self.state.clear()

        self.rate.sleep()

    def publishAngles(self):
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        
        trajectory.header.frame_id = "/base_link"

        trajectory.joint_names.append("waist")
        trajectory.joint_names.append("shoulder")
        trajectory.joint_names.append("elbow")
        trajectory.joint_names.append("wrist_angle")
        trajectory.joint_names.append("wrist_rotate")

        point.positions.append(self.a_waist)
        point.positions.append(self.a_shoulder)
        point.positions.append(self.a_elbow)
        point.positions.append(self.a_wrist)
        point.positions.append(0)
        point.time_from_start = rospy.Duration(0,1e8)

        trajectory.points.append(point)

        trajectory.header.stamp=rospy.Time.now()

        self.angles_pub.publish(trajectory)

    def findJointAngles(self):
        G_R = self.state['Red'] - self.state['Green']
        self.a_waist    = np.arctan2(G_R[1], G_R[0] + 1e-7) 

        G_R = self.state['Red'] - self.state['Green'] - Rz(self.a_waist)*np.matrix([[0.04], [0], [0], [1]], dtype=np.float32)        
        R_B = self.state['Blue'] - self.state['Red']
        B_Y = self.state['Yellow'] - self.state['Blue']

        theta = np.arctan(G_R[2]/(np.linalg.norm(G_R[0:2]) + 1e-7))
        alpha = np.arctan(R_B[2]/(np.linalg.norm(R_B[0:2]) + 1e-7))
        beta  = np.arctan2(B_Y[2], np.linalg.norm(B_Y[0:2]) + 1e-7)
        
        self.a_shoulder = np.pi/2 - theta
        self.a_elbow    = -(-np.pi/2 + theta - alpha)
        self.a_wrist    = -(alpha - beta)
        
        print('Angles:' + '\n  Waist:\t' + str(self.a_waist) + '\n  Shoulder:\t' + str(self.a_shoulder) + '\n  Elbow:\t' + str(self.a_elbow) + '\n  Wrist:\t' + str(self.a_wrist))
        pass

def toWorldFrame(state):
    for key in state:
        state[key] = T([CAMERA_ORIGIN[3], CAMERA_ORIGIN[4], CAMERA_ORIGIN[5]])*Rz(CAMERA_ORIGIN[2])*Ry(CAMERA_ORIGIN[1])*Rx(CAMERA_ORIGIN[0])*Ry(np.pi/2)*Rz(-np.pi/2)*state[key]

    # check = np.linalg.pinv(T([CAMERA_ORIGIN[3], CAMERA_ORIGIN[4], CAMERA_ORIGIN[5]])*Rz(CAMERA_ORIGIN[2])*Ry(CAMERA_ORIGIN[1])*Rx(CAMERA_ORIGIN[0])*Ry(np.pi/2)*Rz(-np.pi/2))*np.matrix([[0.000186], [0.212128], [0.700981], [1.0]], dtype=np.float32)
    # print(check, '\n\n')
    return state

def get_angles():
    ik_ = IK()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')

if __name__ == "__main__":
    try:
        get_angles()
    except rospy.ROSInterruptException():
        print('Shutting Down')
        pass