#!/usr/bin/env python3

import numpy as np
from kalman import Kalman, randn

import rospy

from kinect_robo.msg import ArmPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

CAMERA_ORIGIN = [0, -0.2, 3.14159, 1.25, 0, 0.1]      # [r,p,y,x,y,z] (wrt world Frame) 
LOOP_RATE = 120

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
        self.x = dict()
        self.area = dict()

    def update(self, joint_names, joints, areas):
        for i in range(len(joint_names)):
            self.x[joint_names[i]] = np.matrix([[joints[i].x], [joints[i].y], [joints[i].z], [1.0]], dtype=np.float32)
            self.area[joint_names[i]] = areas[i]

    def clear(self):
        keys = [key for key in self.x]
        for key in keys:
            self.x.pop(key)
            self.area.pop(key)

class IK:
    def __init__(self):
        rospy.init_node("IK_node", anonymous=False)
        self.input_sub  = rospy.Subscriber("/vx250/input_arm_pose", ArmPose, self.callback)
        self.marker_pub = rospy.Publisher("/vx250/joint_state_est", Marker, queue_size=1)
        self.angles_pub = rospy.Publisher("/vx250/arm_controller/command", JointTrajectory, queue_size=1)

        self.measurement = ArmState()

        self.states = {'Green'  : Kalman(np.ones((6,1))/1000, np.diag([2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn()]), 1/LOOP_RATE),
                       'Red'    : Kalman(np.ones((6,1))/1000, np.diag([2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn()]), 1/LOOP_RATE),
                       'Blue'   : Kalman(np.ones((6,1))/1000, np.diag([2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn()]), 1/LOOP_RATE),
                       'Yellow' : Kalman(np.ones((6,1))/1000, np.diag([2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn()]), 1/LOOP_RATE)}
        self.prev_states = self.states

        self.rate = rospy.Rate(LOOP_RATE)

    def callback(self, arm_pose):
        self.measurement.update(arm_pose.joint_names, arm_pose.joints, arm_pose.areas)

        self.measurement = toWorldFrame(self.measurement)
        # _str = ''
        # for key in self.measurement.x:
        #     _str = _str + key + ":\n" + str(self.measurement.x[key]) +'\n' + '  Area = ' + str(self.measurement.area[key]) + '\n'
        # print(_str)

        temp = self.states

        for key in self.states:
            # try:
                # self.states[key].x[3:6, 0] = (self.states[key].x[0:3, 0] - self.prev_states[key].x[0:3, 0])/(1/LOOP_RATE)
            self.states[key].predict()
            # if key == 'Yellow':
            #     print('  States:\n', self.states[key].x, '\n  Prev States:\n', self.prev_states[key].x, '\n')
            # except TypeError:
            #     pass
            # input('predict ' + key)
            try:
                self.states[key].update(self.measurement.x[key], self.measurement.area[key])
            except KeyError:
                pass

        # _str = ''
        # for key in self.states:
        #     _str = _str + key + ":\n" + str(self.states[key].x) +'\n' + '  Trace of P = ' + str(np.trace(self.states[key].P)) + '\n'
        # print(_str)

        self.plotJoints()

        self.findJointAngles()
        self.publishAngles()

        self.measurement.clear()

        self.prev_states = temp

        self.rate.sleep()

    def plotJoints(self):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0

        for joint in self.states:
            point = Point()
            point.x, point.y, point.z = self.states[joint].x[0], self.states[joint].x[1], self.states[joint].x[2]

            marker.points.append(point)

        self.marker_pub.publish(marker)
        
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
        G_R = self.states['Red'].x[0:3, 0] - self.states['Green'].x[0:3,0]
        self.a_waist    = np.arctan2(G_R[1], G_R[0] + 1e-7) 

        G_R = self.states['Red'].x[0:3,0] - self.states['Green'].x[0:3,0] - (Rz(self.a_waist)*np.matrix([[0.05], [0], [0], [1]], dtype=np.float32))[0:3]        
        R_B = self.states['Blue'].x[0:3,0] - self.states['Red'].x[0:3,0]
        B_Y = self.states['Yellow'].x[0:3,0] - self.states['Blue'].x[0:3,0]

        theta = np.arctan(G_R[2]/(np.linalg.norm(G_R[0:2]) + 1e-7))
        alpha = np.arctan(R_B[2]/(np.linalg.norm(R_B[0:2]) + 1e-7))
        beta  = np.arctan2(B_Y[2], np.linalg.norm(B_Y[0:2]) + 1e-7)
        
        self.a_shoulder = np.pi/2 - theta
        self.a_elbow    = -(-np.pi/2 + theta - alpha)
        self.a_wrist    = -(alpha - beta)
        
        print('Angles:' + '\n  Waist:\t' + str(self.a_waist) + '\n  Shoulder:\t' + str(self.a_shoulder) + '\n  Elbow:\t' + str(self.a_elbow) + '\n  Wrist:\t' + str(self.a_wrist))
        pass

def toWorldFrame(state):
    for key in state.x:
        state.x[key] = T([CAMERA_ORIGIN[3], CAMERA_ORIGIN[4], CAMERA_ORIGIN[5]])*Rz(CAMERA_ORIGIN[2])*Ry(CAMERA_ORIGIN[1])*Rx(CAMERA_ORIGIN[0])*Ry(np.pi/2)*Rz(-np.pi/2)*state.x[key]
        state.x[key] = state.x[key][0:3, 0]
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