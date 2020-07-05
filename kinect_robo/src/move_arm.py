#!/usr/bin/env python
from numpy import linspace
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm():
    rospy.init_node('arm_controller_custom', anonymous=False)
    
    topic_name = 'custom/arm_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)

    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        # trajectory=JointTrajectory()
        # point=JointTrajectoryPoint()
        
        # trajectory.header.frame_id = "/base_link"

        # trajectory.joint_names.append("waist")
        # trajectory.joint_names.append("shoulder")
        # trajectory.joint_names.append("elbow")
        # trajectory.joint_names.append("wrist_angle")
        # trajectory.joint_names.append("wrist_rotate")

        # print('Enter Value for angles:')
        # a = float(input('  Waist:\t'))
        # b = float(input('  Shoulder:\t'))
        # c = float(input('  Elbow:\t'))
        # d = float(input('  Wrist Angle:\t'))

        for a in linspace(0, 3.14, num=5):
            for b in linspace(0, 3.14, num=5):
                for c in linspace(0, 3.14, num=5):
                    for d in linspace(0, 3.14, num=5):
                        trajectory=JointTrajectory()
                        point=JointTrajectoryPoint()
                        
                        trajectory.header.frame_id = "/base_link"

                        trajectory.joint_names.append("waist")
                        trajectory.joint_names.append("shoulder")
                        trajectory.joint_names.append("elbow")
                        trajectory.joint_names.append("wrist_angle")
                        trajectory.joint_names.append("wrist_rotate")
                        point.positions.append(a)
                        point.positions.append(b)
                        point.positions.append(c)
                        point.positions.append(d)
                        point.positions.append(0.0)
                        point.time_from_start = rospy.Duration(0, 1e8)

                        trajectory.points.append(point)

                        trajectory.header.stamp=rospy.Time.now()

                        pub.publish(trajectory)
                        rate.sleep()

if __name__ == "__main__":
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass