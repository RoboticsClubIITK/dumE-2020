#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm():
    rospy.init_node('arm_controller_custom', anonymous=False)
    topic_name = '/arm_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    trajectory=JointTrajectory()
    point=JointTrajectoryPoint()
    rate = rospy.Rate(30)
    
    trajectory.header.frame_id = "/base_link"

    trajectory.joint_names.append("waist")
    trajectory.joint_names.append("shoulder")
    trajectory.joint_names.append("elbow")
    trajectory.joint_names.append("wrist_angle")
    trajectory.joint_names.append("wrist_rotate")

    point.positions.append(1.0)
    point.positions.append(0.3)
    point.positions.append(0.5)
    point.positions.append(0.6)
    point.positions.append(0.5)
    point.time_from_start.secs = rospy.Time.now().secs + 1

    trajectory.points.append(point)

    while not rospy.is_shutdown():
        trajectory.header.stamp=rospy.Time.now()

        pub.publish(trajectory)
        rate.sleep()

if __name__ == "__main__":
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass