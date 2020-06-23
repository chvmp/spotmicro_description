#!/usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('send_motion', anonymous=True)
act_client = actionlib.SimpleActionClient(
    '/fullbody_controller/follow_joint_trajectory/', FollowJointTrajectoryAction)

act_client.wait_for_server()

rate = rospy.Rate(10)
# generate msg
traj_msg = FollowJointTrajectoryGoal()
traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
traj_msg.trajectory.joint_names = ['front_left_shoulder', 'front_left_leg', 'front_left_foot',
                                   'front_right_shoulder', 'front_right_leg', 'front_right_foot',
                                   'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot',
                                   'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot']

# reset pose
traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.9, 1.8,
                                                                  0.0, -0.9, 1.8,
                                                                  0.0, -0.9, 1.8,
                                                                  0.0, -0.9, 1.8],
                                                       time_from_start=rospy.Duration(1)))
while not rospy.is_shutdown():
    # MOVE base: front-left and rear-right
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.3, 1.8,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.9, 1.8],
                                                           time_from_start=rospy.Duration(2)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.3, 1.3,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.3, 1.8],
                                                           time_from_start=rospy.Duration(3)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.9, 1.8,
                                                                      0.0, -0.3, 1.8,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.3, 1.3],
                                                           time_from_start=rospy.Duration(4)))
    # MOVE base: front-left and rear-right
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.9, 1.8,
                                                                      0.0, -0.3, 1.3,
                                                                      0.0, -0.3, 1.8,
                                                                      0.0, -0.9, 1.8],
                                                           time_from_start=rospy.Duration(5)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, -0.9, 1.8,
                                                                      0.0, -0.9, 1.8,
                                                                      0.0, -0.3, 1.3,
                                                                      0.0, -0.9, 1.8],
                                                           time_from_start=rospy.Duration(6)))

    act_client.send_goal(traj_msg)

    act_client.wait_for_result()

    rospy.loginfo("done")
