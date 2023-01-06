#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,cos,sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)



waypoints = []
# scale = 10
# wpose = group.get_current_pose().pose
# wpose.position.z = 0 #scale * 0.1  # First move up (z)
# wpose.position.y = 0.85 #scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.x = 0 #scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
# #(plan, fraction) = group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold
pose_goal = geometry_msgs.msg.Pose()
theta = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180]
r = 0.15
#group.get_current_pose().pose
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.206
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.978
pose_goal.position.x = 0.5
pose_goal.position.y = 0.0
pose_goal.position.z = 0.3
waypoints.append(copy.deepcopy(pose_goal))

theta_0 = [180,170,160,150,140.130,120,110,100,90,80,70,60,50,40,30,20,10,0]
size = len(theta)
flag = 0
p = 0.5
t = 0.04
for k in range(6):
    for i in range(len(theta)):
        
        pose_goal.position.x = p
        if(flag==0):
            pose_goal.position.y = r*cos(theta[i]*(pi/180))
            pose_goal.position.z = 0.3 + r*sin(theta[i]*(pi/180))
        else:
            pose_goal.position.y = r*cos(theta[size - i-1]*(pi/180))
            pose_goal.position.z = 0.3 + r*sin(theta[size - i-1]*(pi/180))
        waypoints.append(copy.deepcopy(pose_goal))
    if(flag==0):
        flag = 1
    else:
        flag = 0
    p = p+t

# for i in theta_0:
#     pose_goal.position.x = 0.6
#     pose_goal.position.y = r*cos(i*(pi/180))
#     pose_goal.position.z = 0.3 + r*sin(i*(pi/180))
#     waypoints.append(copy.deepcopy(pose_goal))

# for i in theta:
#     pose_goal.position.x = 0.7
#     pose_goal.position.y = r*cos(i*(pi/180))
#     pose_goal.position.z = 0.3 + r*sin(i*(pi/180))
#     waypoints.append(copy.deepcopy(pose_goal))

(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

#pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.x = 1.0
# pose_goal.orientation.y = 1.0
# pose_goal.orientation.z = 2.0
# pose_goal.orientation.w = 2.0
# pose_goal.position.x = 0.5
# pose_goal.position.y = 0.0
# pose_goal.position.z = 0.3
# group.set_pose_target(pose_goal)
group.execute(plan, wait=True)
#plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

rospy.is_shutdown()






























# import rospy
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryActionGoal


# rospy.init_node('task_space_goal_to_go', anonymous=False)

# pub = rospy.Publisher('/eff_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

# joint_traj_msg = FollowJointTrajectoryActionGoal()

# pos = [-1.57,0.0,1.57, 0.0, 0.0, 0.0]
# joint_traj_msg.goal.trajectory.points = [JointTrajectoryPoint(positions = pos)] 
# joint_traj_msg.goal.trajectory.points = [JointTrajectoryPoint(velocities = [0.4,0.4,0.4,0.4,0.4,0.4])]
# pub.publish(joint_traj_msg)
# print("msg published")
# rospy.spin()