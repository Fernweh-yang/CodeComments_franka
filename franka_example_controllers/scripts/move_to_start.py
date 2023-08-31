#!/usr/bin/env python

import sys
import rospy as ros

from actionlib import SimpleActionClient
# 都是ros在带的msg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint   
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

ros.init_node('move_to_start')

# 解析私有命名空间下的相对名称，获取用于通信的行动服务器名称。
action = ros.resolve_name('~follow_joint_trajectory')
# 创建一个行动客户端对象 client，用于与 FollowJointTrajectory 行动服务器通信。
client = SimpleActionClient(action, FollowJointTrajectoryAction)
ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
# 等待行动服务器启动并准备好进行通信。
client.wait_for_server()

# 解析私有命名空间下的相对参数名，获取机器人关节初始位置的参数。
param = ros.resolve_name('~joint_pose')
# 从参数服务器获取关节初始位置，存储在 pose 中
pose = ros.get_param(param, None)
if pose is None:
    ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
    sys.exit(1)

# 解析私有命名空间下的相对话题名，用于获取当前关节状态。
topic = ros.resolve_name('~joint_states')
ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
# 等待并获取关节状态消息。
joint_state = ros.wait_for_message(topic, JointState)
# 将当前关节状态存储在字典 initial_pose 中。
initial_pose = dict(zip(joint_state.name, joint_state.position))

# 计算关节初始位置与目标初始位置的最大位移。
max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

point = JointTrajectoryPoint()
# 设置运动时间。
point.time_from_start = ros.Duration.from_sec(
    # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    # whatever is greater
    max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
)

# 设置目标点
goal = FollowJointTrajectoryGoal()
#  设置关节名和位置，以及关节速度信息
goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
point.velocities = [0] * len(pose)
# 设置目标时间容差
goal.trajectory.points.append(point)
goal.goal_time_tolerance = ros.Duration.from_sec(0.5)
# 向行动服务器发送目标并等待其完成。
ros.loginfo('Sending trajectory Goal to move into initial config')
client.send_goal_and_wait(goal)

result = client.get_result()
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    ros.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL:
        """
        The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
        Is the 'joint_pose' reachable?
        """,

        FollowJointTrajectoryResult.INVALID_JOINTS:
        """
        The joint pose you specified is for different joints than the joint trajectory controller
        is claiming. Does you 'joint_pose' include all 7 joints of the robot?
        """,

        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        """
        During the motion the robot deviated from the planned path too much. Is something blocking
        the robot?
        """,

        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
        """
        After the motion the robot deviated from the desired goal pose too much. Probably the robot
        didn't reach the joint_pose properly
        """,
    }[result.error_code])

else:
    ros.loginfo('move_to_start: Successfully moved into start pose')
