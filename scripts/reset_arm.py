#!/usr/bin/env python3
import rospy
from arm_control.msg import PosCmd, JointControl, JointInformation


def _wait_current_pose(topic, timeout=5.0):
    try:
        msg = rospy.wait_for_message(topic, PosCmd, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for %s", topic)
        return None
    return [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.gripper]


def _interpolate(start, target, steps):
    for i in range(1, steps + 1):
        ratio = float(i) / float(steps)
        yield [s + (t - s) * ratio for s, t in zip(start, target)]


def _to_msg(values):
    msg = PosCmd()
    msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.gripper = values
    msg.mode1 = 0
    msg.mode2 = 0
    return msg


def _wait_current_joint(topic, timeout=5.0):
    try:
        msg = rospy.wait_for_message(topic, JointInformation, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for %s", topic)
        return None
    return msg


def _to_joint_cmd(info_msg, joint_pos):
    cmd = JointControl()
    cmd.joint_pos = list(joint_pos)
    cmd.joint_vel = list(info_msg.joint_vel)
    cmd.joint_cur = list(info_msg.joint_cur)
    cmd.mode = info_msg.mode
    return cmd


def main():
    rospy.init_node("reset_arm_safe")
    rate = rospy.Rate(50)
    master_mode = rospy.get_param("/master_mode", 1)

    if master_mode == 3:
        pub1 = rospy.Publisher('/master_pos_cmd_1', PosCmd, queue_size=10)
        pub2 = rospy.Publisher('/master_pos_cmd_2', PosCmd, queue_size=10)

        start1 = _wait_current_pose('/master1_pos_back')
        start2 = _wait_current_pose('/master2_pos_back')
        if start1 is None or start2 is None:
            rospy.logerr("Failed to get current poses, aborting reset.")
            return

        start1[6] = 5.0  # Open gripper
        start2[6] = 5.0  # Open gripper
        pub1.publish(_to_msg(start1))
        pub2.publish(_to_msg(start2))
        rospy.sleep(1.0)  # Allow time for gripper to open

        target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]
        steps = 75
        rospy.loginfo("Resetting arms to %s in %d steps (~%.1fs)", target, steps, steps / 50.0)

        for pose1, pose2 in zip(_interpolate(start1, target, steps), _interpolate(start2, target, steps)):
            pub1.publish(_to_msg(pose1))
            pub2.publish(_to_msg(pose2))
            rate.sleep()

        target[-1] = 0.0
        pub1.publish(_to_msg(target))  # close gripper
        pub2.publish(_to_msg(target))  # close gripper

        rospy.loginfo("reset_arm complete (position control).")
   
    elif master_mode == 2:
        pub1 = rospy.Publisher('/master_joint_control', JointControl, queue_size=10)
        pub2 = rospy.Publisher('/master_joint_control2', JointControl, queue_size=10)

        info1 = _wait_current_joint('/joint_control')
        info2 = _wait_current_joint('/joint_control2')
        if info1 is None or info2 is None:
            rospy.logerr("Failed to get current joints, aborting reset.")
            return

        start1 = list(info1.joint_pos)
        start2 = list(info2.joint_pos)
        target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]
        steps = 75
        rospy.loginfo("Resetting joints to %s in %d steps (~%.1fs)", target, steps, steps / 50.0)

        for pos1, pos2 in zip(_interpolate(start1, target, steps), _interpolate(start2, target, steps)):
            pub1.publish(_to_joint_cmd(info1, pos1))
            pub2.publish(_to_joint_cmd(info2, pos2))
            rate.sleep()

        target[-1] = 0.0
        pub1.publish(_to_joint_cmd(info1, target))
        pub2.publish(_to_joint_cmd(info2, target))
        
        rospy.loginfo("reset_arm complete (joint control).")
   
    else:
        rospy.logwarn("Unknown /master_mode=%s; defaulting to position control.", str(master_mode))


if __name__ == "__main__":
    main()
