#!/usr/bin/env python3
import rospy
from arm_control.msg import JointControl, JointInformation
from collections import deque

joint1 = deque(maxlen=7)
joint2 = None


def _to_cmd(msg):
    out = JointControl()
    out.joint_pos = list(msg.joint_pos)
    out.joint_vel = list(msg.joint_vel)
    out.joint_cur = list(msg.joint_cur)
    out.mode = msg.mode
    return out


def main():
    rospy.init_node("gravity_comp_joint_raw")

    pub1 = rospy.Publisher("/master_joint_control", JointControl, queue_size=10)
    pub2 = rospy.Publisher("/master_joint_control2", JointControl, queue_size=10)

    def cb1(msg):
        joint1.append(msg.joint_pos)
        if len(joint1) == 7:
            print([y - x for x, y in zip(joint1[0], msg.joint_pos)])
            msg.joint_pos = [z + 0.8 * (z * 2 - x - y) for x, y, z in zip(joint1[0], joint1[3], joint1[6])]
        pub1.publish(_to_cmd(msg))

    def cb2(msg):
        pub2.publish(_to_cmd(msg))

    rospy.Subscriber("/joint_control", JointInformation, cb1, queue_size=10)
    rospy.Subscriber("/joint_control2", JointInformation, cb2, queue_size=10)

    rospy.loginfo("gravity_comp_joint_raw running: passthrough joint_control -> master_joint_control")
    rospy.spin()


if __name__ == "__main__":
    main()
