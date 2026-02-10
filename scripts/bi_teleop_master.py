#!/usr/bin/env python3
import argparse
import socket
import threading
import time

import rospy
from arm_control.msg import JointControl, JointInformation, PosCmd

from utils import MSG_JOINT, MSG_MODE, MSG_POSE, VERSION, recv_frame, send_frame


class BiTeleopMaster:
    def __init__(
        self,
        server_host,
        server_port,
        master_pose_left_topic,
        master_pose_right_topic,
        master_joint_cmd_left_topic,
        master_joint_cmd_right_topic,
        master_joint_state_left_topic,
        master_joint_state_right_topic,
    ):
        self._server_host = server_host
        self._server_port = server_port
        self._master_pose_left_topic = master_pose_left_topic
        self._master_pose_right_topic = master_pose_right_topic
        self._master_joint_state_left_topic = master_joint_state_left_topic
        self._master_joint_state_right_topic = master_joint_state_right_topic

        self._lock = threading.Lock()
        self._master_pose_left = [0.] * 7
        self._master_pose_right = [0.] * 7
        self._slave_joint_left = [0.] * 7
        self._slave_joint_right = [0.] * 7
        self._running_mode = 1

        self._sock = None
        self._recv_thread = None
        self._seq = 0
        self._sock_lock = threading.Lock()

        self._master_joint_cmd_left_pub = rospy.Publisher(
            master_joint_cmd_left_topic,
            JointControl,
            queue_size=10,
        )
        self._master_joint_cmd_right_pub = rospy.Publisher(
            master_joint_cmd_right_topic,
            JointControl,
            queue_size=10,
        )

    def start(self):
        rospy.Subscriber(self._master_pose_left_topic, PosCmd, lambda msg: self._master_pose_cb(msg, "left"))
        rospy.Subscriber(self._master_pose_right_topic, PosCmd, lambda msg: self._master_pose_cb(msg, "right"))
        self._connect()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def _master_pose_cb(self, msg, side):
        pose = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.gripper]
        with self._lock:
            if side == "left":
                self._master_pose_left = pose
            else:
                self._master_pose_right = pose

    def _connect(self):
        while not rospy.is_shutdown():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._server_host, self._server_port))
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError as exc:
                rospy.logwarn("Connect failed: %s", exc)
                time.sleep(1.0)
                continue
            with self._sock_lock:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                self._sock = sock
            rospy.loginfo("Connected to %s:%s", self._server_host, self._server_port)
            return

    def _recv_loop(self):
        while not rospy.is_shutdown():
            with self._sock_lock:
                sock = self._sock
            if sock is None:
                time.sleep(0.1)
                continue
            data = recv_frame(sock)
            if data is None:
                rospy.logwarn("Socket recv error; reconnecting")
                self._connect()
                continue
            header = data.get("header")
            payload = data.get("payload")
            if not header or payload is None:
                continue
            msg_type = header.get("msg_type")
            if msg_type == MSG_MODE:
                with self._lock:
                    self._running_mode = int(payload.get("running_mode", 1))
            elif msg_type == MSG_JOINT:
                with self._lock:
                    self._slave_joint_left = payload.get("joint_left")
                    self._slave_joint_right = payload.get("joint_right")
        rospy.logwarn("Socket recv loop ended")
    
    def _apply_slave_joint_state(self, slave_joint_left, slave_joint_right):
        def _interpolate(start, target, steps):
            for i in range(1, steps + 1):
                ratio = float(i) / float(steps)
                yield [s + (t - s) * ratio for s, t in zip(start, target)]
    
        left_info = rospy.wait_for_message(self._master_joint_state_left_topic, JointInformation, timeout=3.0)
        right_info = rospy.wait_for_message(self._master_joint_state_right_topic, JointInformation, timeout=3.0)
        
        left_start = list(left_info.joint_pos)
        right_start = list(right_info.joint_pos)
        steps = 75
        rate = rospy.Rate(50)
        for left_pos, right_pos in zip(
            _interpolate(left_start, slave_joint_left, steps),
            _interpolate(right_start, slave_joint_right, steps),
        ):
            left_info.joint_pos = left_pos
            right_info.joint_pos = right_pos
            self._master_joint_cmd_left_pub.publish(left_info)
            self._master_joint_cmd_right_pub.publish(right_info)
            rate.sleep()

    def run(self, send_hz):
        self.start()
        rate = rospy.Rate(send_hz)
        last_mode = None
        while not rospy.is_shutdown():
            with self._lock:
                mode = self._running_mode
                
            if last_mode != mode and mode == 2:
                rospy.loginfo("Takeover detected; applying slave joint state")
                rospy.set_param("/master_mode", 2)
                rospy.sleep(1.0)

                with self._lock:
                    slave_joint_left = self._slave_joint_left
                    slave_joint_right = self._slave_joint_right
                self._apply_slave_joint_state(slave_joint_left, slave_joint_right)
                rospy.sleep(1.0)
                
                rospy.set_param("/master_mode", 1)
                rospy.loginfo("Takeover start; /master_mode set to 1")

            elif mode == 2:
                self._seq = (self._seq + 1) & 0xFFFFFFFF
                header = {
                    "version": VERSION,
                    "msg_type": MSG_POSE,
                    "seq": self._seq,
                    "timestamp_us": int(time.time() * 1e6),
                }
                with self._lock:
                    payload = {
                        "pose_left": self._master_pose_left, 
                        "pose_right": self._master_pose_right,
                    }
                with self._sock_lock:
                    sock = self._sock
                if sock is not None:
                    try:
                        send_frame(sock, {"header": header, "payload": payload})
                    except OSError as exc:
                        rospy.logwarn("Send failed: %s", exc)
             
            last_mode = mode
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Master TCP teleop client")
    parser.add_argument("--server-host", default="192.168.110.83", help="Slave server host")
    parser.add_argument("--server-port", type=int, default=8765, help="Slave server port")
    parser.add_argument("--send-rate", type=float, default=100.0, help="Send rate in Hz")
    parser.add_argument("--pose-left-topic", default="/master1_pos_back", help="Left arm topic")
    parser.add_argument("--pose-right-topic", default="/master2_pos_back", help="Right arm topic")
    parser.add_argument("--joint-cmd-left-topic", default="/master_joint_control", help="Left joint command topic")
    parser.add_argument("--joint-cmd-right-topic", default="/master_joint_control2", help="Right joint command topic")
    parser.add_argument("--joint-left-topic", default="/joint_control", help="Left joint state topic")
    parser.add_argument("--joint-right-topic", default="/joint_control2", help="Right joint state topic")
    args = parser.parse_args()

    rospy.init_node("bi_teleop_master_socket")

    master = BiTeleopMaster(
        args.server_host,
        args.server_port,
        args.pose_left_topic,
        args.pose_right_topic,
        args.joint_cmd_left_topic,
        args.joint_cmd_right_topic,
        args.joint_left_topic,
        args.joint_right_topic,
    )
    master.run(args.send_rate)


if __name__ == "__main__":
    main()
