#!/usr/bin/env python3
import argparse
import socket
import threading
import time

import rospy
from arm_control.msg import JointInformation
from communicationPort.msg import PosCmd

from utils import MSG_JOINT, MSG_MODE, MSG_POSE, VERSION, recv_frame, send_frame

def _to_pos_cmd(values):
    msg = PosCmd()
    msg.x = float(values[0])
    msg.y = float(values[1])
    msg.z = float(values[2])
    msg.roll = float(values[3])
    msg.pitch = float(values[4])
    msg.yaw = float(values[5])
    msg.gripper = float(values[6])
    msg.mode1 = 0
    msg.mode2 = 0
    return msg


class BiTeleopSlave:
    def __init__(
        self,
        host,
        port,
        slave_pose_cmd_left_topic,
        slave_pose_cmd_right_topic,
        slave_joint_left_topic,
        slave_joint_right_topic,
    ):
        rospy.set_param("/running_mode", 0)
        rospy.loginfo("Bi teleop slave start, set /running_mode to 0.")

        self._host = host
        self._port = port

        self._lock = threading.Lock()
        self._master_pose_left = [0.] * 7
        self._master_pose_right = [0.] * 7
        self._slave_joint_left = [0.] * 7
        self._slave_joint_right = [0.] * 7
        self._running_mode = 1
        self._take_over_time = time.time()
        
        self._seq = 0
        self._sock = None
        self._recv_thread = None

        self._slave_pose_cmd_left_pub = rospy.Publisher(
            slave_pose_cmd_left_topic,
            PosCmd,
            queue_size=10,
        )
        self._slave_pose_cmd_right_pub = rospy.Publisher(
            slave_pose_cmd_right_topic,
            PosCmd,
            queue_size=10,
        )
        self._slave_joint_left_sub = rospy.Subscriber(
            slave_joint_left_topic,
            JointInformation,
            self._on_left_joint,
        )
        self._slave_joint_right_sub = rospy.Subscriber(
            slave_joint_right_topic,
            JointInformation,
            self._on_right_joint,
        )

    def _on_left_joint(self, msg):
        with self._lock:
            self._slave_joint_left = msg

    def _on_right_joint(self, msg):
        with self._lock:
            self._slave_joint_right = msg

    def start(self):
        self._bind_and_accept()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def _bind_and_accept(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._host, self._port))
        server.listen(1)
        rospy.loginfo("Listening on %s:%s", self._host, self._port)
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = conn
        rospy.loginfo("Client connected: %s:%s", addr[0], addr[1])

    def _recv_loop(self):
        while not rospy.is_shutdown():
            data = recv_frame(self._sock)
            if data is None:
                break
            header = data.get("header")
            payload = data.get("payload")
            if not header or payload is None:
                continue
            msg_type = header.get("msg_type")
            if msg_type == MSG_MODE:
                with self._lock:
                    self._running_mode = int(payload.get("running_mode", 1))
            elif msg_type == MSG_POSE:
                with self._lock:
                    self._master_pose_left = payload.get("pose_left")
                    self._master_pose_right = payload.get("pose_right")
        rospy.logwarn("Socket recv loop ended")

    def _send_message(self, msg_type, payload):
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        header = {
            "version": VERSION,
            "msg_type": msg_type,
            "seq": self._seq,
            "timestamp_us": int(time.time() * 1e6),
        }
        send_frame(self._sock, {"header": header, "payload": payload})

    @staticmethod
    def _get_running_mode():
        return int(rospy.get_param("/running_mode", 1))

    def run(self, send_hz):
        self.start()
        rate = rospy.Rate(send_hz)
        last_mode = None
        while not rospy.is_shutdown():
            mode = self._get_running_mode()
            if last_mode is None or mode != last_mode:
                payload = {"running_mode": mode}
                self._send_message(MSG_MODE, payload)
            if last_mode != mode and mode == 2:
                with self._lock:
                    slave_joint_left = self._slave_joint_left
                    slave_joint_right = self._slave_joint_right
                if slave_joint_left is not None and slave_joint_right is not None:
                    payload = {
                        "joint_left": slave_joint_left.joint_pos,
                        "joint_right": slave_joint_right.joint_pos,
                    }
                    self._send_message(MSG_JOINT, payload)
                self._take_over_time = time.time()
            if mode == 2 and time.time() - self._take_over_time > 4.0:
                with self._lock:
                    master_pose_left = self._master_pose_left
                    master_pose_right = self._master_pose_right
                self._slave_pose_cmd_left_pub.publish(_to_pos_cmd(master_pose_left))
                self._slave_pose_cmd_right_pub.publish(_to_pos_cmd(master_pose_right))
            last_mode = mode
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Slave TCP teleop server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host")
    parser.add_argument("--port", type=int, default=8765, help="Bind port")
    parser.add_argument("--left-pose-cmd-topic", default="/follow_pos_cmd_1", help="Left pose command topic")
    parser.add_argument("--right-pose-cmd-topic", default="/follow_pos_cmd_2", help="Right pose command topic")
    parser.add_argument("--left-joint-topic", default="/joint_information", help="Left joint state topic")
    parser.add_argument("--right-joint-topic", default="/joint_information2", help="Right joint state topic")
    parser.add_argument("--send-hz", type=float, default=100.0, help="Loop frequency")
    args = parser.parse_args()

    rospy.init_node("bi_teleop_slave_socket")

    slave = BiTeleopSlave(
        args.host,
        args.port,
        args.left_pose_cmd_topic,
        args.right_pose_cmd_topic,
        args.left_joint_topic,
        args.right_joint_topic,
    )
    slave.run(args.send_hz)


if __name__ == "__main__":
    main()
