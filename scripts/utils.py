import struct
import time

import rospy

VERSION = 1
MSG_MODE = 1
MSG_POSE = 2
MSG_JOINT = 3

HEADER_FMT = "<BBIQ"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def _read_exact(sock, size):
    chunks = []
    received = 0
    while received < size and not rospy.is_shutdown():
        data = sock.recv(size - received)
        if not data:
            return None
        chunks.append(data)
        received += len(data)
    return b"".join(chunks)


def recv_frame(sock):
    length_bytes = _read_exact(sock, 4)
    if length_bytes is None:
        return None
    (length,) = struct.unpack("<I", length_bytes)
    body = _read_exact(sock, length)
    if body is None:
        return None
    if len(body) < HEADER_SIZE:
        return {"header": None, "payload": None}

    header_bytes = body[:HEADER_SIZE]
    payload_bytes = body[HEADER_SIZE:]
    version, msg_type, seq, timestamp_us = struct.unpack(HEADER_FMT, header_bytes)
    header = {
        "version": version,
        "msg_type": msg_type,
        "seq": seq,
        "timestamp_us": timestamp_us,
    }
    if version != VERSION:
        return {"header": header, "payload": None}

    if msg_type == MSG_MODE:
        if len(payload_bytes) < 1:
            return {"header": header, "payload": None}
        (running_mode,) = struct.unpack("<b", payload_bytes[:1])
        return {"header": header, "payload": {"running_mode": int(running_mode)}}

    if msg_type == MSG_POSE:
        if len(payload_bytes) < 14 * 4:
            return {"header": header, "payload": None}
        values = struct.unpack("<" + "f" * 14, payload_bytes[:14 * 4])
        return {
            "header": header,
            "payload": {
                "pose_left": list(values[:7]),
                "pose_right": list(values[7:]),
            },
        }

    if msg_type == MSG_JOINT:
        if len(payload_bytes) < 1:
            return {"header": header, "payload": None}
        joint_count = payload_bytes[0]
        expected = 1 + joint_count * 4 * 2
        if len(payload_bytes) < expected:
            return {"header": header, "payload": None}
        offset = 1
        joint_left = struct.unpack(
            "<" + "f" * joint_count,
            payload_bytes[offset:offset + 4 * joint_count],
        )
        offset += 4 * joint_count
        joint_right = struct.unpack(
            "<" + "f" * joint_count,
            payload_bytes[offset:offset + 4 * joint_count],
        )
        return {
            "header": header,
            "payload": {
                "joint_count": joint_count,
                "joint_left": list(joint_left),
                "joint_right": list(joint_right),
            },
        }

    return {"header": header, "payload": None}


def send_frame(sock, data):
    header = data.get("header") or {}
    payload = data.get("payload") or {}

    version = header.get("version", VERSION)
    msg_type = header.get("msg_type")
    seq = header.get("seq", 0)
    timestamp_us = header.get("timestamp_us", int(time.time() * 1e6))

    if msg_type is None:
        raise ValueError("header.msg_type is required")

    if msg_type == MSG_MODE:
        running_mode = int(payload.get("running_mode", 0))
        payload_bytes = struct.pack("<b", running_mode)
    elif msg_type == MSG_POSE:
        pose_left = payload.get("pose_left") or []
        pose_right = payload.get("pose_right") or []
        if len(pose_left) != 7 or len(pose_right) != 7:
            raise ValueError("pose_left and pose_right must have 7 floats")
        payload_bytes = struct.pack("<" + "f" * 14, *(pose_left + pose_right))
    elif msg_type == MSG_JOINT:
        joint_left = payload.get("joint_left") or []
        joint_right = payload.get("joint_right") or []
        joint_count = payload.get("joint_count")
        if joint_count is None:
            joint_count = min(len(joint_left), len(joint_right))
        joint_count = int(min(joint_count, 255))
        if joint_count <= 0:
            raise ValueError("joint_count must be > 0")
        payload_bytes = struct.pack("<B", joint_count)
        payload_bytes += struct.pack("<" + "f" * joint_count, *joint_left[:joint_count])
        payload_bytes += struct.pack("<" + "f" * joint_count, *joint_right[:joint_count])
    else:
        raise ValueError("unknown msg_type")

    header_bytes = struct.pack(HEADER_FMT, version, msg_type, seq, timestamp_us)
    body = header_bytes + payload_bytes
    packet = struct.pack("<I", len(body)) + body
    sock.sendall(packet)
