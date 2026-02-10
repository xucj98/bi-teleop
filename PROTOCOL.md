# Bi-teleop Socket Protocol Notes

## Transport
- Use TCP sockets.
- Messages are binary.
- Each message is framed with a length prefix (TCP is a byte stream).

## Common Header
- Keep only version (no flags).
- Proposed header fields:
  - version: uint8
  - msg_type: uint8
  - seq: uint32
  - timestamp_us: uint64

## Message Types
### 1) Mode (slave -> master)
- Fields:
  - slave_mode: int8
- No reserved bytes.

### 2) Pose (master -> slave)
- Left pose: 7 floats
- Right pose: 7 floats

### 3) Joint (slave -> master)
- Fields:
  - joint_count: uint8 (default 7)
  - left: joint_count floats
  - right: joint_count floats
- No reserved bytes, no vel/cur.

## Rationale
- Binary reduces bandwidth and parsing overhead at 100-200 Hz.
- Length prefix keeps framing simple even for variable-length joint messages.
