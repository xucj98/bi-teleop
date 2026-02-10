# bi-teleop

## Socket Refactor Plan (Master)
- Use a single class to manage ROS subscription, socket recv thread, and the main loop.
- Subscribe to the two master pose topics and keep the latest pose in member state.
- Start one blocking socket recv thread to parse frames and update member state:
  - On mode message, update local mode.
  - On joint message, store latest joint payload.
- Main thread runs a 100 Hz loop with a simple state machine:
  - If mode is takeover, send pose packets.
  - If mode switches from takeover -> autonomous, stop sending pose.
  - If mode switches from autonomous -> takeover, apply the latest slave joint state,
    then start sending pose.
- The joint-apply step can block (about 3 seconds); this is acceptable because pose
  sending should start only after the adjustment completes.
- No send lock is needed initially (single send path), but can be added later if
  multiple senders are introduced.

## Socket Refactor Plan (Slave)
- Use a single class for ROS subscription, socket recv thread, and the main loop.
- Socket recv thread only parses frames and updates member variables (no ROS actions).
- Main thread runs a 100 Hz loop and reads /slave_mode for behavior:
  - If mode changes, send a mode packet to the master.
  - If mode switches to takeover (== 2), send one joint packet to the master.
  - If mode == 2, publish the latest pose (from recv thread) to the arm command topics.
  - If mode != 2, do not publish pose (stop following).
