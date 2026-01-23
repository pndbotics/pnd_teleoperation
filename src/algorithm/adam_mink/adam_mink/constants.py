"""Constants for Adam robot joint and finger names."""

# Timing and control constants
DEFAULT_TIMER_PERIOD = 0.01  # 100 Hz
DEFAULT_JOINT_STATE_QUEUE_SIZE = 10
DEFAULT_MUJOCO_VIEWER_FREQUENCY = 200.0

# IK solver constants
DEFAULT_IK_SOLVER = "daqp"
DEFAULT_IK_DAMPING = 3e-1
DEFAULT_IK_ITER_MAX = 3
DEFAULT_IK_ERROR_THRESHOLD = 0.001
DEFAULT_IK_ITER_WARN_THRESHOLD = 0.8

# Joint and finger constants
DEFAULT_FINGER_POSITION = 1000.0
ROOT_POSE_NUM = 7  # x, y, z, qw, qx, qy, qz

# Visualization constants
DEFAULT_FRAME_SIZE = 0.1

# Left hand thumb rotate joints (base names)
_L_THUMB_ROTATE_JOINTS_BASE = [
    "L_thumb_MCP_joint1",
]

# Left hand trigger joints (base names)
_L_TRIGGER_JOINTS_BASE = [
    "L_thumb_MCP_joint2",
    "L_thumb_PIP_joint",
    "L_thumb_DIP_joint",
    "L_index_MCP_joint",
    "L_index_DIP_joint",
]

# Left hand grip joints (base names)
_L_GRIP_JOINTS_BASE = [
    "L_middle_MCP_joint",
    "L_middle_DIP_joint",
    "L_ring_MCP_joint",
    "L_ring_DIP_joint",
    "L_pinky_MCP_joint",
    "L_pinky_DIP_joint",
]

# Right hand thumb rotate joints (base names)
_R_THUMB_ROTATE_JOINTS_BASE = [
    "R_thumb_MCP_joint1",
]

# Right hand trigger joints (base names)
_R_TRIGGER_JOINTS_BASE = [
    "R_thumb_MCP_joint2",
    "R_thumb_PIP_joint",
    "R_thumb_DIP_joint",
    "R_index_MCP_joint",
    "R_index_DIP_joint",
]

# Right hand grip joints (base names)
_R_GRIP_JOINTS_BASE = [
    "R_middle_MCP_joint",
    "R_middle_DIP_joint",
    "R_ring_MCP_joint",
    "R_ring_DIP_joint",
    "R_pinky_MCP_joint",
    "R_pinky_DIP_joint",
]

# Joint names with dof_pos prefix
L_GRIP_JOINTS = [f"dof_pos/{name}" for name in _L_GRIP_JOINTS_BASE]
R_GRIP_JOINTS = [f"dof_pos/{name}" for name in _R_GRIP_JOINTS_BASE]
L_TRIGGER_JOINTS = [f"dof_pos/{name}" for name in _L_TRIGGER_JOINTS_BASE]
R_TRIGGER_JOINTS = [f"dof_pos/{name}" for name in _R_TRIGGER_JOINTS_BASE]
L_THUMB_ROTATE_JOINTS = [f"dof_pos/{name}" for name in _L_THUMB_ROTATE_JOINTS_BASE]
R_THUMB_ROTATE_JOINTS = [f"dof_pos/{name}" for name in _R_THUMB_ROTATE_JOINTS_BASE]

# Finger joint names
L_GRIP_FINGER = [
    "dof_pos/hand_pinky_Left",
    "dof_pos/hand_ring_Left",
    "dof_pos/hand_middle_Left",
]
R_GRIP_FINGER = [
    "dof_pos/hand_pinky_Right",
    "dof_pos/hand_ring_Right",
    "dof_pos/hand_middle_Right",
]
L_TRIGGER_FINGER = [
    "dof_pos/hand_index_Left",
    "dof_pos/hand_thumb_1_Left",
]
R_TRIGGER_FINGER = [
    "dof_pos/hand_index_Right",
    "dof_pos/hand_thumb_1_Right",
]
L_THUMB_ROTATE_FINGER = [
    "dof_pos/hand_thumb_2_Left",
]
R_THUMB_ROTATE_FINGER = [
    "dof_pos/hand_thumb_2_Right",
]

# All finger joints combined
ALL_FINGER = (
    L_GRIP_FINGER
    + L_TRIGGER_FINGER
    + L_THUMB_ROTATE_FINGER
    + R_GRIP_FINGER
    + R_TRIGGER_FINGER
    + R_THUMB_ROTATE_FINGER
)
