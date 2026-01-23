#!/usr/bin/env python3
"""Noitom-based Adam Pro robot inverse kinematics node using Mink solver."""

import math
from typing import List, Dict, Tuple

import numpy as np

# Type alias for mocap data
MocapData = Dict[str, Tuple[np.ndarray, np.ndarray]]
import rclpy
from scipy.spatial.transform import Rotation as R
from adam_mink.adam_mink_base import AdamMinkBase
from adam_mink.constants import ROOT_POSE_NUM
from adam_mink.utils import quat_mul_single

# Constants for finger mapping
DEFAULT_FINGER_LOWER_LIMIT = 0.0
DEFAULT_FINGER_UPPER_LIMIT_THUMB_1 = 1.2
DEFAULT_FINGER_UPPER_LIMIT_THUMB_2 = 0.5
DEFAULT_FINGER_UPPER_LIMIT_OTHER = math.pi
DEFAULT_FINGER_MAPPING_SCALE = 1000.0


class AdamMinkProNode(AdamMinkBase):
    """Noitom-based Adam Pro robot IK node without joystick control."""

    def __init__(self) -> None:
        """Initialize the Pro node."""
        super().__init__("adam_mink_pro")
        self.finger_handle_param = self._create_finger_handle_params()
        self.rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
        self.rotation_matrix_T = self.rotation_matrix.T
        self.rotation_quat = R.from_matrix(self.rotation_matrix).as_quat(scalar_first=True)
        self.calibrated = True
        self.get_logger().info("Adam Mink Pro IK node initialized.")

    def _create_finger_handle_params(self) -> List[Tuple]:
        """Create finger handle parameter configuration.
        
        Returns:
            List of tuples containing (finger_idx, joints_idx, lower_limit, upper_limit).
        """
        # Finger mapping configuration: (hand_name, joint_names, upper_limit)
        finger_configs = [
            # Left hand
            ("hand_thumb_2_Left", ["L_thumb_MCP_joint1"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_THUMB_2),
            ("hand_thumb_1_Left", ["L_thumb_MCP_joint2", "L_thumb_PIP_joint", "L_thumb_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_THUMB_1),
            ("hand_index_Left", ["L_index_MCP_joint", "L_index_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_middle_Left", ["L_middle_MCP_joint", "L_middle_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT,DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_ring_Left", ["L_ring_MCP_joint", "L_ring_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT,DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_pinky_Left", ["L_pinky_MCP_joint", "L_pinky_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT,DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            # Right hand
            ("hand_thumb_2_Right", ["R_thumb_MCP_joint1"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_THUMB_2),
            ("hand_thumb_1_Right", ["R_thumb_MCP_joint2", "R_thumb_PIP_joint", "R_thumb_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_THUMB_1),
            ("hand_index_Right", ["R_index_MCP_joint", "R_index_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_middle_Right", ["R_middle_MCP_joint", "R_middle_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_ring_Right", ["R_ring_MCP_joint", "R_ring_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_OTHER),
            ("hand_pinky_Right", ["R_pinky_MCP_joint", "R_pinky_DIP_joint"], DEFAULT_FINGER_LOWER_LIMIT, DEFAULT_FINGER_UPPER_LIMIT_OTHER),
        ]
        
        params = []
        for hand_name, joint_names, lower_limit, upper_limit in finger_configs:
            finger_idx = self.all_joint.get(f"dof_pos/{hand_name}")
            joints_idx = [self.all_joint.get(f"dof_pos/{name}") for name in joint_names]
            params.append((
                finger_idx,
                joints_idx,
                lower_limit,
                upper_limit,
            ))
        return params

    def get_bone_frames(self) -> List[str]:
        """Get the list of bone frame names to track for Noitom."""
        return [
            "Hips",
            "RightUpLeg",
            "RightLeg",
            "RightFoot",
            "LeftUpLeg",
            "LeftLeg",
            "LeftFoot",
            # "Spine",
            # "Spine1",
            "Spine2",
            # "Neck",
            # "Neck1",
            "Head",
            # "RightShoulder",
            "RightArm",
            "RightForeArm",
            "RightHand",
            "RightHandThumb1",
            "RightHandThumb2",
            "RightHandThumb3",
            # "RightInHandIndex",
            "RightHandIndex1",
            "RightHandIndex2",
            # "RightHandIndex3",
            # "RightInHandMiddle",
            "RightHandMiddle1",
            "RightHandMiddle2",
            # "RightHandMiddle3",
            # "RightInHandRing",
            "RightHandRing1",
            "RightHandRing2",
            # "RightHandRing3",
            # "RightInHandPinky",
            "RightHandPinky1",
            "RightHandPinky2",
            # "RightHandPinky3",
            # "LeftShoulder",
            "LeftArm",
            "LeftForeArm",
            "LeftHand",
            "LeftHandThumb1",
            "LeftHandThumb2",
            "LeftHandThumb3",
            # "LeftInHandIndex",
            "LeftHandIndex1",
            "LeftHandIndex2",
            # "LeftHandIndex3",
            # "LeftInHandMiddle",
            "LeftHandMiddle1",
            "LeftHandMiddle2",
            # "LeftHandMiddle3",
            # "LeftInHandRing",
            "LeftHandRing1",
            "LeftHandRing2",
            # "LeftHandRing3",
            # "LeftInHandPinky",
            "LeftHandPinky1",
            "LeftHandPinky2",
            # "LeftHandPinky3",
        ]

    def offset_mocap_data(self, data: MocapData) -> None:
        for name, (pos, rot) in data.items():
            rotated_pos = pos @ self.rotation_matrix_T
            rotated_quat = quat_mul_single(self.rotation_quat, rot)
            data[name] = (rotated_pos, rotated_quat)

        super().offset_mocap_data(data)
    
    def update_joint_states(self) -> None:
        """Update finger joint states based on mapped joint angles.
        
        This method converts joint angles to finger control values using
        the finger_handle_param configuration.
        """
        for (
            finger_idx,
            joints_idx,
            lower_limit,
            upper_limit,
        ) in self.finger_handle_param:
            joints_radian = 0.0
            for joint_idx in joints_idx:
                if joint_idx is not None:
                    joints_radian += self.joint_state_msg.position[
                        ROOT_POSE_NUM + joint_idx
                    ]
            if finger_idx is not None:
                self.joint_state_msg.position[ROOT_POSE_NUM + finger_idx] = (
                    self.convert_finger(joints_radian, lower_limit, upper_limit)
                )

    def convert_finger(
        self, radian: float, lower_limit: float, upper_limit: float
    ) -> float:
        """Convert joint angle (radian) to finger control value.
        
        Args:
            radian: Joint angle in radians.
            lower_limit: Lower limit for the joint angle.
            upper_limit: Upper limit for the joint angle.
            
        Returns:
            Finger control value mapped to [0, 1000] range.
        """
        # Clamp radian to valid range
        if radian < lower_limit:
            radian = lower_limit
        elif radian > upper_limit:
            radian = upper_limit
        
        # Map from [lower_limit, upper_limit] to [1000, 0]
        if upper_limit == lower_limit:
            return DEFAULT_FINGER_MAPPING_SCALE
        
        mapped_value = DEFAULT_FINGER_MAPPING_SCALE - int(
            ((radian - lower_limit) / (upper_limit - lower_limit))
            * DEFAULT_FINGER_MAPPING_SCALE
        )
        return float(mapped_value)


def main(args=None) -> None:
    """Main entry point for the Pro node."""
    rclpy.init(args=args)

    node = AdamMinkProNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
