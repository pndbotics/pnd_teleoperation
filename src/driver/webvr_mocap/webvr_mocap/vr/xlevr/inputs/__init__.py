"""
Input providers for the teleoperation system.
Contains VR WebSocket server and keyboard listener implementations.
"""

from .base import ControlGoal
from .vr_ws_server import VRWebSocketServer

__all__ = [
    "VRWebSocketServer",
    "ControlGoal",
]
