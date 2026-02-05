# scripts/cli/check_env.py — ROS environment checks

from pathlib import Path

import typer

from .common import PROJECT_ROOT, echo_error


def _check_ros_environment() -> None:
    ros_setup = Path("/opt/ros/humble/setup.bash")
    if not ros_setup.is_file():
        echo_error("ROS Humble not found: /opt/ros/humble/setup.bash")
        raise typer.Exit(1)
    install_setup = PROJECT_ROOT / "install" / "setup.bash"
    if not install_setup.is_file():
        echo_error("Missing install/setup.bash. Please build the workspace first.")
        raise typer.Exit(1)
