# scripts/cli/launch_cli.py — launch command

from __future__ import annotations

import typer

from .caddy import _start_caddy_background
from .check_env import _check_ros_environment
from .common import echo_error, echo_info, echo_launch_and_exit
from .teleop_cmd import (
    _complete_launch_stem,
    _get_all_launch_stems,
    _get_launch_command_by_stem,
)


def launch(
    launch_stem: str = typer.Argument(
        ...,
        help="Launch file stem, e.g. pinocchio-adam_u-noitom, mink-adam_u-webvr. Tab-completable.",
        autocompletion=_complete_launch_stem,
    ),
) -> None:
    """
    Select and run a bringup launch file by stem (Tab lists all bringup launch files).

      pteleop launch pinocchio-adam_u-noitom
      pteleop launch test_retarget_vr
    """
    all_stems = _get_all_launch_stems()
    if not all_stems:
        echo_error(
            "No bringup launch files found. Build the workspace (colcon build) or check src/bringup/launch."
        )
        raise typer.Exit(1)

    launch_cmd = _get_launch_command_by_stem(launch_stem)
    if not launch_cmd:
        echo_error(f"Unknown launch: {launch_stem}")
        echo_info("Bringup launch files (stem):")
        for s in all_stems:
            typer.echo(f"  {s}")
        raise typer.Exit(1)

    echo_info("Checking ROS environment...")
    _check_ros_environment()
    if "mink-adam_u-webvr" in launch_stem:
        _start_caddy_background()

    echo_launch_and_exit(launch_cmd)


def register(app: typer.Typer) -> None:
    app.command()(launch)
