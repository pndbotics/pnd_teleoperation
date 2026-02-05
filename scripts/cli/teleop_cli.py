# scripts/cli/teleop_cli.py — teleop command

from __future__ import annotations

import typer

from .caddy import _start_caddy_background
from .check_env import _check_ros_environment
from .common import echo_error, echo_info, echo_launch_and_exit
from .teleop_cmd import (
    _complete_adam,
    _complete_algorithm,
    _complete_mocap,
    _get_launch_command,
    _get_valid_options_from_map,
)


def teleop(
    adam_type: str | None = typer.Argument(
        None,
        help="Robot type (default: adam_pro). Choices are discovered from bringup launch filenames.",
        autocompletion=_complete_adam,
    ),
    mocap_driver: str | None = typer.Argument(
        None,
        help="Mocap driver (default: noitom). Choices are discovered from bringup launch filenames.",
        autocompletion=_complete_mocap,
    ),
    algorithm: str | None = typer.Argument(
        None,
        help="Algorithm (default: mink). Choices are discovered from bringup launch filenames.",
        autocompletion=_complete_algorithm,
    ),
    _adam_opt: str | None = typer.Option(None, "--adam-type", "-a", hidden=True),
    _mocap_opt: str | None = typer.Option(None, "--mocap", "-m", hidden=True),
    _algo_opt: str | None = typer.Option(None, "--algorithm", "-g", hidden=True),
) -> None:
    """
    Start teleoperation (PND Retarget).

      pteleop teleop                       # default: adam_pro + noitom + mink
      pteleop teleop adam_u                # adam_u + noitom + pinocchio
      pteleop teleop adam_pro zerolab      # adam_pro + zerolab + pinocchio
      pteleop teleop adam_u webvr mink     # adam_u + webvr + mink

    Options are also available: --adam-type / -a, --mocap / -m, --algorithm / -g
    """
    valid_adam, valid_mocap, valid_algo = _get_valid_options_from_map()
    adam_type = adam_type or _adam_opt or "adam_pro"
    mocap_driver = mocap_driver or _mocap_opt or "noitom"
    algorithm = algorithm or _algo_opt or "mink"

    if adam_type not in valid_adam:
        echo_error(f"Invalid adam_type: {adam_type}")
        typer.echo(f"Choices: {', '.join(valid_adam)}")
        raise typer.Exit(1)
    if mocap_driver not in valid_mocap:
        echo_error(f"Invalid mocap_driver: {mocap_driver}")
        typer.echo(f"Choices: {', '.join(valid_mocap)}")
        raise typer.Exit(1)
    if algorithm not in valid_algo:
        echo_error(f"Invalid algorithm: {algorithm}")
        typer.echo(f"Choices: {', '.join(valid_algo)}")
        raise typer.Exit(1)

    echo_info("Launch parameters:")
    echo_info(f"  ADAM Type: {adam_type}")
    echo_info(f"  Mocap Driver: {mocap_driver}")
    echo_info(f"  Algorithm: {algorithm}")
    echo_info("Checking ROS environment...")
    _check_ros_environment()

    launch_cmd = _get_launch_command(adam_type, mocap_driver, algorithm)
    if (adam_type, mocap_driver, algorithm) == ("adam_u", "webvr", "mink"):
        _start_caddy_background()

    echo_launch_and_exit(launch_cmd)


def register(app: typer.Typer) -> None:
    app.command()(teleop)
