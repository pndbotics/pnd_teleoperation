# scripts/cli/common.py — project root, output style, launch runner

import subprocess
import sys
from pathlib import Path

import typer

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


# ---------- 输出风格 ----------
def echo_info(msg: str) -> None:
    typer.echo(typer.style("[INFO] ", fg=typer.colors.BLUE) + msg)


def echo_success(msg: str) -> None:
    typer.echo(typer.style("[SUCCESS] ", fg=typer.colors.GREEN) + msg)


def echo_error(msg: str) -> None:
    typer.echo(typer.style("[ERROR] ", fg=typer.colors.RED) + msg)


def echo_warning(msg: str) -> None:
    typer.echo(typer.style("[WARNING] ", fg=typer.colors.YELLOW) + msg)


# ---------- ROS launch 执行 ----------
def run_launch_shell(launch_cmd: str) -> int:
    """Run a launch command in bash after sourcing ROS and workspace."""
    venv_activate = PROJECT_ROOT / ".venv" / "bin" / "activate"
    source_venv = f"source {venv_activate}" if venv_activate.is_file() else "true"
    shell_cmd = (
        "set -e; "
        "source /opt/ros/humble/setup.bash; "
        f"source {PROJECT_ROOT}/install/setup.bash; "
        "export ROS_LOCALHOST_ONLY=1; "
        f"{source_venv}; "
        f"exec {launch_cmd}"
    )
    return subprocess.run(
        ["bash", "-c", shell_cmd],
        cwd=str(PROJECT_ROOT),
    ).returncode


def echo_launch_and_exit(launch_cmd: str) -> None:
    """Print launch info and exit with the launch process return code."""
    echo_success("Environment ready")
    echo_info(f"Run: {launch_cmd}")
    echo_info("Press Ctrl+C to stop...")
    typer.echo()
    sys.exit(run_launch_shell(launch_cmd))
