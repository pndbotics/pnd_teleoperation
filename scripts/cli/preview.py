import atexit
import os
import signal
import subprocess
import sys

import typer

from .common import PROJECT_ROOT, echo_error, echo_info, echo_success

_preview_pid: int | None = None


def _kill_preview() -> None:
    """结束由本进程启动的 foxglove_bridge（包含其进程组）。"""
    global _preview_pid
    if _preview_pid is None:
        return
    pid = _preview_pid
    _preview_pid = None
    try:
        os.killpg(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        try:
            subprocess.run(
                ["sudo", "kill", "-TERM", f"-{pid}"],
                capture_output=True,
                timeout=2,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
    except Exception:
        # 避免在退出流程中因为清理失败而影响主逻辑
        pass


def _install_preview_cleanup_on_signal() -> None:
    """注册 SIGINT/SIGTERM 时退出，atexit 会负责清理 foxglove_bridge。"""

    def _handler(signum: int, frame: object) -> None:  # type: ignore[unused-argument]
        sys.exit(128 + (signum if signum < 128 else 0))

    try:
        signal.signal(signal.SIGINT, _handler)
    except (ValueError, OSError):
        pass
    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, OSError):
        pass


def _start_preview_background() -> None:
    """在后台启动 foxglove_bridge，用于 Foxglove Studio 预览。"""
    global _preview_pid
    if _preview_pid is not None:
        echo_info("Foxglove 预览已在后台运行")
        return

    echo_info("在后台启动 Foxglove 预览 (foxglove_bridge)...")

    venv_activate = PROJECT_ROOT / ".venv" / "bin" / "activate"
    source_venv = f"source {venv_activate}" if venv_activate.is_file() else "true"
    shell_cmd = (
        "set -e; "
        "source /opt/ros/humble/setup.bash; "
        f"source {PROJECT_ROOT}/install/setup.bash; "
        "export ROS_LOCALHOST_ONLY=1; "
        f"{source_venv}; "
        "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
    )

    try:
        proc = subprocess.Popen(
            ["bash", "-c", shell_cmd],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _preview_pid = proc.pid
        atexit.register(_kill_preview)
        _install_preview_cleanup_on_signal()
        echo_success(f"Foxglove 预览已在后台启动 (PID: {proc.pid})")
    except Exception as e:  # pragma: no cover - 防御性错误路径
        echo_error(f"启动 Foxglove 预览失败: {e}")
        raise typer.Exit(1)

