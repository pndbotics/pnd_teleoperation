# scripts/cli/caddy.py — background Caddy with cleanup

import atexit
import os
import shutil
import signal
import subprocess
import sys

import typer

from .common import PROJECT_ROOT, echo_error, echo_info, echo_success

# 由本进程启动的 Caddy 的 PID（用于退出时清理）
_caddy_pid: int | None = None


def _check_caddy() -> bool:
    return shutil.which("caddy") is not None


def _kill_caddy() -> None:
    """退出时终止由本进程启动的 Caddy（含其进程组）。"""
    global _caddy_pid
    if _caddy_pid is None:
        return
    pid = _caddy_pid
    _caddy_pid = None
    try:
        # 杀进程组（Caddy 可能由 start_caddy.sh 在子进程里启动）
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
        pass


def _install_caddy_cleanup_on_signal() -> None:
    """注册 SIGINT/SIGTERM 时退出，atexit 会负责清理 Caddy。"""

    def _handler(signum: int, frame: object) -> None:
        sys.exit(128 + (signum if signum < 128 else 0))

    try:
        signal.signal(signal.SIGINT, _handler)
    except (ValueError, OSError):
        pass
    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, OSError):
        pass


def _start_caddy_background() -> None:
    if not _check_caddy():
        echo_error("Caddy is not installed or not in PATH")
        typer.echo(
            "Install docs: https://caddyserver.com/docs/install#debian-ubuntu-raspbian"
        )
        raise typer.Exit(1)

    echo_info("Starting Caddy in background...")
    global _caddy_pid
    caddy_script = PROJECT_ROOT / "scripts" / "start_caddy.sh"
    if not caddy_script.is_file():
        echo_error(f"Caddy script not found: {caddy_script}")
        raise typer.Exit(1)
    try:
        proc = subprocess.Popen(
            ["sudo", "bash", str(caddy_script)],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _caddy_pid = proc.pid
        atexit.register(_kill_caddy)
        _install_caddy_cleanup_on_signal()
        echo_success(f"Caddy started in background (PID: {proc.pid})")
    except Exception as e:
        echo_error(f"Failed to start Caddy: {e}")
        raise typer.Exit(1)
