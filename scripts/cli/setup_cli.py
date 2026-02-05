# scripts/cli/setup_cmd.py — setup subcommands (SSH, etc.)

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import typer

from .common import echo_error, echo_info, echo_warning

setup_app = typer.Typer(help="Setup helpers (SSH, etc.).")


@setup_app.command("ssh")
def setup_ssh(
    host: str = typer.Option("10.10.20.126", help="Target host/IP."),
    key_path: Path = typer.Option(
        Path("~/.ssh/jetson_ed25519"),
        help="Private key path to create/use (public key will be <key_path>.pub).",
    ),
    dry_run: bool = typer.Option(
        False, "--dry-run", help="Print commands without executing."
    ),
    connect_timeout: int = typer.Option(
        5,
        "--connect-timeout",
        min=1,
        help="SSH connection timeout (seconds) for ssh-copy-id.",
    ),
) -> None:
    """
    Setup SSH key for the target.

    It runs (in order):
      1) ssh-keygen -t ed25519 -f <key_path>
      2) ssh-copy-id -i <key_path>.pub -o ConnectTimeout=<N> -o ConnectionAttempts=1 <host>
    """
    key_path = Path(str(key_path)).expanduser()
    ssh_dir = key_path.parent
    pub_path = Path(str(key_path) + ".pub")

    cmds: list[list[str]] = [
        ["ssh-keygen", "-t", "ed25519", "-f", str(key_path)],
        [
            "ssh-copy-id",
            "-i",
            str(pub_path),
            "-o",
            f"ConnectTimeout={connect_timeout}",
            "-o",
            "ConnectionAttempts=1",
            host,
        ],
    ]

    echo_info("Commands:")
    for c in cmds:
        typer.echo("  " + " ".join(c))

    if dry_run:
        return

    for exe in ("ssh-keygen", "ssh-copy-id"):
        if shutil.which(exe) is None:
            echo_error(f"{exe} not found in PATH")
            raise typer.Exit(1)

    # Ensure ~/.ssh exists and has sane permissions.
    ssh_dir.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(ssh_dir, 0o700)
    except OSError:
        # Best effort; don't fail on restrictive FS/ACL.
        pass

    try:
        # 1) keygen (interactive if passphrase is set)
        r1 = subprocess.run(cmds[0], check=False)
        if r1.returncode != 0:
            echo_error(f"ssh-keygen failed (exit {r1.returncode})")
            raise typer.Exit(r1.returncode or 1)
        if not pub_path.is_file():
            echo_error(f"Public key not found: {pub_path}")
            raise typer.Exit(1)

        # 2) copy-id (interactive: password prompt on first time)
        r2 = subprocess.run(cmds[1], check=False)
        if r2.returncode != 0:
            echo_error(f"ssh-copy-id failed (exit {r2.returncode})")
            raise typer.Exit(r2.returncode or 1)
    except KeyboardInterrupt:
        echo_warning("Interrupted by user (Ctrl+C)")
        raise typer.Exit(130)


def register(app: typer.Typer) -> None:
    """Register setup subcommands on the main app."""
    app.add_typer(setup_app, name="setup")
