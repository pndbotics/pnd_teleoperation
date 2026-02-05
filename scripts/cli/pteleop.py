#!/usr/bin/env python3
"""
PND Retarget CLI.

Usage:
  pteleop                     # start teleop with defaults
  pteleop teleop [OPTIONS] [adam_type] [mocap_driver] [algorithm]
  pteleop launch <launch_stem> # launch a bringup launch file (Tab-completable)
  pteleop setup ssh            # setup SSH key and login
"""

import typer

from .launch_cli import register as _register_launch
from .setup_cli import register as _register_setup
from .teleop_cli import register as _register_teleop
from .teleop_cli import teleop

app = typer.Typer(
    name="pteleop",
    help="PND Retarget launcher: start nodes by ADAM type, mocap driver and algorithm.",
    add_completion=True,
    no_args_is_help=False,
)

_register_setup(app)
_register_teleop(app)
_register_launch(app)


@app.callback(invoke_without_command=True)
def main(ctx: typer.Context) -> None:
    """If no subcommand is given, start teleop with defaults."""
    if ctx.invoked_subcommand is None:
        ctx.invoke(
            teleop,
            adam_type=None,
            mocap_driver=None,
            algorithm=None,
            _adam_opt=None,
            _mocap_opt=None,
            _algo_opt=None,
        )


if __name__ == "__main__":
    app()
