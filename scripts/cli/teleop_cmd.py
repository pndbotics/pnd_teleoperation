# scripts/cli/teleop_cmd.py — launch discovery, completion, command selection

import re
from pathlib import Path

import typer

from .common import PROJECT_ROOT, echo_error, echo_warning

_LAUNCH_MAP_CACHE: dict[tuple[str, str, str], str] | None = None

VALID_ADAM_TYPES = ["adam_sp", "adam_u", "adam_pro"]
VALID_MOCAP_DRIVERS = ["noitom", "zerolab", "webvr"]
VALID_ALGORITHMS = ["pinocchio", "mink"]


def _get_bringup_launch_dir() -> Path | None:
    """Return bringup launch dir: prefer install share, else src/bringup/launch."""
    install_launch = (
        PROJECT_ROOT / "install" / "bringup" / "share" / "bringup" / "launch"
    )
    if install_launch.is_dir():
        return install_launch
    src_launch = PROJECT_ROOT / "src" / "bringup" / "launch"
    return src_launch if src_launch.is_dir() else None


def _parse_launch_stem(stem: str) -> tuple[str, str, str] | None:
    """
    Parse launch file stem into (adam_type, mocap_driver, algorithm).
    Unified format: <algorithm>-<adam_type>-<mocap_driver>[-suffix], e.g. pinocchio-adam_u-noitom, mink-adam_u-webvr.
    """
    if stem.startswith("test_"):
        return None
    m = re.match(r"^([^-]+)-([^-]+)-([^-]+)(?:-.+)*$", stem)
    if not m:
        return None
    algorithm, adam_type, mocap_driver = m.group(1), m.group(2), m.group(3)
    return (adam_type, mocap_driver, algorithm)


def _discover_launch_map() -> dict[tuple[str, str, str], str]:
    """Discover (adam_type, mocap_driver, algorithm) -> ros2 launch command from bringup launch filenames."""
    global _LAUNCH_MAP_CACHE
    if _LAUNCH_MAP_CACHE is not None:
        return _LAUNCH_MAP_CACHE
    launch_dir = _get_bringup_launch_dir()
    result: dict[tuple[str, str, str], str] = {}
    if not launch_dir:
        _LAUNCH_MAP_CACHE = result
        return result

    # Process in deterministic order; prefer shorter stem when key clashes (e.g. mink-adam_u-webvr over mink-adam_u-webvr-sg)
    # Use .removesuffix: Path.stem only strips .py, leaving e.g. "pinocchio-adam_u-zerolab.launch"
    def _stem(p: Path) -> str:
        return p.name.removesuffix(".launch.py")

    files = sorted(
        launch_dir.glob("*.launch.py"), key=lambda p: (len(_stem(p)), _stem(p))
    )
    for path in files:
        key = _parse_launch_stem(_stem(path))
        if key is None or key in result:
            continue
        result[key] = f"ros2 launch bringup {path.name}"
    _LAUNCH_MAP_CACHE = result
    # typer.echo(typer.style("[INFO] ", fg=typer.colors.BLUE) + f"发现 {len(result)} 个 launch 文件:")
    # for (at, md, al), cmd in result.items():
    #     typer.echo(typer.style("  ", fg=typer.colors.YELLOW) + f"  {at} + {md} + {al} -> {cmd}")
    return result


def _get_valid_options_from_map() -> tuple[list[str], list[str], list[str]]:
    """Return (valid adam types, valid mocap drivers, valid algorithms) from discovered launch map."""
    m = _discover_launch_map()
    if not m:
        return (VALID_ADAM_TYPES, VALID_MOCAP_DRIVERS, VALID_ALGORITHMS)
    return (
        sorted({k[0] for k in m}),
        sorted({k[1] for k in m}),
        sorted({k[2] for k in m}),
    )


def _complete_adam(ctx: typer.Context, args: list[str], incomplete: str) -> list[str]:
    """Typer completion: only adam_type values present in launch files."""
    opts = _get_valid_options_from_map()[0]
    if not incomplete:
        return opts
    return [o for o in opts if o.startswith(incomplete)]


def _complete_mocap(ctx: typer.Context, args: list[str], incomplete: str) -> list[str]:
    """Typer completion: mocap_driver values filtered by selected adam_type (available in ctx.params)."""
    m = _discover_launch_map()
    if not m:
        opts = _get_valid_options_from_map()[1]
    else:
        adam = ctx.params.get("adam_type") if ctx.params else None
        if isinstance(adam, str) and adam:
            opts = sorted({k[1] for k in m if k[0] == adam})
            opts = opts if opts else _get_valid_options_from_map()[1]
        else:
            opts = _get_valid_options_from_map()[1]
    if not incomplete:
        return opts
    return [o for o in opts if o.startswith(incomplete)]


def _complete_algorithm(
    ctx: typer.Context, args: list[str], incomplete: str
) -> list[str]:
    """Typer completion: algorithm values filtered by selected adam_type + mocap_driver (from ctx.params)."""
    m = _discover_launch_map()
    if not m:
        opts = _get_valid_options_from_map()[2]
    else:
        adam = ctx.params.get("adam_type") if ctx.params else None
        mocap = ctx.params.get("mocap_driver") if ctx.params else None
        if isinstance(adam, str) and isinstance(mocap, str) and adam and mocap:
            opts = sorted({k[2] for k in m if k[0] == adam and k[1] == mocap})
            opts = opts if opts else sorted({k[2] for k in m if k[0] == adam})
        elif isinstance(adam, str) and adam:
            opts = sorted({k[2] for k in m if k[0] == adam})
        else:
            opts = _get_valid_options_from_map()[2]
        opts = opts if opts else _get_valid_options_from_map()[2]
    if not incomplete:
        return opts
    return [o for o in opts if o.startswith(incomplete)]


def _get_all_launch_stems() -> list[str]:
    """Return sorted list of all bringup launch file stems (all *.launch.py, including test_*)."""
    launch_dir = _get_bringup_launch_dir()
    if not launch_dir:
        return []
    stems = [
        p.name.removesuffix(".launch.py")
        for p in sorted(launch_dir.glob("*.launch.py"))
    ]
    return stems


def _get_launch_command_by_stem(stem: str) -> str | None:
    """Return ros2 launch command if stem.launch.py exists in bringup, else None."""
    launch_dir = _get_bringup_launch_dir()
    if not launch_dir:
        return None
    name = f"{stem}.launch.py"
    path = launch_dir / name
    return f"ros2 launch bringup {name}" if path.is_file() else None


def _complete_launch_stem(
    ctx: typer.Context, args: list[str], incomplete: str
) -> list[str]:
    """Typer completion: list all bringup launch file stems (including test_*)."""
    stems = _get_all_launch_stems()
    if not incomplete:
        return stems
    return [s for s in stems if s.startswith(incomplete)]


def _get_launch_command(adam_type: str, mocap_driver: str, algorithm: str) -> str:
    launch_map = _discover_launch_map()
    key = (adam_type, mocap_driver, algorithm)
    if not launch_map:
        echo_error(
            "No bringup launch files found. Build the workspace (colcon build) or check src/bringup/launch."
        )
        raise typer.Exit(1)
    if key not in launch_map:
        echo_error(
            f"Unsupported combination: {adam_type} + {mocap_driver} + {algorithm}"
        )
        echo_warning("Supported combinations:")
        for (at, md, al), _ in launch_map.items():
            typer.echo(f"  {at} + {md} + {al}")
        raise typer.Exit(1)
    return launch_map[key]
