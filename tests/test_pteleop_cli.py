import types

import pytest
from typer.testing import CliRunner


@pytest.fixture()
def runner():
    return CliRunner()


@pytest.fixture()
def pteleop(monkeypatch):
    # import module and ensure we can monkeypatch its dependencies
    from scripts.cli import pteleop as m

    return m


def test_no_args_invokes_teleop_default(runner, pteleop, monkeypatch):
    # Arrange: make teleop run without touching ROS/subprocess
    from scripts.cli import common, teleop_cli

    monkeypatch.setattr(teleop_cli, "_check_ros_environment", lambda: None)
    monkeypatch.setattr(
        teleop_cli,
        "_get_valid_options_from_map",
        lambda: (["adam_pro"], ["noitom"], ["mink"]),
    )
    monkeypatch.setattr(
        teleop_cli,
        "_get_launch_command",
        lambda a, m, g: "ros2 launch bringup fake.launch.py",
    )
    monkeypatch.setattr(common, "run_launch_shell", lambda cmd: 0)

    # Act
    res = runner.invoke(pteleop.app, [])

    # Assert
    assert res.exit_code == 0
    assert "Run: ros2 launch bringup fake.launch.py" in res.stdout


def test_teleop_invalid_adam_type_exits(runner, pteleop, monkeypatch):
    from scripts.cli import teleop_cli

    monkeypatch.setattr(
        teleop_cli,
        "_get_valid_options_from_map",
        lambda: (["adam_sp"], ["noitom"], ["mink"]),
    )
    res = runner.invoke(pteleop.app, ["teleop", "bad_adam"])
    assert res.exit_code != 0
    assert "Invalid adam_type" in res.stdout


def test_teleop_invalid_mocap_driver_exits(runner, pteleop, monkeypatch):
    from scripts.cli import teleop_cli

    monkeypatch.setattr(
        teleop_cli,
        "_get_valid_options_from_map",
        lambda: (["adam_sp"], ["noitom"], ["mink"]),
    )
    res = runner.invoke(pteleop.app, ["teleop", "adam_sp", "bad_mocap"])
    assert res.exit_code != 0
    assert "Invalid mocap_driver" in res.stdout


def test_teleop_invalid_algorithm_exits(runner, pteleop, monkeypatch):
    from scripts.cli import teleop_cli

    monkeypatch.setattr(
        teleop_cli,
        "_get_valid_options_from_map",
        lambda: (["adam_sp"], ["noitom"], ["mink"]),
    )
    res = runner.invoke(pteleop.app, ["teleop", "adam_sp", "noitom", "bad_algo"])
    assert res.exit_code != 0
    assert "Invalid algorithm" in res.stdout


def test_teleop_starts_caddy_for_adam_u_webvr_mink(runner, pteleop, monkeypatch):
    from scripts.cli import common, teleop_cli

    monkeypatch.setattr(teleop_cli, "_check_ros_environment", lambda: None)
    monkeypatch.setattr(
        teleop_cli,
        "_get_valid_options_from_map",
        lambda: (["adam_u"], ["webvr"], ["mink"]),
    )
    monkeypatch.setattr(
        teleop_cli,
        "_get_launch_command",
        lambda a, m, g: "ros2 launch bringup mink-adam_u-webvr.launch.py",
    )
    monkeypatch.setattr(common, "run_launch_shell", lambda cmd: 0)

    called = {"caddy": 0}

    def _caddy():
        called["caddy"] += 1

    monkeypatch.setattr(teleop_cli, "_start_caddy_background", _caddy)

    res = runner.invoke(pteleop.app, ["teleop", "adam_u", "webvr", "mink"])
    assert res.exit_code == 0
    assert called["caddy"] == 1


def test_launch_lists_all_stems_on_unknown(runner, pteleop, monkeypatch):
    from scripts.cli import launch_cli

    monkeypatch.setattr(launch_cli, "_get_all_launch_stems", lambda: ["a", "b"])
    monkeypatch.setattr(launch_cli, "_get_launch_command_by_stem", lambda stem: None)
    res = runner.invoke(pteleop.app, ["launch", "nope"])
    assert res.exit_code != 0
    assert "Unknown launch" in res.stdout
    assert "a" in res.stdout and "b" in res.stdout


def test_launch_runs_selected_stem(runner, pteleop, monkeypatch):
    from scripts.cli import common, launch_cli

    monkeypatch.setattr(launch_cli, "_check_ros_environment", lambda: None)
    monkeypatch.setattr(
        launch_cli, "_get_all_launch_stems", lambda: ["test_retarget_vr"]
    )
    monkeypatch.setattr(
        launch_cli,
        "_get_launch_command_by_stem",
        lambda stem: "ros2 launch bringup test_retarget_vr.launch.py",
    )
    monkeypatch.setattr(common, "run_launch_shell", lambda cmd: 0)

    res = runner.invoke(pteleop.app, ["launch", "test_retarget_vr"])
    assert res.exit_code == 0
    assert "Run: ros2 launch bringup test_retarget_vr.launch.py" in res.stdout


def test_launch_starts_caddy_for_mink_adam_u_webvr(runner, pteleop, monkeypatch):
    from scripts.cli import common, launch_cli

    monkeypatch.setattr(launch_cli, "_check_ros_environment", lambda: None)
    monkeypatch.setattr(
        launch_cli, "_get_all_launch_stems", lambda: ["mink-adam_u-webvr"]
    )
    monkeypatch.setattr(
        launch_cli,
        "_get_launch_command_by_stem",
        lambda stem: "ros2 launch bringup mink-adam_u-webvr.launch.py",
    )
    monkeypatch.setattr(common, "run_launch_shell", lambda cmd: 0)

    called = {"caddy": 0}

    def _caddy():
        called["caddy"] += 1

    monkeypatch.setattr(launch_cli, "_start_caddy_background", _caddy)

    res = runner.invoke(pteleop.app, ["launch", "mink-adam_u-webvr"])
    assert res.exit_code == 0
    assert called["caddy"] == 1


def test_setup_ssh_dry_run_prints_commands(runner, pteleop, tmp_path):
    key_path = tmp_path / "jetson_ed25519"
    res = runner.invoke(
        pteleop.app,
        [
            "setup",
            "ssh",
            "--host",
            "10.10.20.126",
            "--key-path",
            str(key_path),
            "--dry-run",
        ],
    )
    assert res.exit_code == 0
    assert "ssh-keygen -t ed25519 -f" in res.stdout
    assert (
        f"ssh-copy-id -i {key_path}.pub -o ConnectTimeout=5 -o ConnectionAttempts=1 10.10.20.126"
        in res.stdout
    )


def test_setup_ssh_runs_two_commands(runner, pteleop, monkeypatch, tmp_path):
    key_path = tmp_path / "jetson_ed25519"
    # setup_ssh validates that <key_path>.pub exists after ssh-keygen step
    (tmp_path / "jetson_ed25519.pub").write_text("dummy\n")

    from scripts.cli import setup_cmd

    # pretend executables exist
    monkeypatch.setattr(setup_cmd.shutil, "which", lambda exe: f"/usr/bin/{exe}")

    calls: list[list[str]] = []

    def _run(cmd, check=False):
        calls.append(list(cmd))
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(setup_cmd.subprocess, "run", _run)

    res = runner.invoke(
        pteleop.app,
        ["setup", "ssh", "--host", "10.10.20.126", "--key-path", str(key_path)],
    )
    assert res.exit_code == 0
    assert calls == [
        ["ssh-keygen", "-t", "ed25519", "-f", str(key_path)],
        [
            "ssh-copy-id",
            "-i",
            f"{key_path}.pub",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "ConnectionAttempts=1",
            "10.10.20.126",
        ],
    ]
