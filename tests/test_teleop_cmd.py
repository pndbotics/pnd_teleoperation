import types

import pytest
import typer


@pytest.fixture()
def teleop_cmd(monkeypatch):
    """
    Import module fresh-ish and ensure launch map cache cleared.
    """
    from scripts.cli import teleop_cmd as m

    # clear cache between tests
    monkeypatch.setattr(m, "_LAUNCH_MAP_CACHE", None, raising=False)
    return m


def _mk_launch_files(tmp_path, names):
    for n in names:
        (tmp_path / n).write_text("# dummy\n")


def test_get_all_launch_stems_lists_all_launch_files(teleop_cmd, monkeypatch, tmp_path):
    _mk_launch_files(
        tmp_path,
        [
            "pinocchio-adam_sp-noitom.launch.py",
            "mink-adam_u-webvr.launch.py",
            "test_retarget_vr.launch.py",
        ],
    )
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)

    stems = teleop_cmd._get_all_launch_stems()
    assert stems == [
        "mink-adam_u-webvr",
        "pinocchio-adam_sp-noitom",
        "test_retarget_vr",
    ]


def test_get_launch_command_by_stem_returns_none_if_missing(
    teleop_cmd, monkeypatch, tmp_path
):
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    assert teleop_cmd._get_launch_command_by_stem("does-not-exist") is None


def test_get_launch_command_by_stem_returns_ros2_command(
    teleop_cmd, monkeypatch, tmp_path
):
    _mk_launch_files(tmp_path, ["foo.launch.py"])
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)

    assert (
        teleop_cmd._get_launch_command_by_stem("foo")
        == "ros2 launch bringup foo.launch.py"
    )


def test_discover_launch_map_parses_expected_combinations(
    teleop_cmd, monkeypatch, tmp_path
):
    _mk_launch_files(
        tmp_path,
        [
            "pinocchio-adam_sp-noitom.launch.py",
            "pinocchio-adam_pro-zerolab.launch.py",
            "mink-adam_u-webvr-sg.launch.py",
            "mink-adam_u-webvr.launch.py",
            "test_retarget_vr.launch.py",  # should be ignored by _parse_launch_stem
            "weird_name.launch.py",  # should be ignored by regex
        ],
    )
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    m = teleop_cmd._discover_launch_map()
    # test_* ignored
    assert ("adam_sp", "noitom", "pinocchio") in m
    assert ("adam_pro", "zerolab", "pinocchio") in m
    # prefer shorter stem when key clashes (webvr over webvr-sg)
    assert m[("adam_u", "webvr", "mink")].endswith("mink-adam_u-webvr.launch.py")


def test_get_valid_options_from_map_returns_discovered_values(
    teleop_cmd, monkeypatch, tmp_path
):
    _mk_launch_files(
        tmp_path,
        [
            "pinocchio-adam_sp-noitom.launch.py",
            "mink-adam_u-webvr.launch.py",
        ],
    )
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    adam, mocap, algo = teleop_cmd._get_valid_options_from_map()
    assert adam == ["adam_sp", "adam_u"]
    assert mocap == ["noitom", "webvr"]
    assert algo == ["mink", "pinocchio"]


def test_complete_mocap_filters_by_ctx_params(teleop_cmd, monkeypatch, tmp_path):
    _mk_launch_files(
        tmp_path,
        [
            "pinocchio-adam_sp-noitom.launch.py",
            "mink-adam_u-webvr.launch.py",
            "pinocchio-adam_u-zerolab.launch.py",
        ],
    )
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    ctx = types.SimpleNamespace(params={"adam_type": "adam_sp"})
    opts = teleop_cmd._complete_mocap(ctx, [], "")
    assert opts == ["noitom"]

    ctx = types.SimpleNamespace(params={"adam_type": "adam_u"})
    opts = teleop_cmd._complete_mocap(ctx, [], "")
    assert opts == ["webvr", "zerolab"]


def test_complete_algorithm_filters_by_ctx_params(teleop_cmd, monkeypatch, tmp_path):
    _mk_launch_files(
        tmp_path,
        [
            "pinocchio-adam_u-noitom.launch.py",
            "mink-adam_u-webvr.launch.py",
        ],
    )
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    ctx = types.SimpleNamespace(
        params={"adam_type": "adam_u", "mocap_driver": "noitom"}
    )
    opts = teleop_cmd._complete_algorithm(ctx, [], "")
    assert opts == ["pinocchio"]

    ctx = types.SimpleNamespace(params={"adam_type": "adam_u", "mocap_driver": "webvr"})
    opts = teleop_cmd._complete_algorithm(ctx, [], "")
    assert opts == ["mink"]


def test_get_launch_command_success(teleop_cmd, monkeypatch, tmp_path):
    _mk_launch_files(tmp_path, ["pinocchio-adam_sp-noitom.launch.py"])
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    cmd = teleop_cmd._get_launch_command("adam_sp", "noitom", "pinocchio")
    assert cmd == "ros2 launch bringup pinocchio-adam_sp-noitom.launch.py"


def test_get_launch_command_unknown_combo_exits(teleop_cmd, monkeypatch, tmp_path):
    _mk_launch_files(tmp_path, ["pinocchio-adam_sp-noitom.launch.py"])
    monkeypatch.setattr(teleop_cmd, "_get_bringup_launch_dir", lambda: tmp_path)
    monkeypatch.setattr(teleop_cmd, "_LAUNCH_MAP_CACHE", None, raising=False)

    with pytest.raises(typer.Exit):
        teleop_cmd._get_launch_command("adam_sp", "webvr", "mink")
