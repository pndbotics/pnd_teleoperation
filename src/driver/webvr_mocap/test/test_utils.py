import pytest
from webvr_mocap.utils import count_elements


def test_count_elements():
    data = {
        "axes": [0.0, 1.0, -1.0],
        "buttons": [
            [1, 0],
            [0, 1],
            [
                1,
            ],
            [0, 0],
        ],
    }
    assert count_elements(data) == 10
    assert count_elements(data["axes"]) == 3
    assert count_elements(data["buttons"]) == 7
