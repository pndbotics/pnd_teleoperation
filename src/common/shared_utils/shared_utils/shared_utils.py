from enum import IntEnum


class JoyBtnIndices(IntEnum):
    L_thumb = 0
    L_X = 1
    L_Y = 2
    R_thumb = 3
    R_A = 4
    R_B = 5
    L_thumb_touch = 6
    L_X_touch = 7
    L_Y_touch = 8
    R_thumb_touch = 9
    R_A_touch = 10
    R_B_touch = 11


class JoyAxesIndices(IntEnum):
    L_x = 0
    L_y = 1
    L_trigger = 2
    L_grip = 3
    R_x = 4
    R_y = 5
    R_trigger = 6
    R_grip = 7


if __name__ == "__main__":
    for btn in JoyBtnIndices:
        print(f"Button {btn.name} has index {btn.value}")
    for axis in JoyAxesIndices:
        print(f"Axis {axis.name} has index {axis.value}")
