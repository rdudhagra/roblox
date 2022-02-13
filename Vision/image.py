import numpy as np

_ = 0
R = 1
O = 2
Y = 3
G = 4
B = 5
V = 6

SMILE_IMAGE = [
    [_, _, _, _, _, Y, _, _],
    [_, B, _, _, _, _, Y, _],
    [_, _, _, R, R, _, Y, _],
    [_, B, _, _, _, _, Y, _],
    [_, _, _, _, _, Y, _, _],
]

FLIP_SMILE_IMAGE = [
    [_, _, Y, _, _, _, _, _],
    [_, Y, _, _, _, _, B, _],
    [_, Y, _, R, R, _, _, _],
    [_, Y, _, _, _, _, B, _],
    [_, _, Y, _, _, _, _, _],
]

HI_IMAGE = [
    [_, V, _, _, B, _, G, _],
    [_, B, _, _, G, _, _, _],
    [_, G, G, Y, Y, _, O, _],
    [_, Y, _, _, O, _, R, _],
    [_, O, _, _, R, _, R, _],
]

GM_IMAGE = [
    [_, G, G, _, G, _, _, G],
    [G, _, _, _, G, G, G, G],
    [G, _, G, G, G, _, _, G],
    [G, _, _, G, G, _, _, G],
    [_, G, G, _, G, _, _, G],
]

BLANK_IMAGE = [
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
]

def cube_color_var_to_str(cube_color):
    if cube_color == _:
        return "blank"
    elif cube_color == R:
        return "red"
    elif cube_color == O:
        return "orange"
    elif cube_color == Y:
        return "yellow"
    elif cube_color == G:
        return "green"
    elif cube_color == B:
        return "blue"
    elif cube_color == V:
        return "purple"
    else:
        raise ValueError("Invalid cube color: {}".format(cube_color))

def cubes_needed(image: np.ndarray, image_progress: np.ndarray):
    for row_ind in range(image.shape[0]):
        image_row = image[row_ind]
        image_progress_row = image_progress[row_ind]

        cubes_done = np.logical_or(image_progress_row == 1, image_row == _)
        if np.all(cubes_done):
            continue

        # Else, we still have cubes left in this row
        cubes_left = np.logical_not(cubes_done)
        return [(image[row_ind, col], (row_ind, col), cube_ind_to_world_pose(row_ind, col)) for col in np.argwhere(cubes_left == 1).ravel()]

    return None


def cube_ind_to_world_pose(cube_row, cube_col):
    return [100 + cube_row * 50, 120 + cube_col * 50, 0]


if __name__ == "__main__":
    print(cubes_needed(
        np.array([[_, _, _, _, _, Y, _, _],
                  [_, B, _, _, _, _, Y, _],
                  [_, _, _, R, R, _, Y, _],
                  [_, B, _, _, _, _, Y, _],
                  [_, _, _, _, _, Y, _, _]]),
        np.array([[0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 1, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0]])))
