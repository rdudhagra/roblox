import sys
import os
import argparse
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument("--width", type=int, default=4, help="Number of tags wide")
args = parser.parse_args()
width = args.width
height = args.width * 4 // 3

# Get tag dimensions
if not os.path.exists("tag36_11_00000.png"):
    os.system("wget https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/" \
              "master/tag36h11/tag36_11_00000.png")
tag0 = np.array(Image.open("tag36_11_00000.png"))
(tag_h, tag_w, tag_channels) = tag0.shape
(grid_h, grid_w) = (tag_h + 2, tag_w + 2)

# Place tags on final grid
tag_grid = np.zeros((height * grid_h, width * grid_w, tag_channels), dtype=np.uint8)
tag_idx = 0
for y in range(height):
    for x in range(width):
        tag_url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/" \
                  "master/tag36h11/tag36_11_%05d.png" % tag_idx
        tag_idx += 1
        tag_file = tag_url.split("/")[-1]
        if not os.path.exists(tag_file):
            os.system(f"wget {tag_url}")

        tag_img = np.array(Image.open(tag_file))
        tag_grid[y * grid_h + 1 : (y + 1) * grid_h - 1, x * grid_w + 1 : (x + 1) * grid_w - 1, :] = tag_img

# Upsample tag grid using nearest neighbor sampling
final_width = 2400
final_scale = final_width // tag_grid.shape[0]
final_width = tag_grid.shape[1] * final_scale
final_height = tag_grid.shape[0] * final_scale
Image.fromarray(tag_grid) \
     .resize((final_width, final_height), Image.NEAREST) \
     .save(f"tag_grid_%d_%d.png" % (width, height))
