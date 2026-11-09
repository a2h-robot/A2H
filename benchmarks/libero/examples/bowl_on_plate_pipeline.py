"""Reference pipeline for the held-out LIBERO-Goal task put_the_bowl_on_the_plate,
written only against the primitive API. Not one of the five training tasks: it gives
the gate sanity checks a frozen pipeline with s0 > 0 without pre-solving a training task.
"""

import numpy as np


def run(api):
    bowl = api.detect("akita_black_bowl_1")
    # Pinch the bowl wall on the far side from the robot (the bowl is too wide to grasp across,
    # and the wine bottle stands next to its near side):
    # fingers close along x, across the wall.
    rim = np.array([bowl.bbox_extents[0] / 2 - 0.012, 0.0, 0.0])
    if not api.grasp("akita_black_bowl_1", strategy="top", offset=rim, depth=0.02, yaw=0.0):
        return
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, 0.12], q))
    plate = api.detect("plate_1")
    target = plate.bbox_center + rim + [0, 0, 0.07]
    api.place((target, q))
    p, q = api.tcp_pose()
    api.move_to((p + [0, 0, 0.10], q))
