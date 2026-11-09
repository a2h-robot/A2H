"""Reference LiftPegUpright pipeline written only against the primitive API.

Mirrors the official motion-planning solution (grasp near one end, lift, tilt, lower).
Used by the sanity checks to exercise the gate with a pipeline that has s0 > 0.
"""

import numpy as np

from api import pose_mul


def run(api):
    if not api.grasp("peg", strategy="top", offset=[0.10, 0.0, 0.0], strength=0.6):
        return
    lift = pose_mul(([0, 0, 0.30], [1, 0, 0, 0]), api.tcp_pose())
    api.move_to(lift)
    theta = np.pi / 10
    tilt = pose_mul(lift, ([0, 0, 0], [np.cos(theta), 0, np.sin(theta), 0]))
    api.move_to(tilt)
    lower = pose_mul(([0, 0, -0.10], [1, 0, 0, 0]), tilt)
    api.move_to(lower)
    api.open_gripper()
