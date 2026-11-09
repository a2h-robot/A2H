"""Sanity (put_bowl_in_sink_ks): move the bowl, but to the wrong place. Grasps it, carries it to the counter
just in front of the sink (outside the basin, more than 10 cm from its start) and releases it there.
Must score 0: moving the bowl is necessary for success, not sufficient."""

import numpy as np


def run(api):
    sink = api.detect("sink")
    bowl = api.detect("bowl")
    if not api.grasp(bowl, "right", strategy="top"):
        return
    api.move_by(dz=0.08, arm="right")
    b = api.detect("bowl")
    target = np.array([sink.position[0], sink.position[1] - sink.bbox_extents[1] / 2 - 0.09])
    p, q = api.tcp_pose("right")
    api.move_to((p + [target[0] - b.position[0], target[1] - b.position[1], 0.0], q), "right")
    api.move_by(dz=-0.07, arm="right", allow_contact=["table"])
    api.open_gripper("right")
    api.move_by(dz=0.1, arm="right", allow_contact=["bowl"])
