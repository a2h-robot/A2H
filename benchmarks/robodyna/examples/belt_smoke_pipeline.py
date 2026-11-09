"""Smoke test (not a training pipeline): a crude place_block_belt attempt through the API only.
It shows that a program other than RoboDyna's expert can reach the task's own success check, and
records the gripper opening when closed on a real object (SANITY.md)."""


def run(api):
    block = api.detect("block")
    belt = api.detect("belt")
    line = api.detect("place line")
    blocker = api.detect("blocker")
    arm = "left" if block.position[0] < 0 else "right"
    if not api.grasp("block", arm):
        return
    api.move_by(arm, dz=0.15)
    top = belt.bbox_center[2] + belt.bbox_extents[2] / 2
    lane_y = belt.bbox_center[1] - 0.09 if blocker is None or blocker.position[1] > belt.bbox_center[1] \
        else belt.bbox_center[1] + 0.09
    x = line.position[0] - 0.06
    target = [x, lane_y, top + block.bbox_extents[2] / 2 + 0.005, *block.quaternion]
    api.place(target, arm, obj="block", pre_dis=0.06)
    api.move_by(arm, dz=0.12)
    api.wait(4.0)
