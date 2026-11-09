"""Sanity (put_bowl_in_sink_ks): a do-nothing policy. Looks at the bowl, opens and closes the free gripper,
moves nothing. Must score 0: the start pose the backend records cannot make an untouched bowl succeed."""


def run(api):
    api.detect("bowl")
    api.close_gripper("left")
    api.open_gripper("left")
