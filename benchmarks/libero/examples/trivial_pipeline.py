"""Does nothing useful; used by sanity check (a) to exercise eval.py end to end."""


def run(api):
    api.detect(api.objects()[0])
    api.open_gripper()
