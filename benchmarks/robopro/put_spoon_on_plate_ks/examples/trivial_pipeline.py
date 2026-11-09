"""Trivial pipeline for the sanity checks: look at the spoon and lift one arm a little."""


def run(api):
    api.detect("spoon")
    api.move_by(dz=0.05, arm="left")
