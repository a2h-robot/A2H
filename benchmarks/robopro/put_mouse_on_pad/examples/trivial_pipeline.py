"""Trivial pipeline for the sanity checks: look at the cup and lift one arm a little."""


def run(api):
    api.detect("cup")
    api.move_by(dz=0.05, arm="left")
