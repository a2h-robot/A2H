"""Trivial pipeline for sanity check (a): looks at the scene and does nothing else."""


def run(api):
    for name in api.objects() + api.parts():
        api.detect(name)
    api.wait(0.5)
