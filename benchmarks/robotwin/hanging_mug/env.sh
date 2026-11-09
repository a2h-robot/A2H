#!/usr/bin/env bash
# Environment for the RoboTwin GAH tools. Usage (from the repo root, on a GPU node / inside a Slurm job):
#   source benchmarks/robotwin/env.sh && python benchmarks/robotwin/tools/eval.py --task hanging_mug
# RT2_ENV points at a RoboTwin 2.0 install with an env.sh (sapien 3.0.0b1, cuRobo, aloha-agilex assets).
source "${RT2_ENV:?set RT2_ENV to the RoboTwin 2.0 install env.sh}"
export PYTHONUNBUFFERED=1
# RoboTwin builds a new SAPIEN renderer on every reset and its descriptor pools are not released, so a worker
# leaks VRAM per episode until OptiX runs out of memory and the worker hangs (seen 2026-10-23 on 16 GB 5080s).
# Small pools keep the leak far below the card size; the task scenes use far fewer materials/textures.
export RT_MAX_MATERIALS=${RT_MAX_MATERIALS:-1000}
export RT_MAX_TEXTURES=${RT_MAX_TEXTURES:-1000}
