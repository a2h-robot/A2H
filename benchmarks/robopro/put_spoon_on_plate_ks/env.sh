#!/usr/bin/env bash
# Environment of the RoboPRO GAH backend. Usage: source benchmarks/robopro/env.sh
# Needs ROBOPRO_ROOT (a RoboPRO checkout with customized_robotwin/ and benchmark/, assets installed) and
# SIM_PY (python of the RoboPRO simulator env: py3.10, SAPIEN 3.0.0b1, curobo). Put machine-specific
# values (these two, extra LD_LIBRARY_PATH entries, GAH_SCRATCH, GAH_HOME) in env.local.sh next to this file.
GAH_BACKEND=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[ -f "$GAH_BACKEND/env.local.sh" ] && source "$GAH_BACKEND/env.local.sh"
: "${ROBOPRO_ROOT:?set ROBOPRO_ROOT (RoboPRO checkout) in env.local.sh}"
: "${SIM_PY:?set SIM_PY (RoboPRO simulator python) in env.local.sh}"
export ROBOPRO_ROOT SIM_PY
export ROBOTWIN_ROOT=$ROBOPRO_ROOT/customized_robotwin
export BENCH_ROOT=$ROBOPRO_ROOT/benchmark
export ROBOTWIN_BENCH_TASK=bench
export PATH="$(dirname "$SIM_PY"):$PATH"
export PYTHONPATH="$ROBOTWIN_ROOT:$BENCH_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONWARNINGS=ignore::UserWarning
export VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}
unset DISPLAY
# OIDN is a silent no-op on Blackwell GPUs (noisy ray-traced frames); OptiX works everywhere.
export DXVLA_DENOISER=${DXVLA_DENOISER:-optix}
# RoboPRO builds a renderer per reset with 50000-entry pools that leak; gah_env caps them at these sizes.
export RT_MAX_MATERIALS=${RT_MAX_MATERIALS:-4000} RT_MAX_TEXTURES=${RT_MAX_TEXTURES:-4000}
# Node-local scratch (never /dev/shm). A private CUDA MPS pipe dir avoids hanging on a foreign MPS daemon.
SCR=${GAH_SCRATCH:-/var/tmp/gah_robopro_$(id -un)}
mkdir -p "$SCR/nomps" "$SCR/warp" "$SCR/cuda" "$SCR/mpl" "$SCR/xdg"
export CUDA_MPS_PIPE_DIRECTORY=$SCR/nomps CUDA_MPS_LOG_DIRECTORY=$SCR/nomps WARP_CACHE_PATH=$SCR/warp
export CUDA_CACHE_PATH=$SCR/cuda MPLCONFIGDIR=$SCR/mpl XDG_CACHE_HOME=$SCR/xdg TMPDIR=$SCR
[ -n "${GAH_HOME:-}" ] && export HOME=$GAH_HOME
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
