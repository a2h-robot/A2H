# Environment for the RoboPRO GAH backend. Source it before running tools/*.py:
#   export ROBOPRO_STACK=/path/to/stack; source benchmarks/robopro/env.sh
# ROBOPRO_STACK holds a RoboPRO checkout (RoboPRO/customized_robotwin + RoboPRO/benchmark with assets), the robotwin_b1
# venv (envs/robotwin_b1: py3.10, SAPIEN 3.0.0b1, curobo) and a libvulkan directory (tooling/vklib).
# For an install laid out differently, override the pieces: ROBOPRO_CODE (dir holding customized_robotwin/ and
# benchmark/), SIM_PY (the venv's python), VK_LIB_DIR, and GAH_SCRATCH (per-process scratch; use node-local disk on a
# cluster, never /dev/shm).
: "${ROBOPRO_STACK:?set ROBOPRO_STACK to the directory that holds RoboPRO/, envs/robotwin_b1/ and tooling/vklib/}"
_code=${ROBOPRO_CODE:-$ROBOPRO_STACK/RoboPRO}
export SIM_PY=${SIM_PY:-$ROBOPRO_STACK/envs/robotwin_b1/bin/python}
_vk=${VK_LIB_DIR:-$ROBOPRO_STACK/tooling/vklib}
_scratch=${GAH_SCRATCH:-$ROBOPRO_STACK/tmp}
export ROBOPRO_ROOT=$_code/customized_robotwin
export BENCH_ROOT=$_code/benchmark
export ROBOTWIN_BENCH_TASK=bench
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export LD_LIBRARY_PATH=$_vk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
unset DISPLAY
# scratch and caches (never /dev/shm or $HOME)
export TMPDIR=$_scratch/tmp TMP=$_scratch/tmp TEMP=$_scratch/tmp
export XDG_CACHE_HOME=$_scratch/cache WARP_CACHE_PATH=$_scratch/cache/warp
export CUDA_CACHE_PATH=$_scratch/cache/nv_compute TORCH_EXTENSIONS_DIR=$_scratch/cache/torch_extensions
# a private MPS pipe dir: another user's MPS daemon otherwise makes new CUDA clients hang
export CUDA_MPS_PIPE_DIRECTORY=$_scratch/mps
mkdir -p "$TMPDIR" "$CUDA_MPS_PIPE_DIRECTORY" "$WARP_CACHE_PATH" "$CUDA_CACHE_PATH" 2>/dev/null || true
# OIDN is a silent no-op on Blackwell; RoboPRO's bench scenes hard-code optix anyway
export DXVLA_DENOISER=optix
# RoboPRO builds a new renderer per reset with huge descriptor pools; gah_env caps them (VRAM leak)
export RT_MAX_MATERIALS=${RT_MAX_MATERIALS:-4000} RT_MAX_TEXTURES=${RT_MAX_TEXTURES:-4000}
export PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} TOKENIZERS_PARALLELISM=false
unset _code _vk _scratch
