#!/usr/bin/env bash
# Install LIBERO (robosuite 1.4.1 / MuJoCo) into a conda env named "libero".
# usage: install_libero.sh <parent_dir_for_LIBERO_clone> <path/to/conda.sh>
set -eo pipefail
# Ignore ~/.local user-site packages (shared accounts may have hundreds of them).
export PYTHONNOUSERSITE=1
PARENT=$1; CONDA_SH=$2; ENV=libero
source "$CONDA_SH"
mkdir -p "$PARENT"; cd "$PARENT"
[ -d LIBERO/.git ] || git clone -q https://github.com/Lifelong-Robot-Learning/LIBERO.git
cd LIBERO
git checkout -q 8f1084e
# torch>=2.6 defaults torch.load(weights_only=True); LIBERO init states are pickled numpy arrays.
sed -i 's/init_states = torch.load(init_states_path)$/init_states = torch.load(init_states_path, weights_only=False)/' \
  libero/libero/benchmark/__init__.py

conda env list | awk '{print $1}' | grep -qx "$ENV" || conda create -y -q -n "$ENV" python=3.10
conda activate "$ENV"
python -m pip install -q --upgrade pip
# cu128 wheels: needed for RTX 50xx (sm_120), also fine on RTX 40xx.
python -m pip install -q torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
python -m pip install -q numpy==1.26.4 robosuite==1.4.1 mujoco==3.2.3 bddl==1.0.1 easydict cloudpickle \
  gym==0.25.2 "imageio[ffmpeg]" termcolor matplotlib tqdm pyyaml future huggingface_hub scipy
python -m pip install -q -e . --config-settings editable_mode=compat

# Env-scoped LIBERO config (does not touch ~/.libero, which may belong to another install).
CFG="$CONDA_PREFIX/etc/libero"; mkdir -p "$CFG" libero/datasets
R="$PWD/libero/libero"
cat > "$CFG/config.yaml" <<EOF
assets: $R/assets
benchmark_root: $R
bddl_files: $R/bddl_files
datasets: $PWD/libero/datasets
init_states: $R/init_files
EOF
conda env config vars set -n "$ENV" LIBERO_CONFIG_PATH="$CFG" MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONNOUSERSITE=1 >/dev/null
python -m pip check || true
python -c "import numpy, torch, robosuite, mujoco; print('numpy', numpy.__file__); print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), '| robosuite', robosuite.__version__, '| mujoco', mujoco.__version__)"
echo INSTALL_DONE
