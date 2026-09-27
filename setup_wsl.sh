#!/usr/bin/env bash
# One-time setup inside WSL 2 (Ubuntu). Installs into ~/okra-venv, ~/cyclonedds,
# ~/unitree_sdk2_python. Nothing is installed on the robot.
# Usage: bash setup_wsl.sh            (CPU torch, ~1.6 GB venv)
#        OKRA_GPU=1 bash setup_wsl.sh (CUDA 12.6 torch, ~7 GB venv; driver must support CUDA >= 12.6)
# Safe to re-run: finished steps are skipped.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
export PIP_NO_CACHE_DIR=1
# "No space left on device" with hundreds of GB free = /tmp is a small RAM disk (tmpfs, 1.9 GB here)
# and pip unpacks wheels there. The CUDA torch wheels need several GB -> use a temp dir on the real disk.
export TMPDIR="$HOME/.pip-tmp"
mkdir -p "$TMPDIR"
trap 'rm -rf "$HOME/.pip-tmp"' EXIT

echo "== apt packages =="
# libglib2.0-0 is libglib2.0-0t64 on Ubuntu >= 24.04; check both names
APT_PKGS="build-essential cmake git python3-venv python3-dev iputils-ping iproute2 libgl1 libusb-1.0-0 openssh-client"
missing=""
for p in $APT_PKGS; do dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"; done
dpkg -s libglib2.0-0t64 >/dev/null 2>&1 || dpkg -s libglib2.0-0 >/dev/null 2>&1 \
  || missing="$missing $(apt-cache show libglib2.0-0t64 >/dev/null 2>&1 && echo libglib2.0-0t64 || echo libglib2.0-0)"
if [ -n "$missing" ]; then
  sudo apt-get update
  sudo apt-get install -y $missing
else
  echo "all present"
fi

echo "== python venv (~/okra-venv, Python <= 3.12) =="
# cyclonedds 0.10.2 (pinned by unitree_sdk2py) breaks on Python >= 3.13: lazy annotations
# (PEP 649) -> "TypeError: Member ... is not defined" on import. Ubuntu 26.04 ships 3.14,
# so fetch a standalone 3.12 with uv (no sudo, lives in ~/.local).
venv_too_new() { ~/okra-venv/bin/python -c 'import sys; sys.exit(0 if sys.version_info >= (3, 13) else 1)'; }
if [ -x ~/okra-venv/bin/python ] && venv_too_new; then
  echo "existing venv is Python >= 3.13 - recreating it"
  rm -rf ~/okra-venv
fi
if [ ! -x ~/okra-venv/bin/python ]; then
  PY=""
  for v in 3.12 3.11 3.10; do command -v "python$v" >/dev/null && { PY="python$v"; break; }; done
  if [ -z "$PY" ]; then
    export PATH="$HOME/.local/bin:$PATH"
    command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
    uv python install 3.12
    PY="$(uv python find 3.12)"
  fi
  "$PY" -m venv ~/okra-venv
fi
source ~/okra-venv/bin/activate
pip install -U pip wheel

if python -c "import torch" 2>/dev/null; then
  echo "torch already installed: $(python -c 'import torch; print(torch.__version__)')"
elif [ "${OKRA_GPU:-0}" = "1" ]; then
  echo "OKRA_GPU=1 - installing CUDA 12.6 torch (large)"
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126 --extra-index-url https://pypi.org/simple
else
  echo "Installing CPU torch (small; yolo11n is fine on CPU). OKRA_GPU=1 for CUDA."
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi
pip install -r "$HERE/requirements.txt"

echo "== CycloneDDS 0.10.x (needed by unitree_sdk2_python) =="
if [ ! -d ~/cyclonedds/install ]; then
  git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds ~/cyclonedds
  mkdir -p ~/cyclonedds/build && cd ~/cyclonedds/build
  cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
  cmake --build . --target install -j"$(nproc)"
fi
export CYCLONEDDS_HOME=~/cyclonedds/install

echo "== unitree_sdk2_python =="
[ -d ~/unitree_sdk2_python ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python ~/unitree_sdk2_python
pip install -e ~/unitree_sdk2_python

grep -q "okra-venv" ~/.bashrc || cat >> ~/.bashrc <<'RC'
# okra-g1
export CYCLONEDDS_HOME=~/cyclonedds/install
alias okra='source ~/okra-venv/bin/activate && cd "/mnt/c/Users/aqilq/Documents/okra-g1"'
RC

echo "== okra model =="
cd "$HERE" && python -c "from okra_vision import download_model; print('model at', download_model())"

echo
echo "Done. Open a new WSL terminal and type:  okra"
echo "Then:  python probe_robot.py"
