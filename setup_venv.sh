#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="${VENV_DIR:-"$HOME/.venv"}"

if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required. Install Python 3 and python3-venv, then try again." >&2
    exit 1
fi

python3 -m venv --system-site-packages "$venv_dir"
"$venv_dir/bin/python" -m pip install --upgrade pip
"$venv_dir/bin/python" -m pip install -r "$repo_root/requirements.txt"

echo "Virtual environment ready at: $venv_dir"
echo "Next: run $repo_root/run_motor_control.sh"
echo "The Digilent DAQ HAT library and RPi.GPIO must also be installed for the system Python."
