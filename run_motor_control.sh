#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_python="${VENV_DIR:-"$repo_root/.venv"}/bin/python"

if [[ ! -x "$venv_python" ]]; then
    echo "Virtual environment not found. Run ./setup_venv.sh first." >&2
    exit 1
fi

cd "$repo_root"
exec "$venv_python" "$repo_root/motor_control.py" "$@"
