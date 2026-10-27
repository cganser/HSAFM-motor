#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launcher="$repo_root/run_motor_control.sh"
desktop_dir="$HOME/Desktop"
desktop_file="$desktop_dir/HSAFM-Motor.desktop"

if [[ ! -x "$launcher" ]]; then
    echo "Application launcher not found or not executable: $launcher" >&2
    exit 1
fi

mkdir -p "$desktop_dir"

escape_exec_arg() {
    local value="$1"
    value="${value//\\/\\\\}"
    value="${value//\"/\\\"}"
    value="${value//%/%%}"
    printf '"%s"' "$value"
}

exec_path="$(escape_exec_arg "$launcher")"

cat > "$desktop_file" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=HSAFM Motor Controller
Comment=Launch the HS-AFM motor controller
Exec=$exec_path
Path=$repo_root
Terminal=false
Categories=Science;
EOF

chmod +x "$desktop_file"
echo "Created desktop launcher: $desktop_file"
echo "If prompted, right-click it and choose Allow Launching or Trust and Launch."
