#!/usr/bin/env bash
# Install x4tags on a Raspberry Pi (Raspberry Pi OS / Debian 12 or newer, 64-bit), from this checkout:
#   - a virtual environment in .venv with x4tags, NAPA (built from source: needs a C compiler) and pyntcore
#   - /etc/x4tags.toml (from config.example.toml, never overwritten)
#   - the x4tags systemd service, started now and at every boot
#
# Usage: deploy/install.sh               everything
#        deploy/install.sh --no-service  just the virtual environment (no sudo needed if build tools are present)
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"

missing=()
command -v cc >/dev/null || missing+=(build-essential)
python3 -c "import venv, ensurepip" 2>/dev/null || missing+=(python3-venv)
[ -e "/usr/include/python3.$(python3 -c 'import sys; print(sys.version_info.minor)')/Python.h" ] || missing+=(python3-dev)
if ((${#missing[@]})); then
    echo "installing ${missing[*]}"
    sudo apt-get update && sudo apt-get install -y "${missing[@]}"
fi

[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install --upgrade pip >/dev/null
.venv/bin/pip install -e ".[nt]"
.venv/bin/x4tags --help >/dev/null
echo "installed: $REPO/.venv/bin/x4tags"

if [[ "${1:-}" == "--no-service" ]]; then
    exit 0
fi
if [ ! -e /etc/x4tags.toml ]; then
    sudo install -m 644 config.example.toml /etc/x4tags.toml
    echo "wrote /etc/x4tags.toml: set [mount] and [output] for your robot"
fi
sed -e "s|@USER@|$USER|g" -e "s|@REPO@|$REPO|g" deploy/x4tags.service | sudo tee /etc/systemd/system/x4tags.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now x4tags
echo "service running: journalctl -u x4tags -f"
