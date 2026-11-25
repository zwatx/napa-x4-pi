#!/usr/bin/env bash
# Install x4tags on a Raspberry Pi 5 (64-bit Raspberry Pi OS / Debian 12 or newer), from this checkout:
#   - a virtual environment in .venv with x4tags, NAPA and pyntcore (prebuilt wheels: no compiler needed)
#   - /etc/x4tags.toml (from config.example.toml, never overwritten)
#   - the x4tags systemd service, (re)started now and at every boot
#
# Usage: deploy/install.sh               everything; run it again after `git pull` to update
#        deploy/install.sh --no-service  just the virtual environment (no sudo needed if python3-venv is there)
#        deploy/install.sh --uninstall   stop and remove the service (keeps /etc/x4tags.toml and .venv)
set -euo pipefail
if [ "$(id -u)" -eq 0 ]; then
    echo "run this as the user the service should run as (it uses sudo itself where needed), not as root" >&2
    exit 1
fi
cd "$(dirname "$0")/.."
REPO="$(pwd)"
RUN_AS="$(id -un)"

if [[ "${1:-}" == "--uninstall" ]]; then
    sudo systemctl disable --now x4tags 2>/dev/null || true
    sudo rm -f /etc/systemd/system/x4tags.service
    sudo systemctl daemon-reload
    echo "removed the x4tags service (kept /etc/x4tags.toml and $REPO/.venv)"
    exit 0
fi

case "$(uname -m)" in
    aarch64|x86_64) ;;
    *) echo "needs a 64-bit OS (this is $(uname -m)): pyntcore and NAPA have no wheels for it" >&2; exit 1 ;;
esac

if ! python3 -c "import venv, ensurepip" 2>/dev/null; then
    echo "installing python3-venv"
    sudo apt-get update && sudo apt-get install -y python3-venv
fi

# A .venv from another Python or machine (copied over, or left from an older OS) won't work: start it again.
want="$(python3 -c 'import platform, sys; print(sys.version_info[:2], platform.machine())')"
have="$(.venv/bin/python -c 'import platform, sys; print(sys.version_info[:2], platform.machine())' 2>/dev/null || true)"
if [ -d .venv ] && [ "$have" != "$want" ]; then
    echo "rebuilding .venv (it was ${have:-broken}, this system is $want)"
    rm -rf .venv
fi
[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install --upgrade pip >/dev/null 2>&1 || echo "couldn't upgrade pip (offline?): carrying on"
.venv/bin/pip install -e ".[nt]"
.venv/bin/x4tags --version
.venv/bin/python -c "import ntcore" || { echo "pyntcore didn't install: NetworkTables output won't work" >&2; exit 1; }
echo "installed: $REPO/.venv/bin/x4tags"

if [[ "${1:-}" == "--no-service" ]]; then
    exit 0
fi
if [ ! -e /etc/x4tags.toml ]; then
    sudo install -m 644 config.example.toml /etc/x4tags.toml
    echo "wrote /etc/x4tags.toml: set [mount], [tags], [field] and [output] for your robot"
fi
sed -e "s|@USER@|$RUN_AS|g" -e "s|@REPO@|$REPO|g" deploy/x4tags.service \
    | sudo tee /etc/systemd/system/x4tags.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable x4tags >/dev/null
sudo systemctl restart x4tags  # so an update takes effect now, not at the next boot
echo "service running: journalctl -u x4tags -f"
