#!/usr/bin/env bash
# Deploy the read-only HMI to a vehicle and restart its systemd service.
#
#   ./deploy_and_start_gnss_hmi.sh user@vehicle [dest]
#
# Env: DEST (default /opt/gnss-hmi), SERVICE (default gnss-hmi),
#      HOSTNAME_OR_IP for the post-deploy self-check (default vehicle host).
set -euo pipefail
TARGET="${1:?usage: $0 user@vehicle [dest]}"
DEST="${2:-${DEST:-/opt/gnss-hmi}}"
SERVICE="${SERVICE:-gnss-hmi}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT="$HERE/deploy/gnss-hmi.service"

echo "==> rsync $HERE -> $TARGET:$DEST"
rsync -a --delete \
  --exclude '.git/' --exclude '__pycache__/' --exclude '.pytest_cache/' \
  --exclude '*.egg-info/' --exclude 'data/' --exclude '.venv/' \
  "$HERE/" "$TARGET:$DEST/"

echo "==> remote venv + install"
ssh "$TARGET" bash -s -- "$DEST" <<'REMOTE'
set -euo pipefail
DEST="$1"
python3 -m venv "$DEST/.venv" 2>/dev/null || true
"$DEST/.venv/bin/pip" install -q -e "$DEST"
REMOTE

echo "==> install systemd unit + restart"
ssh "$TARGET" "sudo install -m 644 '$DEST/deploy/gnss-hmi.service' '/etc/systemd/system/$SERVICE.service' && \
               sudo systemctl daemon-reload && sudo systemctl enable '$SERVICE' && \
               sudo systemctl restart '$SERVICE'"

echo "==> self-check /api/status"
if ssh "$TARGET" "sleep 3; curl -fsS http://127.0.0.1:8080/api/status >/dev/null"; then
  echo "gnss-hmi is up on $TARGET"
else
  echo "self-check FAILED; last vehicle logs:"
  ssh "$TARGET" "sudo journalctl -u '$SERVICE' -n 50 --no-pager" || true
  exit 1
fi
