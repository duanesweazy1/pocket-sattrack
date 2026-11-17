#!/usr/bin/env bash
set -euo pipefail
install -D -m 0644 sattrack.py "$STAGE$APP_INSTALL_DIR/sattrack.py"
# Lets the gpio group switch on the Cap's 5 V rail, which powers its GPS (applied at boot)
install -D -m 0644 packaging/pocket-sattrack.tmpfiles "$STAGE/usr/lib/tmpfiles.d/pocket-sattrack.conf"
tmp=$(mktemp)
cat >"$tmp" <<'LAUNCHER'
#!/bin/sh
exec python3 /usr/share/APPLaunch/apps/pocket-sattrack/sattrack.py "$@"
LAUNCHER
install -D -m 0755 "$tmp" "$STAGE$INSTALL_PREFIX/bin/pocket-sattrack"
rm -f "$tmp"
