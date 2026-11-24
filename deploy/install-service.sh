#!/bin/bash
# Install unit files only. Never enable the service or change the current power state.
set -euo pipefail
cd "$(dirname "$0")"
install -d /usr/local/libexec /etc/bxi
install -m755 start-node /usr/local/libexec/bxi-arms-node
install -m644 arms-node.service /etc/systemd/system/arms-node.service
[[ -e /etc/bxi/arms-node.env ]] || install -m640 arms-node.env /etc/bxi/arms-node.env
[[ -e /etc/bxi/arms.json ]] || install -m640 ../arms_driver/config/arms.json /etc/bxi/arms.json
systemctl daemon-reload
printf '%s
' 'Installed only; review /etc/bxi/arms-node.env, master ownership, wiring and site approval before enabling.'
