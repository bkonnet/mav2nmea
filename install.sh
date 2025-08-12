#!/usr/bin/env bash
set -euo pipefail
APP_DIR="/opt/mav2nmea"
USER_NAME="${SUDO_USER:-$USER}"

echo ">> Instalando mav2nmea en ${APP_DIR} (user=${USER_NAME})"

sudo mkdir -p "$APP_DIR"
sudo cp -a mav2nmea.py requirements.txt "$APP_DIR/"
sudo chown -R "${USER_NAME}:${USER_NAME}" "$APP_DIR"

cd "$APP_DIR"
if [[ ! -d venv ]]; then
  sudo -u "${USER_NAME}" python3 -m venv venv
fi
sudo -u "${USER_NAME}" bash -lc "source venv/bin/activate && pip install --upgrade pip && pip install -r requirements.txt"

UNIT=/etc/systemd/system/mav2nmea.service
sudo tee "$UNIT" >/dev/null <<'EOF'
[Unit]
Description=MAVLink → NMEA bridge (UDP->TCP/UDP) via venv
Wants=network-online.target
After=network-online.target

[Service]
User=__USER__
WorkingDirectory=/opt/mav2nmea
Environment=PYTHONUNBUFFERED=1
ExecStart=/opt/mav2nmea/venv/bin/python3 /opt/mav2nmea/mav2nmea.py
Restart=always
RestartSec=3
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
EOF
sudo sed -i "s|__USER__|${USER_NAME}|g" "$UNIT"

sudo systemctl daemon-reload
sudo systemctl enable --now mav2nmea

echo ">> Listo. Revisa: sudo systemctl status mav2nmea"
