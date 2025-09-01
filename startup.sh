#!/bin/bash
# WiFi Radar startup with 2.4 GHz channel hopper (AR9271 is 2.4 GHz only)
# - Creates/activates venv, installs deps
# - Puts wlan1 in monitor mode
# - Hops channels in background (default 1..11)
# - Launches wifi_radar.py using the venv's Python under sudo (so gpsd-py3 is available)

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# ======= CONFIG YOU CAN TWEAK =======
IFACE="wlan1"
GPSD_ADDR="127.0.0.1:2947"
OUT_GEOJSON="ap_estimates.geojson"

# Channel control
CHANNEL_MODE="hop"             # "hop" or "fixed"
FIXED_CHANNEL="6"              # used when CHANNEL_MODE="fixed"
HOP_LIST=(1 2 3 4 5 6 7 8 9 10 11)   # for CE regions you can use 1..13
HOP_DWELL=4                    # seconds per channel

# Radar UI (headless by default). Set SHOW_GUI=1 only if you have desktop/Tk.
SHOW_GUI=0
DISPLAY_TARGET=":0"
X_USER="${USER}"
X_AUTH="/home/${X_USER}/.Xauthority"

# MQTT (optional)
MQTT_HOST=""                   # e.g., "127.0.0.1" to enable
MQTT_PORT=1883
MQTT_BASE="wifi_radar"
# ====================================

# 0) Ensure requirements exist & venv ready
if [ ! -f "requirements.txt" ]; then
cat > requirements.txt <<'EOF'
scapy==2.5.0
gpsd-py3==0.3.0
paho-mqtt==1.6.1
numpy==1.26.4
pyyaml==6.0.2
matplotlib==3.9.2
EOF
  echo "[i] Created requirements.txt"
fi

if [ ! -d ".venv" ]; then
  echo "[i] Creating Python virtual environment..."
  python3 -m venv .venv
fi

source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# 1) Ensure no manager is controlling wlan1
sudo pkill wpa_supplicant || true

# 2) Put IFACE into monitor mode
echo "[i] Putting ${IFACE} into monitor mode…"
sudo ip link set "${IFACE}" down || true
sudo iw dev "${IFACE}" set type monitor || true
sudo ip link set "${IFACE}" up || true

# 3) Start channel control
HOP_PID=""
if [ "${CHANNEL_MODE}" = "fixed" ]; then
  echo "[i] Setting ${IFACE} to fixed channel ${FIXED_CHANNEL}…"
  sudo iw dev "${IFACE}" set channel "${FIXED_CHANNEL}"
else
  echo "[i] Starting channel hopper on ${IFACE}: ${HOP_LIST[*]} (dwell ${HOP_DWELL}s)"
  (
    sleep 0.$(( (RANDOM % 7) + 1 ))   # small stagger
    while true; do
      for ch in "${HOP_LIST[@]}"; do
        sudo iw dev "${IFACE}" set channel "${ch}" 2>/dev/null || true
        sleep "${HOP_DWELL}"
      done
    done
  ) & HOP_PID=$!
  trap 'if [ -n "$HOP_PID" ] && kill -0 "$HOP_PID" 2>/dev/null; then echo "[i] Stopping hopper"; kill "$HOP_PID"; fi' EXIT
fi

# 4) Build radar args
RADAR_ARGS=(--iface "${IFACE}" --gpsd "${GPSD_ADDR}" --out-geojson "${OUT_GEOJSON}")

if [ -n "${MQTT_HOST}" ]; then
  RADAR_ARGS+=(--mqtt-host "${MQTT_HOST}" --mqtt-port "${MQTT_PORT}" --mqtt-base "${MQTT_BASE}")
fi

# 5) GUI vs headless
if [ "${SHOW_GUI}" -eq 1 ]; then
  # Requires: sudo apt-get install -y python3-tk tk ; and run xhost +SI:localuser:root in the desktop session
  export DISPLAY="${DISPLAY_TARGET}"
  export XAUTHORITY="${X_AUTH}"
else
  export MPLBACKEND=Agg
fi

# 6) Launch radar with venv Python under sudo (so gpsd-py3 is found)
echo "[i] Starting WiFi Radar… (mode: ${CHANNEL_MODE})"
exec sudo .venv/bin/python3 wifi_radar.py "${RADAR_ARGS[@]}"
