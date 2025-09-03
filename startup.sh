#!/bin/bash
# WiFi Radar — EKF edition
# - wlan0 untouched (internet stays up)
# - wlan1 monitor (otherbss/fcsfail/control), power save off
# - fast hop on 1/6/11 (0.7s dwell)
# - filters to RSSI >= -70 dBm and prunes stale APs from the UI
# - GUI enabled

set -euo pipefail

IFACE="wlan1"                         # sniffer (Atheros AR9271)
GPSD_ADDR="127.0.0.1:2947"
OUT_GEOJSON="ap_estimates.geojson"

# Fast hopper for responsiveness; expand later to 1..13 if you want
HOP_LIST=(1 6 11)
HOP_DWELL=0.7

# MQTT (optional: set host to enable)
MQTT_HOST=""                          # e.g. "127.0.0.1" or "broker.local"
MQTT_PORT=1883
MQTT_BASE="wifi_radar"
MQTT_USER=""
MQTT_PASS=""
MQTT_QOS=0
MQTT_RETAIN=0

SHOW_RADAR=1                          # 1 = show radar window

# Filter + prune
RSSI_DISPLAY_MIN="-70"                # only accept frames with RSSI >= this (dBm)
PRUNE_AGE_SEC="20"                    # hide APs from radar if not seen for N seconds

cd "$(dirname "$0")"

# ---- venv / deps ----
if [ ! -d .venv ]; then python3 -m venv .venv; fi
source .venv/bin/activate
pip -q install --upgrade pip setuptools wheel
cat > requirements.txt <<'EOF'
scapy==2.5.0
gpsd-py3==0.3.0
paho-mqtt==1.6.1
numpy==1.26.4
pyyaml==6.0.2
matplotlib==3.9.2
EOF
pip -q install -r requirements.txt

# ---- prepare wlan1 only (leave wlan0 alone) ----
sudo pkill -f "wpa_supplicant.*${IFACE}" || true
sudo rfkill unblock all || true

echo "[i] Setting ${IFACE} to monitor (otherbss/fcsfail/control), power save off…"
sudo ip link set "${IFACE}" down || true
sudo iw dev "${IFACE}" set monitor otherbss fcsfail control 2>/dev/null || sudo iw dev "${IFACE}" set type monitor
sudo ip link set "${IFACE}" up
sudo iw dev "${IFACE}" set power_save off 2>/dev/null || true

iw dev "${IFACE}" info || true

# ---- fast channel hopper ----
echo "[i] Hopping ${IFACE}: ${HOP_LIST[*]} (dwell ${HOP_DWELL}s)"
(
  while true; do
    for ch in "${HOP_LIST[@]}"; do
      sudo iw dev "${IFACE}" set channel "${ch}" 2>/dev/null || true
      sleep "${HOP_DWELL}"
    done
  done
) & HOP_PID=$!
trap 'echo "[i] Stopping hopper"; kill ${HOP_PID} 2>/dev/null || true' EXIT

# ---- build radar args ----
RADAR_ARGS=(--iface "${IFACE}" --gpsd "${GPSD_ADDR}" --out-geojson "${OUT_GEOJSON}" \
            --gps-max-age 10 --count-without-gps \
            --rssi-display-min "${RSSI_DISPLAY_MIN}" --prune-age "${PRUNE_AGE_SEC}" \
            --show-radar)
# EKF tuning (optional; defaults are sane). Example:
# RADAR_ARGS+=(--ekf-q 1.0 --range-ema-alpha 0.4 --rr-ema-alpha 0.5 --rr-eps 0.12)

if [ -n "${MQTT_HOST}" ]; then
  RADAR_ARGS+=(--mqtt-host "${MQTT_HOST}" --mqtt-port "${MQTT_PORT}" --mqtt-base "${MQTT_BASE}" --mqtt-qos "${MQTT_QOS}")
  [ -n "${MQTT_USER}" ] && RADAR_ARGS+=(--mqtt-user "${MQTT_USER}")
  [ -n "${MQTT_PASS}" ] && RADAR_ARGS+=(--mqtt-pass "${MQTT_PASS}")
  [ "${MQTT_RETAIN}" = "1" ] && RADAR_ARGS+=(--mqtt-retain)
fi

echo "[i] Starting WiFi Radar…"
exec sudo .venv/bin/python3 wifi_radar.py "${RADAR_ARGS[@]}"
