# wifi-radar-trilateration

Walk-based Wi‑Fi AP trilateration with live radar UI and MQTT telemetry.

## Features
- Passive 802.11 capture (beacons, probe-responses) in monitor mode
- GPS via gpsd; local tangent frame origin on first 2D fix
- Log-distance path-loss RSSI→distance model (calibratable)
- Trilateration via weighted Gauss–Newton refinement (2D)
- Periodic GeoJSON output for latest AP position estimates
- Optional live radar UI (matplotlib) centered at current position
- Optional MQTT publishing per AP on updates
- Basic 5 GHz channel support with optional per-band calibration overrides
- Channel control: `--lock-channel` or `--hop` with `--hop-interval`

## Requirements
- Raspberry Pi 4/5 (or Linux host)
- Python 3.10+
- Monitor-mode capable Wi‑Fi adapter (2.4 GHz to start)
- gpsd reachable (USB GPS or GNSS HAT)

System packages: `gpsd`, `iw`, `tcpdump` (optional for troubleshooting)

## Setup
```
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Quick Start
- Put interface into monitor mode (e.g., with `iw` or `airmon-ng`).
- Ensure gpsd has a 2D+ fix.
- Run:
```
sudo -E env PATH="$PATH" \
python wifi_radar.py \
  --iface wlan0mon \
  --gpsd 127.0.0.1:2947 \
  --min-rssi -88 \
  --min-samples 3 \
  --out-geojson ap_estimates.geojson \
  --logfile wifi_radar.log \
  --show-radar
```

MQTT (optional):
```
--mqtt-host 192.168.1.10 --mqtt-port 1883 --mqtt-base wifi_radar \
--mqtt-user user --mqtt-pass pass --mqtt-qos 0 --mqtt-retain
```

Calibration (optional):
- Edit `example_calibration.yaml` and pass `--calib example_calibration.yaml`.
- Or quickly tune with `--rssi-at-1m` and `--n`.

## Data Outputs
- GeoJSON: `ap_estimates.geojson` FeatureCollection with latest AP estimates.
- Log: `wifi_radar.log` with events and AP updates.
- MQTT: `wifi_radar/aps/<bssid_no_colons>` payload includes BSSID, SSID, RSSI, Lat/Lon, RMSE, samples, etc.

## Notes
- Requires root to sniff; no packet injection is used.
- Processing waits until a GPS 2D fix is available.
- APs stale for 30s are hidden from the UI but kept in logs/GeoJSON.

## Testing
- Unit tests: RSSI→distance, solver convergence, channel mapping.
- Field test: Walk a loop around a known AP; verify estimate error 10–20 m (post-calibration) in open areas.

Dev:
- `pip install -r requirements-dev.txt`
- `pytest -q`

## Extras
- Offline simulator: `python apps/offline_simulator.py --help`
- Pcap replay to CSV: `python apps/replay_pcap.py input.pcap --out samples.csv`
- Offline trilat from CSV: `python apps/offline_trilaterate_csv.py --help`
- Pi setup guide: see `SETUP_PI.md`

## License
- For internal experimentation; do not commit secrets or large captures.
