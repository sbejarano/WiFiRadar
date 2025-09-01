# Usage

## CLI Arguments

- `--iface` (required): Monitor-mode interface (e.g., `wlan0mon`).
- `--gpsd` (default `127.0.0.1:2947`): gpsd host:port.
- `--min-rssi` (default `-88`): Ignore frames weaker than this (dBm).
- `--min-samples` (default `3`): Min samples per BSSID to solve trilateration.
- `--write-period` (default `2.0`): Seconds between GeoJSON/log updates.
- `--calib`: YAML with `rssi_at_1m`, `path_loss_n`, `d0`.
- `--rssi-at-1m` (default `-45.0`): Fallback RSSI at 1 m.
- `--n` / `--path-loss-n` (default `2.5`): Path-loss exponent.
- `--d0` (default `1.0`): Reference distance (m).
- `--rssi-at-1m-5g` / `--n-5g`: Optional 5 GHz overrides.
- `--save-calib`: Write calibration YAML and exit.
- `--mqtt-host`: Enable MQTT by providing host; omit to disable.
- `--mqtt-port` (default `1883`).
- `--mqtt-user` / `--mqtt-pass` (optional).
- `--mqtt-base` (default `wifi_radar`).
- `--mqtt-qos` (default `0`).
- `--mqtt-retain` (flag): Publish retained.
- `--show-radar` (flag): Show live radar UI.
- `--radar-radius` (default `120.0`): Radar range in meters.
- `--out-geojson` (default `ap_estimates.geojson`).
- `--logfile` (default `wifi_radar.log`).
- `--lock-channel` (int): Lock the interface to a single channel.
- `--hop` (list or preset): Comma-separated channels to hop, or `2g`/`5g`.
- `--hop-interval` (default `1.0`): Seconds per hop.

## MQTT Topics and Payload

- Base: `wifi_radar` (configurable via `--mqtt-base`).
- Per-AP topic: `wifi_radar/aps/<bssid_no_colons>`.
- Payload JSON:
  - `BSSID`, `SSID`, `RSSI`, `Channel`, `Lat`, `Lon`, `Alt`, `RMSE_m`, `Samples`, `UpdatedAt`.

## Radar UI Keys

- Polar scatter centered at current GPS position.
- Dots/stars for APs; dotted radial hairline indicates estimated range.
- Stale timeout: APs hidden after 30s without frames.

## Exit Codes & Errors

- Interface error: exits if interface is not in monitor mode / sniff fails.
- GPS missing: processing deferred until 2D fix; warns every ~5s.
- MQTT error: non-fatal; logs and continues without MQTT.

## Offline Tools

- `apps/replay_pcap.py`: Extract Wi‑Fi mgmt frames to CSV: `python apps/replay_pcap.py input.pcap --out samples.csv`
- `apps/offline_trilaterate_csv.py`: Merge Wi‑Fi CSV and GPS CSV to estimate AP location:
  - GPS CSV headers: `timestamp,lat,lon[,alt]` (ISO-8601 timestamps)
  - Example: `python apps/offline_trilaterate_csv.py --wifi-csv samples.csv --gps-csv gps.csv --bssid aa:bb:cc:dd:ee:ff --out-geojson est.geojson`
