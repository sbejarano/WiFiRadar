# Raspberry Pi Setup (Raspberry Pi OS)

- Update and install system deps:
  - `sudo apt update && sudo apt install -y gpsd gpsd-clients iw tcpdump`
- Enable gpsd:
  - `sudo systemctl enable gpsd`
  - `sudo systemctl start gpsd`
  - Check fix: `cgps -s` or `gpsmon`
- Put Wi‑Fi interface in monitor mode (example `wlan0`):
  - `sudo ip link set wlan0 down`
  - `sudo iw dev wlan0 set type monitor`
  - `sudo ip link set wlan0 up`
  - Confirm: `iw dev`
  - Your interface name may remain `wlan0`; pass that to `--iface`.
- Python env:
  - `python3 -m venv .venv && source .venv/bin/activate`
  - `pip install -r requirements.txt` (and `-r requirements-dev.txt` for tests)
- Run app (needs sudo to sniff):
  - `sudo -E env PATH="$PATH" python wifi_radar.py --iface wlan0 --show-radar`

Notes:
- Some adapters require `airmon-ng` (Aircrack-ng) for monitor mode.
- Ensure power management is off if you see drops: `iw dev wlan0 set power_save off`.
- For 5 GHz, ensure regulatory domain permits channels: `sudo iw reg set <CC>`.
