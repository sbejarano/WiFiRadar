#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wi-Fi Radar (walking, 5 dBi omni, channel hop, robust GPS)
- Sniff 802.11 beacons/probe-responses (monitor mode)
- GPS via gpsd (lat, lon, alt) with robust "last-good-fix" grace window
- RSSI -> range (log-distance), incremental trilateration (WLS + Gauss-Newton)
- MQTT publish per-AP with: BSSID, SSID, RSSI, Channel, Lat, Lon, Alt, RMSE, Samples
- Live GeoJSON output of AP estimates
- Optional radar-style live plot (polar); disabled by default for headless

Run with sudo and the venv's interpreter:
  sudo /path/to/.venv/bin/python3 wifi_radar.py --iface wlan1 --gpsd 127.0.0.1:2947
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime, timezone
import subprocess

import numpy as np
import yaml

# Optional imports guarded for clear errors
try:
    import gpsd  # gpsd-py3
except Exception:
    gpsd = None
try:
    from scapy.all import sniff, Dot11, Dot11Elt, RadioTap
except Exception as e:
    print("Scapy import failed. Install it and run with sudo.", file=sys.stderr)
    raise
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

EARTH_R = 6371000.0  # meters

def now_iso():
    return datetime.now(timezone.utc).isoformat()

def latlon_to_xy(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R
    return x, y

def xy_to_latlon(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon

def rssi_to_distance(rssi_dbm, rssi_at_1m=-45.0, path_loss_n=2.5, d0=1.0):
    # Log-distance path-loss
    return float(d0 * (10 ** ((rssi_at_1m - rssi_dbm) / (10.0 * path_loss_n))))

def robust_weight(distance_m, rssi_dbm):
    d = max(1.0, distance_m)
    rssi_quality = max(0.1, (rssi_dbm + 95.0) / 30.0)
    sigma = 0.15 * d / rssi_quality
    return 1.0 / (sigma * sigma)

def wls_trilaterate_2d(points):
    if len(points) < 3:
        return None, None, False
    ref = points[-1]
    px_ref = np.array([ref["x"], ref["y"]])
    d_ref = ref["d"]
    pr2 = np.dot(px_ref, px_ref)
    A, b, Wd = [], [], []
    for i in range(len(points) - 1):
        p = points[i]
        pi = np.array([p["x"], p["y"]])
        di = p["d"]
        A.append(2.0 * (pi - px_ref))
        b.append((di*di - d_ref*d_ref) - (np.dot(pi, pi) - pr2))
        Wd.append(0.5 * (p["w"] + ref["w"]))
    A = np.vstack(A)
    b = np.array(b).reshape(-1, 1)
    W = np.diag(Wd)
    try:
        x_hat = np.linalg.pinv(A.T @ W @ A) @ (A.T @ W @ b)
        x0 = x_hat.flatten()
    except Exception:
        return None, None, False
    res = []
    for p in points:
        res.append((math.hypot(x0[0]-p["x"], x0[1]-p["y"]) - p["d"])**2)
    rmse = math.sqrt(sum(res)/len(res))
    return (x0[0], x0[1]), rmse, True

def gn_refine_2d(points, x_init, iters=4):
    x = np.array(x_init, dtype=float)
    for _ in range(iters):
        J, r, W = [], [], []
        for p in points:
            dx = x[0] - p["x"]; dy = x[1] - p["y"]
            rng = math.hypot(dx, dy) or 1e-3
            J.append([dx/rng, dy/rng])
            r.append(rng - p["d"])
            W.append(p["w"])
        J = np.array(J); r = np.array(r).reshape(-1,1); Wm = np.diag(W)
        try:
            H = J.T @ Wm @ J
            g = J.T @ Wm @ r
            step = -np.linalg.pinv(H) @ g
            x = (x.reshape(-1,1) + step).flatten()
        except Exception:
            break
        if np.linalg.norm(step) < 0.05:  # <5 cm
            break
    res = [(math.hypot(x[0]-p["x"], x[1]-p["y"]) - p["d"])**2 for p in points]
    rmse = math.sqrt(sum(res)/len(res)) if res else None
    return (x[0], x[1]), rmse

def freq_to_channel(freq_mhz):
    try:
        f = int(freq_mhz or 0)
    except Exception:
        return None
    if 2412 <= f <= 2484:
        if f == 2484: return 14
        return (f - 2407) // 5
    if 5160 <= f <= 5885:
        return (f - 5000) // 5
    if 5925 <= f <= 7125:
        return (f - 5950) // 5
    return None

# ---------- Robust GPS thread ----------
class GpsThread(threading.Thread):
    """
    Polls gpsd, keeps last-good fix with age, altitude, and mode (2D/3D).
    get_fix(max_age_s) returns None if stale; get_status() returns (mode_str, age, alt).
    """
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._fix = None          # dict: lat, lon, alt, mode, ts_iso, ts_mono
        self._stop = threading.Event()

    def run(self):
        if gpsd is None:
            print("[!] gpsd-py3 not available in this interpreter. Run with your venv's python.", file=sys.stderr)
            return
        try:
            gpsd.connect(host=self.host, port=self.port)
        except Exception as e:
            print(f"[!] gpsd connect failed: {e}", file=sys.stderr)
            return

        while not self._stop.is_set():
            try:
                p = gpsd.get_current()
                mode = int(getattr(p, "mode", 1) or 1)
                lat = getattr(p, "lat", None)
                lon = getattr(p, "lon", None)
                # altitude may be .alt or .altitude depending on version
                alt = None
                for k in ("alt", "altitude"):
                    v = getattr(p, k, None)
                    if v is not None:
                        try:
                            alt = float(v)
                        except Exception:
                            pass
                        break

                if mode >= 2 and lat is not None and lon is not None:
                    with self._lock:
                        self._fix = {
                            "lat": float(lat),
                            "lon": float(lon),
                            "alt": float(alt) if alt is not None else None,
                            "mode": mode,
                            "ts_iso": now_iso(),
                            "ts_mono": time.monotonic()
                        }
            except Exception:
                pass
            time.sleep(0.25)

    def stop(self):
        self._stop.set()

    def get_fix(self, max_age_s=2.0, require_3d=False):
        """
        Return the most recent fix if not older than max_age_s.
        If require_3d=True, only return if mode==3.
        """
        with self._lock:
            f = self._fix
        if not f:
            return None
        age = time.monotonic() - f["ts_mono"]
        if age > max_age_s:
            return None
        if require_3d and f.get("mode", 1) != 3:
            return None
        return dict(f)

    def get_status(self):
        with self._lock:
            f = self._fix
        if not f:
            return ("NO FIX", None, None)
        age = time.monotonic() - f["ts_mono"]
        mode = f.get("mode", 1)
        mode_str = "3D" if mode == 3 else ("2D" if mode == 2 else "NO FIX")
        return (mode_str, round(age, 1), f.get("alt"))

# ---------- MQTT ----------
class MqttClient:
    def __init__(self, host, port, username=None, password=None, base_topic="wifi_radar", qos=0, retain=False):
        self.client = None
        self.qos = int(qos); self.retain = bool(retain)
        if mqtt is None: return
        self.client = mqtt.Client()
        if username: self.client.username_pw_set(username, password or None)
        try:
            self.client.connect(host, int(port), keepalive=30)
            self.client.loop_start()
        except Exception as e:
            print(f"[!] MQTT connect failed: {e}", file=sys.stderr)
            self.client = None
        self.base = base_topic.rstrip("/")

    def publish_ap(self, bssid, payload_dict):
        if self.client is None: return
        topic = f"{self.base}/aps/{bssid.replace(':','').lower()}"
        try:
            self.client.publish(topic, json.dumps(payload_dict, ensure_ascii=False), qos=self.qos, retain=self.retain)
        except Exception as e:
            print(f"[!] MQTT publish error: {e}", file=sys.stderr)

    def close(self):
        if self.client:
            try:
                self.client.loop_stop(); self.client.disconnect()
            except Exception:
                pass

# ---------- Radar core ----------
class WifiRadar:
    def __init__(self, iface, out_geojson, calib, min_rssi=-88, min_samples=3, write_period=2.0,
                 mqtt_client: MqttClient=None, show_radar=False, radar_radius_m=120.0):
        self.iface = iface
        self.out_geojson = out_geojson
        self.calib = calib
        self.min_rssi = int(min_rssi)
        self.min_samples = int(min_samples)
        self.write_period = float(write_period)
        self.mqtt = mqtt_client
        self.show_radar = bool(show_radar)
        self.radar_radius_m = float(radar_radius_m)

        self.origin = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._last_write = 0.0

        self.aps = defaultdict(lambda: {
            "ssid": None,
            "last_channel": None,
            "samples": deque(maxlen=4000),  # lat, lon, x, y, rssi, dist, w, alt, ch, ts
            "est": None,                    # x,y,lat,lon,rmse,n
            "last_update": None,
        })

        # Radar UI placeholders
        self._radar_fig = None
        self._radar_ax = None
        self._gps_fix = None

    def set_origin_if_needed(self, lat, lon):
        if self.origin is None:
            self.origin = (lat, lon)
            print(f"[i] Origin fixed at lat={lat:.6f}, lon={lon:.6f}")

    def add_observation(self, bssid, ssid, lat, lon, alt, rssi_dbm, channel):
        if rssi_dbm is None or rssi_dbm < self.min_rssi:
            return
        self.set_origin_if_needed(lat, lon)
        lat0, lon0 = self.origin
        x, y = latlon_to_xy(lat, lon, lat0, lon0)
        dist = rssi_to_distance(rssi_dbm,
                                rssi_at_1m=self.calib["rssi_at_1m"],
                                path_loss_n=self.calib["path_loss_n"],
                                d0=self.calib["d0"])
        w = robust_weight(dist, rssi_dbm)
        sample = {
            "lat": lat, "lon": lon, "x": x, "y": y,
            "alt": alt, "rssi": float(rssi_dbm), "dist": float(dist), "w": float(w),
            "channel": channel, "ts": now_iso()
        }
        with self.lock:
            ap = self.aps[bssid]
            if ssid:
                ap["ssid"] = ssid
            if channel:
                ap["last_channel"] = channel
            ap["samples"].append(sample)

    def _estimate_ap(self, samples):
        if len(samples) < self.min_samples:
            return None
        pts = [{"x": s["x"], "y": s["y"], "d": s["dist"], "w": s["w"]} for s in samples]
        x0, rmse0, ok = wls_trilaterate_2d(pts)
        if not ok: return None
        x_ref, rmse = gn_refine_2d(pts, x0, iters=4)
        return (x_ref[0], x_ref[1], rmse if rmse is not None else rmse0)

    def _publish_mqtt(self, bssid, ap):
        if self.mqtt is None: return
        est = ap.get("est")
        if not est: return
        last = ap["samples"][-1] if ap["samples"] else {}
        payload = {
            "BSSID": bssid,
            "SSID": ap.get("ssid"),
            "RSSI": float(last.get("rssi")) if last else None,
            "Channel": ap.get("last_channel"),
            "Lat": float(est["lat"]),
            "Lon": float(est["lon"]),
            "Alt": float(last.get("alt")) if last.get("alt") is not None else None,
            "RMSE_m": float(est.get("rmse")),
            "Samples": len(ap["samples"]),
            "UpdatedAt": ap.get("last_update"),
        }
        self.mqtt.publish_ap(bssid, payload)

    def _try_update_estimate(self, bssid, ap):
        if len(ap["samples"]) < self.min_samples or self.origin is None:
            return None
        res = self._estimate_ap(list(ap["samples"]))
        if not res: return None
        x, y, rmse = res
        lat0, lon0 = self.origin
        lat, lon = xy_to_latlon(x, y, lat0, lon0)
        est = {"x": float(x), "y": float(y), "lat": float(lat), "lon": float(lon), "rmse": float(rmse)}
        ap["est"] = est
        ap["last_update"] = now_iso()
        self._publish_mqtt(bssid, ap)
        return est

    def _write_geojson(self):
        if self.origin is None: return
        features = []
        with self.lock:
            for bssid, ap in self.aps.items():
                est = ap.get("est")
                if not est:
                    est = self._try_update_estimate(bssid, ap)
                if not est: continue
                props = {
                    "bssid": bssid,
                    "ssid": ap.get("ssid"),
                    "last_update": ap.get("last_update"),
                    "n_samples": len(ap["samples"]),
                    "rmse_m": est.get("rmse"),
                    "last_channel": ap.get("last_channel"),
                }
                feat = {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [est["lon"], est["lat"]]},
                    "properties": props
                }
                features.append(feat)
        if not features: return
        fc = {"type": "FeatureCollection", "features": features}
        tmp = self.out_geojson + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(fc, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.out_geojson)

    def periodic(self, gps_thread: "GpsThread"):
        while not self._stop.is_set():
            with self.lock:
                for bssid, ap in self.aps.items():
                    if len(ap["samples"]) >= self.min_samples:
                        self._try_update_estimate(bssid, ap)
            t = time.time()
            if t - self._last_write >= self.write_period and self.origin is not None:
                self._write_geojson()
                self._last_write = t
            self._gps_fix = gps_thread.get_fix(max_age_s=2.0)
            time.sleep(0.5)

    def stop(self): self._stop.set()

    # --------- Optional Radar UI ---------
    def start_radar_ui(self):
        if not self.show_radar:
            return
        # Lazy import to avoid Tk issues when headless
        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            print(f"[!] Matplotlib not available for GUI: {e}", file=sys.stderr)
            return
        self._plt = plt
        self._radar_fig, self._radar_ax = plt.subplots(subplot_kw={'projection':'polar'})
        self._radar_ax.set_title("Wi-Fi Radar (estimates relative to you)")
        self._radar_ax.set_ylim(0, self.radar_radius_m)
        self._radar_ax.grid(True)
        t = threading.Thread(target=self._radar_loop, daemon=True)
        t.start()

    def _radar_loop(self):
        plt = getattr(self, "_plt", None)
        if plt is None: return
        while not self._stop.is_set():
            try:
                fix = self._gps_fix
                if not fix or self.origin is None:
                    time.sleep(0.3); continue
                lat0, lon0 = fix["lat"], fix["lon"]
                thetas, rs, labels = [], [], []
                with self.lock:
                    for bssid, ap in self.aps.items():
                        est = ap.get("est")
                        if not est: continue
                        x, y = latlon_to_xy(est["lat"], est["lon"], lat0, lon0)
                        r = math.hypot(x, y)
                        if r > self.radar_radius_m: continue
                        theta = (math.degrees(math.atan2(x, y)) % 360.0) * math.pi/180.0
                        thetas.append(theta); rs.append(r)
                        labels.append(ap.get("ssid") or bssid)
                self._radar_ax.clear()
                self._radar_ax.set_ylim(0, self.radar_radius_m)
                self._radar_ax.set_title("Wi-Fi Radar (estimates relative to you)")
                self._radar_ax.grid(True)
                self._radar_ax.scatter(thetas, rs, s=30)
                for th, rr, lab in sorted(zip(thetas, rs, labels), key=lambda t: t[1])[:5]:
                    self._radar_ax.text(th, rr, lab[:12], fontsize=8)
                plt.pause(0.2)
            except Exception:
                time.sleep(0.5)

# ---------- Scapy helpers ----------
def parse_ssid(pkt):
    ssid = None
    try:
        elt = pkt.getlayer(Dot11Elt)
        while isinstance(elt, Dot11Elt):
            if elt.ID == 0:
                ssid = elt.info.decode(errors="ignore")
                if ssid == "": ssid = None
                break
            elt = elt.payload.getlayer(Dot11Elt)
    except Exception:
        pass
    return ssid

def get_rssi_dbm(pkt):
    try:
        if pkt.haslayer(RadioTap):
            val = getattr(pkt[RadioTap], "dBm_AntSignal", None)
            if val is not None:
                return int(val)
    except Exception:
        pass
    try:
        if hasattr(pkt, "dBm_AntSignal"):
            return int(pkt.dBm_AntSignal)
    except Exception:
        pass
    return None

def get_channel_from_pkt(pkt):
    freq = None
    try:
        if pkt.haslayer(RadioTap):
            freq = getattr(pkt[RadioTap], "ChannelFrequency", None)
    except Exception:
        pass
    ch = None
    if freq: ch = freq_to_channel(freq)
    if ch is None:
        try:
            elt = pkt.getlayer(Dot11Elt)
            while isinstance(elt, Dot11Elt):
                if elt.ID == 3 and len(elt.info) >= 1:
                    ch = int(elt.info[0])
                    break
                elt = elt.payload.getlayer(Dot11Elt)
        except Exception:
            pass
    return ch

def scapy_sniff_thread(iface, radar: WifiRadar, gps: GpsThread, stop_evt: threading.Event,
                       gps_max_age_s=2.0, require_3d_fix=False):
    def handler(pkt):
        if stop_evt.is_set(): return True
        try:
            if not pkt.haslayer(Dot11): return
            d11 = pkt[Dot11]
            if d11.type != 0 or d11.subtype not in (5, 8):  # probe resp or beacon
                return
            bssid = d11.addr3
            if not bssid: return
            rssi = get_rssi_dbm(pkt)
            ssid = parse_ssid(pkt)
            ch = get_channel_from_pkt(pkt)
            fix = gps.get_fix(max_age_s=gps_max_age_s, require_3d=require_3d_fix)
            if not fix: return
            radar.add_observation(
                bssid=bssid, ssid=ssid,
                lat=fix["lat"], lon=fix["lon"], alt=fix.get("alt"),
                rssi_dbm=rssi, channel=ch
            )
        except Exception:
            pass
    try:
        sniff(iface=iface, store=False, prn=handler, stop_filter=lambda p: stop_evt.is_set())
    except Exception as e:
        print(f"[!] Sniff error on {iface}: {e}", file=sys.stderr)

# ---------- Calibration I/O ----------
def load_calib(path, fallback_rssi1m, fallback_n, fallback_d0):
    if not path: return {"rssi_at_1m": fallback_rssi1m, "path_loss_n": fallback_n, "d0": fallback_d0}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return {
        "rssi_at_1m": float(data.get("rssi_at_1m", fallback_rssi1m)),
        "path_loss_n": float(data.get("path_loss_n", fallback_n)),
        "d0": float(data.get("d0", fallback_d0)),
    }

def save_calib(path, rssi_at_1m, path_loss_n, d0):
    data = {"rssi_at_1m": float(rssi_at_1m), "path_loss_n": float(path_loss_n), "d0": float(d0), "saved_at": now_iso()}
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f)
    print(f"[i] Calibration saved to {path}")

# ---------- CLI / Main ----------
def main():
    ap = argparse.ArgumentParser(description="Wi-Fi Radar (walking, 5 dBi omni, MQTT, robust GPS)")
    ap.add_argument("--iface", required=True, help="Monitor-mode interface (e.g., wlan1)")
    ap.add_argument("--gpsd", default="127.0.0.1:2947", help="gpsd host:port")
    ap.add_argument("--out-geojson", default="ap_estimates.geojson", help="Output GeoJSON path")
    ap.add_argument("--min-rssi", type=int, default=-88, help="Ignore frames weaker than this (dBm)")
    ap.add_argument("--min-samples", type=int, default=3, help="Min samples per BSSID to solve")
    ap.add_argument("--write-period", type=float, default=2.0, help="GeoJSON write interval seconds")
    # Calibration
    ap.add_argument("--calib", help="YAML with rssi_at_1m, path_loss_n, d0")
    ap.add_argument("--rssi-at-1m", type=float, default=-45.0, help="Fallback RSSI at 1 m")
    ap.add_argument("--n", dest="path_loss_n", type=float, default=2.5, help="Fallback path-loss exponent")
    ap.add_argument("--d0", type=float, default=1.0, help="Reference distance (m)")
    ap.add_argument("--save-calib", help="Write a calibration YAML and exit")
    # GPS robustness
    ap.add_argument("--gps-max-age", type=float, default=2.0, help="Grace period to reuse last good GPS fix (s)")
    ap.add_argument("--require-3d-fix", action="store_true", help="Only accept GPS samples with 3D fix")
    # MQTT
    ap.add_argument("--mqtt-host", default=None, help="MQTT broker host (omit to disable MQTT)")
    ap.add_argument("--mqtt-port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument("--mqtt-user", default=None, help="MQTT username")
    ap.add_argument("--mqtt-pass", default=None, help="MQTT password")
    ap.add_argument("--mqtt-base", default="wifi_radar", help="MQTT base topic")
    ap.add_argument("--mqtt-qos", type=int, default=0, help="MQTT QoS (0/1/2)")
    ap.add_argument("--mqtt-retain", action="store_true", help="Retain MQTT messages")
    # Radar UI
    ap.add_argument("--show-radar", action="store_true", help="Show radar-style live plot (requires GUI/Tk)")
    ap.add_argument("--radar-radius", type=float, default=120.0, help="Radar range (m)")
    args = ap.parse_args()

    if args.save_calib:
        save_calib(args.save_calib, args.rssi_at_1m, args.path_loss_n, args.d0)
        return

    calib = load_calib(args.calib, args.rssi_at_1m, args.path_loss_n, args.d0)
    print(f"[i] Using calibration: {calib}")

    # GPS
    host, port = (args.gpsd.split(":") + ["2947"])[:2]
    gps = GpsThread(host, int(port)); gps.start()

    # MQTT
    mc = None
    if args.mqtt_host:
        mc = MqttClient(
            host=args.mqtt_host, port=args.mqtt_port,
            username=args.mqtt_user, password=args.mqtt_pass,
            base_topic=args.mqtt_base, qos=args.mqtt_qos, retain=args.mqtt_retain
        )

    radar = WifiRadar(
        iface=args.iface, out_geojson=args.out_geojson, calib=calib,
        min_rssi=args.min_rssi, min_samples=args.min_samples, write_period=args.write_period,
        mqtt_client=mc, show_radar=args.show_radar, radar_radius_m=args.radar_radius
    )
    radar.start_radar_ui()  # no-op if not enabled

    stop_evt = threading.Event()
    sniffer = threading.Thread(
        target=scapy_sniff_thread,
        args=(args.iface, radar, gps, stop_evt, args.gps_max_age, args.require_3d_fix),
        daemon=True
    )
    sniffer.start()

    periodic = threading.Thread(target=radar.periodic, args=(gps,), daemon=True)
    periodic.start()

    def handle_sig(sig, frame):
        print("\n[!] Stopping ...")
        stop_evt.set()
        radar.stop()
        gps.stop()
        if mc: mc.close()
        time.sleep(0.4)
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    # Heartbeat
    try:
        while True:
            time.sleep(3.5)
            mode_str, age_s, alt = gps.get_status()
            with radar.lock:
                n_aps = len(radar.aps)
                solved = sum(1 for a in radar.aps.values() if a.get("est"))
            gps_part = f"GPS:{mode_str}"
            if age_s is not None:
                gps_part += f" age={age_s}s"
            if alt is not None:
                gps_part += f" alt={alt:.1f}m"
            print(f"[{now_iso()}] APs seen: {n_aps} | estimated: {solved} | {gps_part} | geojson: {args.out_geojson}")
    except KeyboardInterrupt:
        handle_sig(None, None)

if __name__ == "__main__":
    main()
