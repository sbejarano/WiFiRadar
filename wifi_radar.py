#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import logging
import math
import os
import subprocess
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# Third-party deps (see requirements.txt)
try:
    from scapy.all import (
        sniff,
        Dot11,
        Dot11Beacon,
        Dot11ProbeResp,
        Dot11Elt,
        RadioTap,
    )
except Exception:  # pragma: no cover - allow import-free review
    sniff = None
    Dot11 = Dot11Beacon = Dot11ProbeResp = Dot11Elt = RadioTap = object  # type: ignore

try:
    from gpsdclient import GPSDClient
except Exception:  # pragma: no cover
    GPSDClient = None  # type: ignore

try:
    import paho.mqtt.client as mqtt
except Exception:  # pragma: no cover
    mqtt = None  # type: ignore

try:
    import matplotlib
    matplotlib.use("Agg")  # switch to non-interactive unless --show-radar
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None  # type: ignore

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None  # type: ignore


ISO = "%Y-%m-%dT%H:%M:%S.%fZ"


@dataclass
class Calibration:
    rssi_at_1m: float = -45.0
    path_loss_n: float = 2.5
    d0_m: float = 1.0
    # Optional per-band overrides (5 GHz). If None, fall back to defaults above.
    rssi_at_1m_5g: Optional[float] = None
    path_loss_n_5g: Optional[float] = None

    @staticmethod
    def from_args(args) -> "Calibration":
        calib = Calibration(
            rssi_at_1m=args.rssi_at_1m,
            path_loss_n=args.path_loss_n,
            d0_m=args.d0,
            rssi_at_1m_5g=getattr(args, "rssi_at_1m_5g", None),
            path_loss_n_5g=getattr(args, "path_loss_n_5g", None),
        )
        if args.calib:
            if yaml is None:
                raise RuntimeError("PyYAML not available to load calibration file")
            with open(args.calib, "r") as f:
                data = yaml.safe_load(f) or {}
                calib.rssi_at_1m = float(data.get("rssi_at_1m", calib.rssi_at_1m))
                calib.path_loss_n = float(data.get("path_loss_n", calib.path_loss_n))
                calib.d0_m = float(data.get("d0_m", calib.d0_m))
                if "rssi_at_1m_5g" in data:
                    calib.rssi_at_1m_5g = float(data.get("rssi_at_1m_5g"))
                if "path_loss_n_5g" in data:
                    calib.path_loss_n_5g = float(data.get("path_loss_n_5g"))
        return calib

    def to_dict(self):
        out = {
            "rssi_at_1m": self.rssi_at_1m,
            "path_loss_n": self.path_loss_n,
            "d0_m": self.d0_m,
        }
        if self.rssi_at_1m_5g is not None:
            out["rssi_at_1m_5g"] = self.rssi_at_1m_5g
        if self.path_loss_n_5g is not None:
            out["path_loss_n_5g"] = self.path_loss_n_5g
        return out


def save_calibration_yaml(path: str, calib: Calibration):
    if yaml is None:
        raise RuntimeError("PyYAML not available to save calibration file")
    with open(path, "w") as f:
        yaml.safe_dump(calib.to_dict(), f, sort_keys=False)


class RssiModel:
    def __init__(self, calib: Calibration):
        self.calib = calib

    def distance_from_rssi(self, rssi_dbm: float, channel: Optional[int] = None) -> float:
        # Log-distance path-loss: RSSI(d) = RSSI(d0) - 10*n*log10(d/d0)
        # => d = d0 * 10^((RSSI(d0) - RSSI(d)) / (10*n))
        use_5g = False
        if channel is not None and channel >= 36:
            use_5g = True
        if use_5g and self.calib.rssi_at_1m_5g is not None:
            rssi0 = float(self.calib.rssi_at_1m_5g)
        else:
            rssi0 = float(self.calib.rssi_at_1m)
        if use_5g and self.calib.path_loss_n_5g is not None:
            n = float(self.calib.path_loss_n_5g)
        else:
            n = float(self.calib.path_loss_n)
        n = max(0.1, n)
        d0 = max(0.01, float(self.calib.d0_m))
        exp = (rssi0 - float(rssi_dbm)) / (10.0 * n)
        return d0 * (10.0 ** exp)


def latlon_to_local_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    # Equirectangular approximation around small areas
    # meters per degree
    lat0_rad = math.radians(lat0)
    m_per_deg_lat = 111_132.92 - 559.82 * math.cos(2 * lat0_rad) + 1.175 * math.cos(4 * lat0_rad)
    m_per_deg_lon = 111_412.84 * math.cos(lat0_rad) - 93.5 * math.cos(3 * lat0_rad)
    dx = (lon - lon0) * m_per_deg_lon
    dy = (lat - lat0) * m_per_deg_lat
    return dx, dy


def local_m_to_latlon(lat0: float, lon0: float, x: float, y: float) -> Tuple[float, float]:
    lat0_rad = math.radians(lat0)
    m_per_deg_lat = 111_132.92 - 559.82 * math.cos(2 * lat0_rad) + 1.175 * math.cos(4 * lat0_rad)
    m_per_deg_lon = 111_412.84 * math.cos(lat0_rad) - 93.5 * math.cos(3 * lat0_rad)
    lon = lon0 + x / m_per_deg_lon
    lat = lat0 + y / m_per_deg_lat
    return lat, lon


class Trilaterator:
    def __init__(self):
        pass

    @staticmethod
    def _initial_guess(samples: List[dict]) -> Tuple[float, float]:
        # Weighted centroid of sample positions as a robust initial guess
        ws = np.array([max(1e-6, s.get("w", 1.0)) for s in samples], dtype=float)
        xs = np.array([s["x"] for s in samples], dtype=float)
        ys = np.array([s["y"] for s in samples], dtype=float)
        wsum = float(ws.sum())
        if wsum <= 0:
            return float(xs.mean()), float(ys.mean())
        return float((ws * xs).sum() / wsum), float((ws * ys).sum() / wsum)

    def solve(self, samples: List[dict], iters: int = 5) -> Tuple[float, float, float]:
        if len(samples) < 3:
            raise ValueError("Need at least 3 samples for trilateration")

        x, y = self._initial_guess(samples)
        for _ in range(max(1, iters)):
            A = np.zeros((2, 2), dtype=float)
            b = np.zeros((2,), dtype=float)
            for s in samples:
                xi, yi, di = float(s["x"]), float(s["y"]), max(1e-3, float(s["dist_m"]))
                w = float(max(1e-6, s.get("w", 1.0)))
                dx = x - xi
                dy = y - yi
                ri = math.hypot(dx, dy)
                if ri < 1e-6:
                    # If current estimate exactly at a sample point, jitter slightly
                    ri = 1e-6
                # Residual: predicted range - measured range
                resid = ri - di
                # Jacobian
                jx = dx / ri
                jy = dy / ri
                # Accumulate weighted normal equations
                A[0, 0] += w * jx * jx
                A[0, 1] += w * jx * jy
                A[1, 0] += w * jy * jx
                A[1, 1] += w * jy * jy
                b[0] += w * jx * resid
                b[1] += w * jy * resid
            # Solve A * delta = -b
            try:
                delta = -np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                break
            x += float(delta[0])
            y += float(delta[1])
            if np.linalg.norm(delta) < 1e-3:
                break
        # RMSE
        residuals = []
        for s in samples:
            ri = math.hypot(x - float(s["x"]), y - float(s["y"]))
            residuals.append((ri - float(s["dist_m"])) ** 2)
        rmse = math.sqrt(sum(residuals) / len(residuals)) if residuals else float("nan")
        return x, y, rmse


class GPSService(threading.Thread):
    def __init__(self, host: str, port: int, logger: logging.Logger):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.logger = logger
        self._stop = threading.Event()
        self.current_fix = {
            "lat": None,
            "lon": None,
            "alt": None,
            "mode": 0,
            "ts": None,
        }
        self.origin = None  # (lat0, lon0)

    def run(self):
        if GPSDClient is None:
            self.logger.error("gpsd-py3 not installed; GPS disabled")
            return
        client = GPSDClient(host=self.host, port=self.port)
        last_warn = 0.0
        for result in client.json_stream(filter=["TPV"]):
            if self._stop.is_set():
                break
            if not isinstance(result, dict):
                continue
            if result.get("class") != "TPV":
                continue
            lat = result.get("lat")
            lon = result.get("lon")
            alt = result.get("alt")
            mode = int(result.get("mode", 0) or 0)
            ts = dt.datetime.utcnow().strftime(ISO)
            self.current_fix.update({"lat": lat, "lon": lon, "alt": alt, "mode": mode, "ts": ts})
            if mode >= 2 and lat is not None and lon is not None and self.origin is None:
                self.origin = (float(lat), float(lon))
                self.logger.info("GPS origin set at lat=%.6f lon=%.6f", lat, lon)
            if mode < 2:
                now = time.time()
                if now - last_warn > 5.0:
                    self.logger.warning("Waiting for GPS 2D fix...")
                    last_warn = now

    def stop(self):
        self._stop.set()


def short_bssid(bssid: str) -> str:
    try:
        return bssid.replace(":", "").lower()
    except Exception:
        return bssid


class MQTTPublisher:
    def __init__(self, args, logger: logging.Logger):
        self.logger = logger
        self.enabled = bool(args.mqtt_host)
        self.base = args.mqtt_base
        self.qos = int(args.mqtt_qos)
        self.retain = bool(args.mqtt_retain)
        self.client = None
        if self.enabled:
            if mqtt is None:
                self.logger.error("paho-mqtt not installed; MQTT disabled")
                self.enabled = False
            else:
                self.client = mqtt.Client()
                if args.mqtt_user:
                    self.client.username_pw_set(args.mqtt_user, args.mqtt_pass or None)
                try:
                    self.client.connect(args.mqtt_host, int(args.mqtt_port))
                    self.client.loop_start()
                    self.logger.info("Connected MQTT %s:%s", args.mqtt_host, args.mqtt_port)
                except Exception as e:
                    self.logger.error("MQTT connect failed: %s; continuing without MQTT", e)
                    self.enabled = False

    def publish_ap(self, payload: dict):
        if not self.enabled or self.client is None:
            return
        bssid_key = short_bssid(payload.get("BSSID", "unknown"))
        topic = f"{self.base}/aps/{bssid_key}"
        try:
            self.client.publish(topic, json.dumps(payload), qos=self.qos, retain=self.retain)
        except Exception as e:
            self.logger.error("MQTT publish failed: %s", e)

    def close(self):
        if self.client is not None:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass


class GeoJSONWriter(threading.Thread):
    def __init__(self, path: str, interval: float, estimates_ref: Dict[str, dict], lock: threading.Lock, logger: logging.Logger):
        super().__init__(daemon=True)
        self.path = path
        self.interval = interval
        self.estimates_ref = estimates_ref
        self.lock = lock
        self.logger = logger
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            self.write_once()
            self._stop.wait(self.interval)

    def stop(self):
        self._stop.set()

    def write_once(self):
        with self.lock:
            feats = []
            for est in self.estimates_ref.values():
                if not ("Lat" in est and "Lon" in est):
                    continue
                props = {
                    "BSSID": est.get("BSSID"),
                    "SSID": est.get("SSID"),
                    "RMSE_m": est.get("RMSE_m"),
                    "n_samples": est.get("Samples"),
                    "last_update": est.get("UpdatedAt"),
                    "last_channel": est.get("LastChannel"),
                }
                feats.append(
                    {
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [float(est["Lon"]), float(est["Lat"])],
                        },
                        "properties": props,
                    }
                )
            fc = {"type": "FeatureCollection", "features": feats}
        try:
            with open(self.path, "w") as f:
                json.dump(fc, f)
        except Exception as e:
            self.logger.error("Failed writing GeoJSON: %s", e)


class RadarUI(threading.Thread):
    def __init__(self, args, gps: GPSService, estimates_ref: Dict[str, dict], last_seen_ref: Dict[str, float], lock: threading.Lock, logger: logging.Logger):
        super().__init__(daemon=True)
        self.args = args
        self.gps = gps
        self.estimates_ref = estimates_ref
        self.last_seen_ref = last_seen_ref
        self.lock = lock
        self.logger = logger
        self._stop = threading.Event()
        self.radius = float(args.radar_radius)

        if plt is None:
            raise RuntimeError("matplotlib not available for radar UI")

        matplotlib.use("TkAgg", force=True)  # try to enable interactive backend
        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_rlim(0, self.radius)
        self.ax.set_title("WiFi Radar")
        self.scat = None

    def run(self):
        plt.ion()
        self.fig.show()
        while not self._stop.is_set():
            self.refresh()
            plt.pause(1.0 / max(1.0, float(self.args.refresh_rate_hz)))

    def stop(self):
        self._stop.set()

    def refresh(self):
        if self.gps.origin is None:
            return
        lat0, lon0 = self.gps.origin
        thetas = []
        rs = []
        labels = []
        now = time.time()
        with self.lock:
            for bssid, est in list(self.estimates_ref.items()):
                last = self.last_seen_ref.get(bssid, 0)
                if now - last > float(self.args.ap_stale_timeout_s):
                    # hide stale in UI only
                    continue
                # position relative to origin
                lat = est.get("Lat")
                lon = est.get("Lon")
                if lat is None or lon is None:
                    continue
                x, y = latlon_to_local_m(lat0, lon0, float(lat), float(lon))
                r = math.hypot(x, y)
                theta = (math.atan2(x, y))  # bearing, 0 at north
                if r <= self.radius:
                    thetas.append(theta)
                    rs.append(r)
                    label = est.get("SSID") or bssid[:8]
                    labels.append(label)
        self.ax.clear()
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_rlim(0, self.radius)
        self.ax.grid(True, linestyle=":")
        self.ax.set_title("WiFi Radar")
        if rs:
            self.ax.scatter(thetas, rs, s=30, c="tab:green")
        # Optionally, draw hairlines with labels
        for theta, r, label in zip(thetas, rs, labels):
            self.ax.plot([theta, theta], [0, r], linestyle=":", color="gray", linewidth=0.8)
            self.ax.text(theta, min(self.radius, r + 3.0), str(label), fontsize=8)


def frequency_to_channel(freq_mhz: int) -> Optional[int]:
    # 2.4 GHz
    if 2412 <= freq_mhz <= 2472:
        return int((freq_mhz - 2412) / 5 + 1)
    if freq_mhz == 2484:
        return 14
    # 5 GHz common channels
    # Map exact center frequencies to channel numbers where possible
    freq_to_chan = {
        5035: 7,  # uncommon
        5040: 8,
        5045: 9,
        5055: 11,
        5060: 12,
        5080: 16,
        5180: 36,
        5200: 40,
        5220: 44,
        5240: 48,
        5260: 52,
        5280: 56,
        5300: 60,
        5320: 64,
        5500: 100,
        5520: 104,
        5540: 108,
        5560: 112,
        5580: 116,
        5600: 120,
        5620: 124,
        5640: 128,
        5660: 132,
        5680: 136,
        5700: 140,
        5720: 144,
        5745: 149,
        5765: 153,
        5785: 157,
        5805: 161,
        5825: 165,
    }
    return freq_to_chan.get(freq_mhz)


def extract_channel(pkt) -> Optional[int]:
    try:
        if hasattr(pkt, "ChannelFrequency") and int(pkt.ChannelFrequency) > 0:
            freq = int(pkt.ChannelFrequency)
            ch = frequency_to_channel(freq)
            if ch is not None:
                return ch
    except Exception:
        pass
    # Fallback: DS Parameter Set (IE 3)
    try:
        elts = pkt.getlayer(Dot11Elt)
        while isinstance(elts, Dot11Elt):
            if int(elts.ID) == 3:
                return int(elts.info[0])
            elts = elts.payload.getlayer(Dot11Elt)
    except Exception:
        pass
    return None


def extract_ssid(pkt) -> Optional[str]:
    try:
        ssid = pkt[Dot11Elt].info.decode(errors="ignore")
        if ssid:
            return ssid
    except Exception:
        pass
    return None


class Estimator:
    def __init__(self, args, gps: GPSService, rssi_model: RssiModel, mqtt_pub: MQTTPublisher, logger: logging.Logger):
        self.args = args
        self.gps = gps
        self.rssi_model = rssi_model
        self.mqtt_pub = mqtt_pub
        self.logger = logger

        self.lock = threading.Lock()
        self.samples: Dict[str, List[dict]] = {}
        self.estimates: Dict[str, dict] = {}
        self.last_seen: Dict[str, float] = {}
        self.solver = Trilaterator()

    def add_sample(self, bssid: str, ssid: Optional[str], rssi: int, channel: Optional[int]):
        # Only process when we have a 2D fix and origin
        fix = self.gps.current_fix
        if self.gps.origin is None or (fix.get("mode", 0) or 0) < 2:
            return
        lat = fix.get("lat")
        lon = fix.get("lon")
        alt = fix.get("alt")
        if lat is None or lon is None:
            return
        lat0, lon0 = self.gps.origin
        x, y = latlon_to_local_m(lat0, lon0, float(lat), float(lon))
        dist = self.rssi_model.distance_from_rssi(rssi, channel=channel)
        # Weight: downweight distant/weak samples
        # Combine distance-based and RSSI-based weighting
        w_dist = 1.0 / ((dist + 1.0) ** 2)
        w_rssi = max(0.05, min(1.0, (rssi - self.args.min_rssi) / 20.0))
        w = float(w_dist * w_rssi)

        sample = {
            "BSSID": bssid,
            "SSID": ssid,
            "RSSI": int(rssi),
            "Channel": int(channel) if channel is not None else None,
            "Lat": float(lat),
            "Lon": float(lon),
            "Alt": float(alt) if alt is not None else None,
            "x": float(x),
            "y": float(y),
            "dist_m": float(dist),
            "w": float(w),
            "timestamp": dt.datetime.utcnow().strftime(ISO),
        }
        with self.lock:
            arr = self.samples.setdefault(bssid, [])
            arr.append(sample)
            self.last_seen[bssid] = time.time()
            # Update estimate if enough samples
            if len(arr) >= int(self.args.min_samples):
                self._update_estimate(bssid, ssid, arr)

    def _update_estimate(self, bssid: str, ssid: Optional[str], arr: List[dict]):
        try:
            x, y, rmse = self.solver.solve(arr, iters=5)
        except Exception as e:
            self.logger.debug("Solve failed for %s: %s", bssid, e)
            return
        lat0, lon0 = self.gps.origin
        lat, lon = local_m_to_latlon(lat0, lon0, x, y)
        channel = None
        # last known channel from samples
        for s in reversed(arr):
            if s.get("Channel") is not None:
                channel = int(s["Channel"])  # type: ignore
                break
        est = {
            "BSSID": bssid,
            "SSID": ssid,
            "Lat": float(lat),
            "Lon": float(lon),
            "RMSE_m": float(rmse),
            "Samples": int(len(arr)),
            "LastChannel": channel,
            "UpdatedAt": dt.datetime.utcnow().strftime(ISO),
        }
        self.estimates[bssid] = est
        # Publish MQTT on each update
        payload = {
            "BSSID": bssid,
            "SSID": ssid,
            "RSSI": int(arr[-1]["RSSI"]),
            "Channel": channel,
            "Lat": est["Lat"],
            "Lon": est["Lon"],
            "Alt": arr[-1].get("Alt"),
            "RMSE_m": est["RMSE_m"],
            "Samples": est["Samples"],
            "UpdatedAt": est["UpdatedAt"],
        }
        self.mqtt_pub.publish_ap(payload)
        self.logger.info(
            "AP %s updated: samples=%d rmse=%.1f m lat=%.6f lon=%.6f",
            bssid,
            est["Samples"],
            est["RMSE_m"],
            est["Lat"],
            est["Lon"],
        )


class CaptureService(threading.Thread):
    def __init__(self, args, estimator: Estimator, logger: logging.Logger):
        super().__init__(daemon=True)
        self.args = args
        self.estimator = estimator
        self.logger = logger
        self._stop = threading.Event()

    def run(self):
        if sniff is None:
            self.logger.error("Scapy not available; capture disabled")
            return
        try:
            sniff(
                iface=self.args.iface,
                prn=self._handle_pkt,
                store=False,
                stop_filter=lambda _: self._stop.is_set(),
                lfilter=self._mgmt_beacon_or_probe,
            )
        except Exception as e:
            self.logger.error("Sniff failed: %s", e)

    def stop(self):
        self._stop.set()

    @staticmethod
    def _mgmt_beacon_or_probe(pkt) -> bool:
        try:
            if not pkt.haslayer(Dot11):
                return False
            dot11 = pkt.getlayer(Dot11)
            # type=0 mgmt; subtype=8 beacon, 5 probe response
            return int(dot11.type) == 0 and int(dot11.subtype) in (5, 8)
        except Exception:
            return False

    def _handle_pkt(self, pkt):
        try:
            if not pkt.haslayer(Dot11):
                return
            dot11 = pkt.getlayer(Dot11)
            # BSSID depends on mgmt subtype; for beacon/probe-resp, addr3 is BSSID
            bssid = str(dot11.addr3 or "").lower()
            if not bssid:
                return
            # RSSI from radiotap
            rssi = None
            try:
                rssi = int(pkt.dBm_AntSignal)  # type: ignore[attr-defined]
            except Exception:
                pass
            if rssi is None or rssi < int(self.args.min_rssi):
                return
            ssid = extract_ssid(pkt)
            channel = extract_channel(pkt)
            self.estimator.add_sample(bssid=bssid, ssid=ssid, rssi=rssi, channel=channel)
        except Exception as e:
            self.logger.debug("Pkt parse error: %s", e)


class ChannelController(threading.Thread):
    def __init__(self, iface: str, logger: logging.Logger, lock_channel: Optional[int] = None, hop_channels: Optional[List[int]] = None, hop_interval: float = 1.0):
        super().__init__(daemon=True)
        self.iface = iface
        self.logger = logger
        self.lock_channel = lock_channel
        self.hop_channels = hop_channels or []
        self.hop_interval = max(0.2, float(hop_interval))
        self._stop = threading.Event()

    def run(self):
        try:
            if self.lock_channel is not None:
                self._set_channel(self.lock_channel)
                return
            if not self.hop_channels:
                return
            idx = 0
            while not self._stop.is_set():
                ch = self.hop_channels[idx % len(self.hop_channels)]
                self._set_channel(ch)
                idx += 1
                self._stop.wait(self.hop_interval)
        except Exception as e:
            self.logger.error("Channel controller error: %s", e)

    def stop(self):
        self._stop.set()

    def _set_channel(self, ch: int):
        try:
            subprocess.run(["iw", "dev", self.iface, "set", "channel", str(int(ch))], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.logger.info("Interface %s set to channel %s", self.iface, ch)
        except Exception as e:
            self.logger.error("Failed to set channel %s on %s: %s", ch, self.iface, e)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Walk-based Wi-Fi AP trilateration with radar and MQTT")
    p.add_argument("--iface", required=True, help="Monitor-mode interface (e.g., wlan0mon)")
    p.add_argument("--gpsd", default="127.0.0.1:2947", help="gpsd host:port")
    p.add_argument("--min-rssi", type=int, default=-88, help="Ignore frames weaker than this (dBm)")
    p.add_argument("--min-samples", type=int, default=3, help="Min samples per BSSID to solve trilateration")
    p.add_argument("--write-period", type=float, default=2.0, help="Seconds between GeoJSON/log updates")
    p.add_argument("--calib", help="YAML with rssi_at_1m, path_loss_n, d0")
    p.add_argument("--rssi-at-1m", dest="rssi_at_1m", type=float, default=-45.0, help="Fallback RSSI at 1 m")
    p.add_argument("--n", dest="path_loss_n", type=float, default=2.5, help="Path-loss exponent")
    p.add_argument("--path-loss-n", dest="path_loss_n", type=float, help=argparse.SUPPRESS)
    p.add_argument("--d0", type=float, default=1.0, help="Reference distance (m)")
    # Optional 5 GHz overrides
    p.add_argument("--rssi-at-1m-5g", dest="rssi_at_1m_5g", type=float, help="RSSI@1m for 5 GHz (overrides if set)")
    p.add_argument("--n-5g", dest="path_loss_n_5g", type=float, help="Path-loss exponent for 5 GHz (overrides if set)")
    p.add_argument("--save-calib", help="Write calibration YAML and exit")
    # MQTT
    p.add_argument("--mqtt-host", help="Enable MQTT by providing host; omit to disable")
    p.add_argument("--mqtt-port", type=int, default=1883, help="MQTT port")
    p.add_argument("--mqtt-user", help="MQTT username (optional)")
    p.add_argument("--mqtt-pass", help="MQTT password (optional)")
    p.add_argument("--mqtt-base", default="wifi_radar", help="MQTT base topic")
    p.add_argument("--mqtt-qos", type=int, default=0, help="MQTT QoS (0/1/2)")
    p.add_argument("--mqtt-retain", action="store_true", help="Publish retained")
    # UI
    p.add_argument("--show-radar", action="store_true", help="Show live radar UI")
    p.add_argument("--radar-radius", type=float, default=120.0, help="Radar range in meters")
    p.add_argument("--refresh-rate-hz", type=float, default=3.0, help="UI refresh rate in Hz")
    # Persistence and logging
    p.add_argument("--out-geojson", default="ap_estimates.geojson", help="GeoJSON output path")
    p.add_argument("--logfile", default="wifi_radar.log", help="Log file path")
    # Timeouts
    p.add_argument("--ap-stale-timeout-s", type=float, default=30.0, help="Stale timeout for UI hiding (s)")
    # Channel control
    p.add_argument("--lock-channel", type=int, help="Lock interface to a single channel")
    p.add_argument(
        "--hop",
        help="Comma-separated channel list or preset '2g' or '5g' to hop",
    )
    p.add_argument("--hop-interval", type=float, default=1.0, help="Seconds per hop when --hop is used")
    return p


def setup_logging(logfile: str):
    os.makedirs(os.path.dirname(logfile) or ".", exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.FileHandler(logfile), logging.StreamHandler(sys.stdout)],
    )
    return logging.getLogger("wifi_radar")


def parse_hostport(hp: str) -> Tuple[str, int]:
    if ":" in hp:
        host, port = hp.rsplit(":", 1)
        return host, int(port)
    return hp, 2947


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    logger = setup_logging(args.logfile)
    logger.info("Starting wifi-radar-trilateration")

    if args.save_calib:
        calib = Calibration.from_args(args)
        save_calibration_yaml(args.save_calib, calib)
        logger.info("Saved calibration to %s", args.save_calib)
        return 0

    calib = Calibration.from_args(args)
    rssi_model = RssiModel(calib)

    gps_host, gps_port = parse_hostport(args.gpsd)
    gps = GPSService(gps_host, gps_port, logger)
    gps.start()

    mqtt_pub = MQTTPublisher(args, logger)
    estimator = Estimator(args, gps, rssi_model, mqtt_pub, logger)

    geojson_writer = GeoJSONWriter(args.out_geojson, args.write_period, estimator.estimates, estimator.lock, logger)
    geojson_writer.start()

    # Channel control
    hop_channels: Optional[List[int]] = None
    if args.hop:
        if args.hop.lower() == "2g":
            hop_channels = [1, 6, 11]
        elif args.hop.lower() == "5g":
            hop_channels = [36, 40, 44, 48]
        else:
            try:
                hop_channels = [int(x.strip()) for x in args.hop.split(",") if x.strip()]
            except Exception:
                logger.error("Invalid --hop list; expected comma-separated ints or '2g'/'5g'")
                hop_channels = None

    chan_ctl = ChannelController(
        iface=args.iface,
        logger=logger,
        lock_channel=args.lock_channel,
        hop_channels=hop_channels,
        hop_interval=args.hop_interval,
    )
    chan_ctl.start()

    cap = CaptureService(args, estimator, logger)
    cap.start()

    radar = None
    if args.show_radar:
        try:
            radar = RadarUI(args, gps, estimator.estimates, estimator.last_seen, estimator.lock, logger)
            radar.start()
        except Exception as e:
            logger.error("Radar UI init failed: %s", e)

    # Graceful shutdown
    stop_event = threading.Event()

    def _sig_handler(signum, frame):  # noqa: ARG001
        stop_event.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    finally:
        logger.info("Shutting down...")
        cap.stop()
        gps.stop()
        geojson_writer.stop()
        if radar is not None:
            radar.stop()
        chan_ctl.stop()
        mqtt_pub.close()
        # Allow threads to exit
        time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
