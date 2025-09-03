#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, json, math, os, signal, sys, threading, time, hashlib
from collections import defaultdict, deque
from datetime import datetime, timezone

import numpy as np, yaml

# optional deps
try:
    import gpsd
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

EARTH_R = 6371000.0

def now_iso(): return datetime.now(timezone.utc).isoformat()
def now_mono(): return time.monotonic()

def latlon_to_xy(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R
    return x, y

def xy_to_latlon(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon

def rssi_to_distance(rssi_dbm, rssi_at_1m=-45.0, path_loss_n=2.5, d0=1.0):
    if rssi_dbm is None:
        return None
    return float(d0 * (10 ** ((rssi_at_1m - rssi_dbm) / (10.0 * path_loss_n))))

def robust_weight(distance_m, rssi_dbm):
    if distance_m is None or rssi_dbm is None:
        return 0.0
    d = max(1.0, distance_m)
    rssi_quality = max(0.1, (rssi_dbm + 95.0) / 30.0)  # ~[-95..-65] → [0..1]
    sigma = 0.15 * d / rssi_quality
    return 1.0 / (sigma * sigma)

def freq_to_channel(freq_mhz):
    try: f = int(freq_mhz or 0)
    except Exception: return None
    if 2412 <= f <= 2484:
        return 14 if f==2484 else (f-2407)//5
    if 5160 <= f <= 5885: return (f-5000)//5
    if 5925 <= f <= 7125: return (f-5950)//5
    return None

# =============== EKF for static AP position [x, y] ==================
class EKF2D:
    def __init__(self, q_pos=1.0):
        self.x = None            # state [x, y] (meters, local tangent plane)
        self.P = None            # covariance 2x2
        self.Q = np.diag([q_pos**2, q_pos**2])  # small process noise

    def is_init(self): return self.x is not None

    def init_from_guess(self, x0, y0, pos_std=15.0):
        self.x = np.array([[float(x0)], [float(y0)]])
        self.P = np.diag([pos_std**2, pos_std**2])

    def predict(self):
        if self.x is None: return
        self.P = self.P + self.Q

    def _joseph_update(self, H, z_res, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ z_res
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

    def update_range(self, xu, yu, r_meas, sigma_r):
        if self.x is None: return
        dx = self.x[0,0] - xu; dy = self.x[1,0] - yu
        r = math.hypot(dx, dy)
        if r < 1e-3: r = 1e-3
        H = np.array([[dx/r, dy/r]])  # 1x2
        z_res = np.array([[float(r_meas) - r]])
        R = np.array([[float(sigma_r)**2]])
        self._joseph_update(H, z_res, R)

    def update_rdot(self, xu, yu, vx, vy, rdot_meas, sigma_rr):
        if self.x is None: return
        dx = self.x[0,0] - xu; dy = self.x[1,0] - yu
        r = math.hypot(dx, dy)
        if r < 1e-3: r = 1e-3
        q = dx*vx + dy*vy
        rdot = - q / r
        r2 = r*r; r3 = r2*r
        dfdx = - (vx*r2 - q*dx) / r3
        dfdy = - (vy*r2 - q*dy) / r3
        H = np.array([[dfdx, dfdy]])   # 1x2
        z_res = np.array([[float(rdot_meas) - rdot]])
        R = np.array([[float(sigma_rr)**2]])
        self._joseph_update(H, z_res, R)

# ====================================================================

# -------- GPS --------
class GpsThread(threading.Thread):
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host; self.port = port
        self._lock = threading.Lock(); self._fix = None; self._stop = threading.Event()
    def run(self):
        if gpsd is None:
            print("[!] gpsd-py3 missing; use venv python.", file=sys.stderr); return
        try: gpsd.connect(host=self.host, port=self.port)
        except Exception as e:
            print(f"[!] gpsd connect failed: {e}", file=sys.stderr); return
        while not self._stop.is_set():
            try:
                p = gpsd.get_current()
                mode = int(getattr(p, "mode", 1) or 1)
                lat = getattr(p, "lat", None); lon = getattr(p, "lon", None)
                spd = getattr(p, "speed", None)   # m/s
                trk = getattr(p, "track", None)   # degrees from North
                alt = None
                for k in ("alt", "altitude"):
                    v = getattr(p, k, None)
                    if v is not None:
                        try: alt = float(v)
                        except Exception: pass
                        break
                if mode >= 2 and lat is not None and lon is not None:
                    with self._lock:
                        self._fix = {
                            "lat": float(lat), "lon": float(lon),
                            "alt": float(alt) if alt is not None else None,
                            "mode": mode,
                            "speed": float(spd) if spd is not None else None,
                            "track": float(trk) if trk is not None else None,
                            "ts_iso": now_iso(), "ts_mono": now_mono()
                        }
            except Exception:
                pass
            time.sleep(0.2)
    def stop(self): self._stop.set()
    def get_fix(self, max_age_s=10.0, require_3d=False):
        with self._lock: f = self._fix
        if not f: return None
        if now_mono() - f["ts_mono"] > max_age_s: return None
        if require_3d and f.get("mode",1) != 3: return None
        return dict(f)
    def get_status(self):
        with self._lock: f = self._fix
        if not f: return ("NO FIX", None, None)
        age = now_mono() - f["ts_mono"]; mode = f.get("mode",1)
        return ("3D" if mode==3 else ("2D" if mode==2 else "NO FIX"), round(age,1), f.get("alt"))

# -------- MQTT --------
class MqttClient:
    def __init__(self, host, port, username=None, password=None, base_topic="wifi_radar", qos=0, retain=False):
        self.client=None; self.qos=int(qos); self.retain=bool(retain); self.base=base_topic.rstrip("/")
        if mqtt is None: return
        self.client=mqtt.Client()
        if username: self.client.username_pw_set(username, password or None)
        try: self.client.connect(host, int(port), keepalive=30); self.client.loop_start()
        except Exception as e: print(f"[!] MQTT connect failed: {e}", file=sys.stderr); self.client=None
    def publish_ap(self, bssid, payload):
        if not self.client: return
        topic=f"{self.base}/aps/{bssid.replace(':','').lower()}"
        try: self.client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=self.qos, retain=self.retain)
        except Exception as e: print(f"[!] MQTT publish error: {e}", file=sys.stderr)
    def close(self):
        if self.client:
            try: self.client.loop_stop(); self.client.disconnect()
            except Exception: pass

# -------- Radar --------
class WifiRadar:
    def __init__(self, iface, out_geojson, calib, min_rssi=-90, min_samples=3, write_period=2.0,
                 mqtt_client: MqttClient=None, show_radar=False, radar_radius_m=120.0,
                 count_without_gps=False, rssi_display_min=-70, prune_age_s=20.0,
                 ekf_q=1.0, range_ema_alpha=0.4, rr_ema_alpha=0.5, rr_eps=0.12):
        self.iface=iface; self.out_geojson=out_geojson; self.calib=calib
        self.min_rssi=int(min_rssi); self.min_samples=int(min_samples); self.write_period=float(write_period)
        self.mqtt=mqtt_client; self.show_radar=bool(show_radar); self.radar_radius_m=float(radar_radius_m)
        self.count_without_gps=bool(count_without_gps)
        self.rssi_display_min=int(rssi_display_min)   # only accept frames with RSSI >= this
        self.prune_age_s=float(prune_age_s)           # hide APs not seen recently

        # EKF/filters
        self.ekf_q=float(ekf_q)
        self.range_ema_alpha=float(range_ema_alpha)
        self.rr_ema_alpha=float(rr_ema_alpha)
        self.rr_eps=float(rr_eps)

        self.origin=None
        self.lock=threading.Lock(); self._stop=threading.Event(); self._last_write=0.0

        # Seen + samples + EKF filter per AP
        self.seen = defaultdict(lambda: {
            "ssid": None, "last_rssi": None, "last_channel": None,
            "first_seen": None, "last_seen": None, "last_seen_mono": None, "frames": 0
        })
        self.aps = defaultdict(lambda: {
            "ssid": None, "last_channel": None,
            "samples": deque(maxlen=4000),  # lat, lon, x, y, rssi, dist, w, alt, ch, ts
            "est": None, "last_update": None,
            "ekf": EKF2D(q_pos=self.ekf_q),
            "r_ema": None, "r_last": None, "r_last_t": None, "rr_ema": None
        })

        # GUI state
        self._radar_fig=None; self._radar_ax=None; self._gps_fix=None; self._plt=None

    def set_origin_if_needed(self, lat, lon):
        if self.origin is None:
            self.origin=(lat,lon); print(f"[i] Origin fixed at lat={lat:.6f}, lon={lon:.6f}")

    def register_seen(self, bssid, ssid, rssi, channel):
        # RSSI visibility filter
        if rssi is None or int(rssi) < self.rssi_display_min:
            return
        with self.lock:
            s = self.seen[bssid]
            if ssid: s["ssid"]=ssid
            if channel: s["last_channel"]=channel
            s["last_rssi"]=int(rssi)
            now_iso_str = now_iso(); now_m = now_mono()
            if s["first_seen"] is None: s["first_seen"]=now_iso_str
            s["last_seen"]=now_iso_str; s["last_seen_mono"]=now_m
            s["frames"] += 1

    def _sigma_r(self, r, rssi_dbm):
        base = max(2.0, 0.08*max(0.0, float(r)))
        if rssi_dbm is None: return base
        boost = max(0.6, min(1.4, 1.2 - (rssi_dbm + 95.0)/60.0))
        return base*boost

    def _sigma_rr(self, speed_mps):
        return max(0.6, 0.35 + 0.25*max(0.0, float(speed_mps or 0.0)))

    def add_observation(self, bssid, ssid, fix, rssi_dbm, channel):
        """Add a positioning sample (needs GPS). Incorporates EKF with range + range-rate."""
        lat=fix["lat"]; lon=fix["lon"]; alt=fix.get("alt")
        if rssi_dbm is None or int(rssi_dbm) < self.rssi_display_min:
            return
        if rssi_dbm is not None and rssi_dbm < self.min_rssi:
            return

        self.set_origin_if_needed(lat, lon)
        lat0, lon0 = self.origin
        xu, yu = latlon_to_xy(lat, lon, lat0, lon0)

        # range from RSSI
        r_meas = rssi_to_distance(rssi_dbm, self.calib["rssi_at_1m"], self.calib["path_loss_n"], self.calib["d0"])
        if r_meas is None:
            return

        # store sample
        dist = r_meas
        w = robust_weight(dist, rssi_dbm)
        sample={"lat":lat,"lon":lon,"x":xu,"y":yu,"alt":alt,
                "rssi":float(rssi_dbm),"dist":float(dist),
                "w":float(w),"channel":channel,"ts":now_iso()}
        with self.lock:
            ap=self.aps[bssid]
            if ssid: ap["ssid"]=ssid
            if channel: ap["last_channel"]=channel
            ap["samples"].append(sample)

            # --- range EMA + range-rate EMA ---
            t = now_mono()
            if ap["r_ema"] is None:
                ap["r_ema"] = r_meas
            else:
                a = self.range_ema_alpha
                ap["r_ema"] = a*r_meas + (1.0-a)*ap["r_ema"]
            rr_meas = None
            if ap["r_last"] is not None and ap["r_last_t"] is not None:
                dt = max(1e-3, t - ap["r_last_t"])
                rr_inst = (r_meas - ap["r_last"]) / dt
                if ap["rr_ema"] is None:
                    ap["rr_ema"] = rr_inst
                else:
                    b = self.rr_ema_alpha
                    ap["rr_ema"] = b*rr_inst + (1.0-b)*ap["rr_ema"]
                rr_meas = ap["rr_ema"]
            ap["r_last"] = r_meas; ap["r_last_t"] = t

            # --- EKF update ---
            ekf = ap["ekf"]
            speed = fix.get("speed") or 0.0
            track = fix.get("track")
            heading_rad = math.radians(track) if track is not None else None
            if not ekf.is_init():
                # choose initial azimuth using heading & range-rate if moving
                if heading_rad is None or speed < 0.2:
                    h = hashlib.sha1(bssid.encode("utf-8")).digest()
                    theta0 = ((int.from_bytes(h[:4], "big") % 3600) / 3600.0) * 2*math.pi
                else:
                    if rr_meas is None or abs(rr_meas) < self.rr_eps:
                        sign = 1.0 if (int(bssid.replace(":",""),16) & 1) else -1.0
                        theta0 = heading_rad + sign*math.pi/2.0
                    elif rr_meas < 0:
                        theta0 = heading_rad
                    else:
                        theta0 = heading_rad + math.pi
                x0 = xu + r_meas*math.sin(theta0)
                y0 = yu + r_meas*math.cos(theta0)
                pos_std = max(12.0, 0.3*r_meas)
                ekf.init_from_guess(x0, y0, pos_std=pos_std)

            ekf.predict()
            sigma_r = self._sigma_r(r_meas, rssi_dbm)
            ekf.update_range(xu, yu, r_meas, sigma_r)

            if rr_meas is not None and speed >= 0.2 and heading_rad is not None:
                vx = speed*math.sin(heading_rad)  # x: east
                vy = speed*math.cos(heading_rad)  # y: north
                sigma_rr = self._sigma_rr(speed)
                try:
                    ekf.update_rdot(xu, yu, vx, vy, rr_meas, sigma_rr)
                except Exception:
                    pass

            x_est, y_est = float(ekf.x[0,0]), float(ekf.x[1,0])
            lat_est, lon_est = xy_to_latlon(x_est, y_est, lat0, lon0)
            ap["est"] = {"x":x_est,"y":y_est,"lat":lat_est,"lon":lon_est,"rmse":None}
            ap["last_update"]=now_iso()
            self._publish_mqtt(bssid, ap)

    def _publish_mqtt(self, bssid, ap):
        if self.mqtt is None: return
        est=ap.get("est")
        if not est: return
        last=ap["samples"][-1] if ap["samples"] else {}
        payload={"BSSID":bssid,"SSID":ap.get("ssid"),"RSSI":last.get("rssi"),
                 "Channel":ap.get("last_channel"),"Lat":float(est["lat"]),"Lon":float(est["lon"]),
                 "Alt":(float(last.get("alt")) if last.get("alt") is not None else None),
                 "RMSE_m":float(est.get("rmse")) if est.get("rmse") is not None else None,
                 "Samples":len(ap["samples"]), "UpdatedAt":ap.get("last_update")}
        self.mqtt.publish_ap(bssid, payload)

    def _write_geojson(self):
        if self.origin is None: return
        feats=[]
        with self.lock:
            for bssid, ap in self.aps.items():
                est = ap.get("est")
                if not est: continue
                props={"bssid":bssid,"ssid":ap.get("ssid"),"last_update":ap.get("last_update"),
                       "n_samples":len(ap["samples"]),"rmse_m":est.get("rmse"),
                       "last_channel":ap.get("last_channel")}
                feats.append({"type":"Feature","geometry":{"type":"Point","coordinates":[est["lon"],est["lat"]]},
                              "properties":props})
        if not feats: return
        fc={"type":"FeatureCollection","features":feats}; tmp=self.out_geojson+".tmp"
        with open(tmp,"w",encoding="utf-8") as f: json.dump(fc,f,ensure_ascii=False,indent=2)
        os.replace(tmp,self.out_geojson)

    # ---------- azimuth for provisional markers ----------
    def _stable_theta(self, bssid: str) -> float:
        h = hashlib.sha1(bssid.encode("utf-8")).digest()
        val = int.from_bytes(h[:4], "big")
        return (val % 3600) / 3600.0 * 2.0 * math.pi  # 0..2π

    def _provisional_theta(self, bssid:str, speed, heading_rad, rr_ema):
        if heading_rad is None or speed < 0.2 or rr_ema is None:
            return self._stable_theta(bssid)
        if abs(rr_ema) < self.rr_eps:
            sign = 1.0 if (int(bssid.replace(":",""),16) & 1) else -1.0
            return heading_rad + sign*math.pi/2.0
        return heading_rad if rr_ema < 0 else heading_rad + math.pi

    def _last_distance(self, bssid: str):
        ap = self.aps.get(bssid)
        if ap and ap["r_ema"] is not None:
            return float(ap["r_ema"])
        s = self.seen.get(bssid)
        if s and s.get("last_rssi") is not None:
            return rssi_to_distance(s["last_rssi"], self.calib["rssi_at_1m"], self.calib["path_loss_n"], self.calib["d0"])
        return None

    def _is_recent(self, bssid: str):
        s = self.seen.get(bssid)
        if not s: return False
        last_m = s.get("last_seen_mono")
        if last_m is None: return False
        return (now_mono() - float(last_m)) <= self.prune_age_s
    # -----------------------------------------------------

    def periodic(self, gps_thread):
        while not self._stop.is_set():
            t=time.time()
            if t - self._last_write >= self.write_period and self.origin is not None:
                self._write_geojson(); self._last_write=t
            self._gps_fix=gps_thread.get_fix(max_age_s=10.0)
            time.sleep(0.25)
    def stop(self): self._stop.set()

    # ---- GUI in main thread ----
    def start_radar_ui(self):
        if not self.show_radar: return
        try:
            import matplotlib; matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
        except Exception as e:
            print(f"[!] Matplotlib/Tk not available: {e}", file=sys.stderr); return
        self._plt=plt; plt.ion()
        self._radar_fig, self._radar_ax = plt.subplots(subplot_kw={'projection':'polar'})
        self._radar_ax.set_title("Wi-Fi Radar — EKF estimates (●) vs provisional (⨯)")
        self._radar_ax.set_ylim(0, self.radar_radius_m); self._radar_ax.grid(True)
        print(f"[i] Matplotlib backend: {plt.get_backend()} DISPLAY={os.environ.get('DISPLAY')}")
        plt.show(block=False)

    def update_radar_ui(self):
        if not self.show_radar or self._plt is None or self._radar_ax is None:
            return
        fix=self._gps_fix
        if not fix:
            self._plt.pause(0.01); return
        lat0,lon0=fix["lat"],fix["lon"]
        speed = fix.get("speed") or 0.0
        heading_rad = math.radians(fix["track"]) if fix.get("track") is not None else None

        est_thetas, est_rs, est_labels = [], [], []
        prov_thetas, prov_rs, prov_labels = [], [], []

        with self.lock:
            # Estimated APs (EKF); only show if recently seen
            for bssid, ap in self.aps.items():
                if not self._is_recent(bssid):  # hide when out of range
                    continue
                est=ap.get("est")
                if est:
                    x,y=latlon_to_xy(est["lat"],est["lon"],lat0,lon0)
                    r=math.hypot(x,y)
                    if r<=self.radar_radius_m:
                        theta=(math.degrees(math.atan2(x,y))%360.0)*math.pi/180.0
                        est_thetas.append(theta); est_rs.append(r); est_labels.append(bssid)

            # Provisional (distance-only) markers (also require recent)
            for bssid, s in self.seen.items():
                if not self._is_recent(bssid): continue
                ap = self.aps.get(bssid)
                has_est = bool(ap and ap.get("est"))
                if has_est: continue
                d = self._last_distance(bssid)
                if d is None: continue
                r = min(d, self.radar_radius_m)
                rr_ema = ap.get("rr_ema") if ap else None
                theta = self._provisional_theta(bssid, speed, heading_rad, rr_ema)
                prov_thetas.append(theta); prov_rs.append(r); prov_labels.append(bssid)

        ax=self._radar_ax
        ax.clear()
        ax.set_ylim(0, self.radar_radius_m)
        ax.set_title("Wi-Fi Radar — EKF estimates (●) vs provisional (⨯)")
        ax.grid(True)

        if prov_thetas: ax.scatter(prov_thetas, prov_rs, s=25, marker='x')
        if est_thetas:  ax.scatter(est_thetas,  est_rs,  s=36)

        # Label ALL points with BSSID (trim to 17 chars)
        for th, rr, lab in zip(est_thetas, est_rs, est_labels):
            ax.text(th, rr, (lab or "")[:17], fontsize=8)
        for th, rr, lab in zip(prov_thetas, prov_rs, prov_labels):
            ax.text(th, rr, (lab or "")[:17], fontsize=7)

        self._plt.pause(0.01)

# ---- Scapy helpers ----
def parse_ssid(pkt):
    ssid=None
    try:
        elt=pkt.getlayer(Dot11Elt)
        while isinstance(elt, Dot11Elt):
            if elt.ID==0:
                try:
                    s=elt.info.decode(errors="ignore")
                    ssid=s if s!="" else None
                except Exception:
                    ssid=None
                break
            elt=elt.payload.getlayer(Dot11Elt)
    except Exception: pass
    return ssid

def get_rssi_dbm(pkt):
    try:
        if pkt.haslayer(RadioTap):
            rt = pkt[RadioTap]
            for key in ("dBm_AntSignal","dBm_ant_signal","dbm_antsignal","antenna_signal"):
                val = getattr(rt, key, None)
                if val is not None:
                    return int(val)
            if hasattr(rt, "fields"):
                for key in ("dBm_AntSignal","dBm_ant_signal","dbm_antsignal","antenna_signal"):
                    if key in rt.fields and rt.fields[key] is not None:
                        return int(rt.fields[key])
    except Exception:
        pass
    try:
        if hasattr(pkt, "dBm_AntSignal"):
            return int(pkt.dBm_AntSignal)
    except Exception:
        pass
    return None

def get_channel_from_pkt(pkt):
    freq=None
    try:
        if pkt.haslayer(RadioTap):
            freq=getattr(pkt[RadioTap], "ChannelFrequency", None)
    except Exception: pass
    ch=freq_to_channel(freq) if freq else None
    if ch is None:
        try:
            elt=pkt.getlayer(Dot11Elt)
            while isinstance(elt, Dot11Elt):
                if elt.ID==3 and len(elt.info)>=1:
                    ch=int(elt.info[0]); break
                elt=elt.payload.getlayer(Dot11Elt)
        except Exception: pass
    return ch

def scapy_sniff_thread(iface, radar: WifiRadar, gps: "GpsThread", stop_evt: threading.Event,
                       gps_max_age_s=10.0, require_3d_fix=False, count_without_gps=False):
    def handler(pkt):
        if stop_evt.is_set(): return True
        try:
            if not pkt.haslayer(Dot11): return
            d11=pkt[Dot11]
            if d11.type!=0 or d11.subtype not in (5,8):
                return
            bssid=d11.addr3
            if not bssid: return
            rssi=get_rssi_dbm(pkt)
            ssid=parse_ssid(pkt)
            ch=get_channel_from_pkt(pkt)

            radar.register_seen(bssid, ssid, rssi, ch)

            fix=gps.get_fix(max_age_s=gps_max_age_s, require_3d=require_3d_fix)
            if not fix:
                if not count_without_gps:
                    return
                return
            radar.add_observation(bssid=bssid, ssid=ssid, fix=fix, rssi_dbm=rssi, channel=ch)
        except Exception:
            pass
    try:
        sniff(iface=iface, store=False, prn=handler, stop_filter=lambda p: stop_evt.is_set())
    except Exception as e:
        print(f"[!] Sniff error on {iface}: {e}", file=sys.stderr)

# ---- Calib I/O ----
def load_calib(path, fallback_rssi1m, fallback_n, fallback_d0):
    if not path: return {"rssi_at_1m": fallback_rssi1m, "path_loss_n": fallback_n, "d0": fallback_d0}
    with open(path,"r",encoding="utf-8") as f: data=yaml.safe_load(f) or {}
    return {"rssi_at_1m": float(data.get("rssi_at_1m",fallback_rssi1m)),
            "path_loss_n": float(data.get("path_loss_n",fallback_n)),
            "d0": float(data.get("d0",fallback_d0))}

def save_calib(path, rssi_at_1m, path_loss_n, d0):
    data={"rssi_at_1m":float(rssi_at_1m),"path_loss_n":float(path_loss_n),
          "d0":float(d0),"saved_at":now_iso()}
    with open(path,"w",encoding="utf-8") as f: yaml.safe_dump(data,f)
    print(f"[i] Calibration saved to {path}")

# ---- Main ----
def main():
    ap=argparse.ArgumentParser(description="Wi-Fi Radar: EKF trilateration with GPS heading/speed")
    ap.add_argument("--iface", required=True)
    ap.add_argument("--gpsd", default="127.0.0.1:2947")
    ap.add_argument("--out-geojson", default="ap_estimates.geojson")
    ap.add_argument("--min-rssi", type=int, default=-90)
    ap.add_argument("--min-samples", type=int, default=3)
    ap.add_argument("--write-period", type=float, default=2.0)
    ap.add_argument("--calib"); ap.add_argument("--rssi-at-1m", type=float, default=-45.0)
    ap.add_argument("--n", dest="path_loss_n", type=float, default=2.5); ap.add_argument("--d0", type=float, default=1.0)
    ap.add_argument("--save-calib")
    ap.add_argument("--gps-max-age", type=float, default=10.0); ap.add_argument("--require-3d-fix", action="store_true")
    ap.add_argument("--mqtt-host", default=None); ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--mqtt-user", default=None); ap.add_argument("--mqtt-pass", default=None)
    ap.add_argument("--mqtt-base", default="wifi_radar"); ap.add_argument("--mqtt-qos", type=int, default=0)
    ap.add_argument("--mqtt-retain", action="store_true")
    ap.add_argument("--show-radar", action="store_true"); ap.add_argument("--radar-radius", type=float, default=120.0)
    ap.add_argument("--count-without-gps", action="store_true", help="Count APs even if GPS fix is missing")
    # filters / prune
    ap.add_argument("--rssi-display-min", type=int, default=-70, help="Accept frames only if RSSI >= this dBm")
    ap.add_argument("--prune-age", type=float, default=20.0, help="Hide APs not seen for this many seconds")
    # EKF / filters
    ap.add_argument("--ekf-q", type=float, default=1.0, help="EKF process noise std (m)")
    ap.add_argument("--range-ema-alpha", type=float, default=0.4, help="EMA alpha for range smoothing [0..1]")
    ap.add_argument("--rr-ema-alpha", type=float, default=0.5, help="EMA alpha for range-rate smoothing [0..1]")
    ap.add_argument("--rr-eps", type=float, default=0.12, help="m/s threshold for side-on vs toward/away")
    args=ap.parse_args()

    if args.save_calib:
        save_calib(args.save_calib, args.rssi_at_1m, args.path_loss_n, args.d0); return
    calib=load_calib(args.calib, args.rssi_at_1m, args.path_loss_n, args.d0)
    print(f"[i] Using calibration: {calib}")
    print(f"[i] RSSI filter: >= {args.rssi_display_min} dBm | prune age: {args.prune_age}s")

    # GPS
    host,port=(args.gpsd.split(":")+["2947"])[:2]; gps=GpsThread(host, int(port)); gps.start()

    # MQTT
    mc=None
    if args.mqtt_host:
        mc=MqttClient(host=args.mqtt_host, port=args.mqtt_port, username=args.mqtt_user,
                      password=args.mqtt_pass, base_topic=args.mqtt_base, qos=args.mqtt_qos, retain=args.mqtt_retain)

    radar=WifiRadar(iface=args.iface, out_geojson=args.out_geojson, calib=calib,
                    min_rssi=args.min_rssi, min_samples=args.min_samples, write_period=args.write_period,
                    mqtt_client=mc, show_radar=args.show_radar, radar_radius_m=args.radar_radius,
                    count_without_gps=args.count_without_gps, rssi_display_min=args.rssi_display_min,
                    prune_age_s=args.prune_age, ekf_q=args.ekf_q, range_ema_alpha=args.range_ema_alpha,
                    rr_ema_alpha=args.rr_ema_alpha, rr_eps=args.rr_eps)
    radar.start_radar_ui()

    stop_evt=threading.Event()
    sniffer=threading.Thread(target=scapy_sniff_thread,
                             args=(args.iface, radar, gps, stop_evt, args.gps_max_age, args.require_3d_fix, args.count_without_gps),
                             daemon=True); sniffer.start()
    periodic=threading.Thread(target=radar.periodic, args=(gps,), daemon=True); periodic.start()

    def handle_sig(sig, frame):
        print("\n[!] Stopping ..."); stop_evt.set(); radar.stop(); gps.stop()
        if mc: mc.close(); time.sleep(0.4); sys.exit(0)
    signal.signal(signal.SIGINT, handle_sig); signal.signal(signal.SIGTERM, handle_sig)

    try:
        last_print=0.0
        while True:
            time.sleep(0.5)
            mode_str, age_s, alt = gps.get_status()
            with radar.lock:
                ap_detected = len(radar.seen)
                ap_est      = sum(1 for a in radar.aps.values() if a.get("est"))
            t=time.time()
            if t-last_print>3.5:
                gps_part=f"GPS:{mode_str}"
                if age_s is not None: gps_part+=f" age={age_s}s"
                if alt is not None: gps_part+=f" alt={alt:.1f}m"
                print(f"[{now_iso()}] APs detected: {ap_detected} | estimated (EKF): {ap_est} | {gps_part} | geojson: {args.out_geojson}")
                last_print=t
            radar.update_radar_ui()
    except KeyboardInterrupt:
        handle_sig(None, None)

if __name__=="__main__":
    main()
