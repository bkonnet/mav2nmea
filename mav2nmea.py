#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mav2nmea.py — Puente MAVLink -> NMEA (UDP->TCP/UDP) con fixes
- Escucha MAVLink por UDP (MAVLINK_UDP_PORT, default 14552)
- Emite NMEA (RMC/GGA/VTG/HDT) por TCP (NMEA_TCP_PORT, default 10110; 0=off) y UDP (NMEA_UDP_TARGETS)
- RATE_HZ (default 2.0), MAX_TCP_CLIENTS (default 20)
Fixes: limpieza de sockets, límite de clientes, VTG sin formato condicional inválido.
"""
import os, sys, socket, threading, time
from datetime import datetime, timezone
from typing import Optional, Tuple, List

def getenv_int(name, default):
    try:
        v = os.getenv(name)
        return int(v) if v not in (None, "") else default
    except: return default

def getenv_float(name, default):
    try:
        v = os.getenv(name)
        return float(v) if v not in (None, "") else default
    except: return default

def getenv_str(name, default):
    v = os.getenv(name)
    return v if v not in (None, "") else default

MAVLINK_UDP_PORT = getenv_int("MAVLINK_UDP_PORT", 14552)
NMEA_TCP_PORT    = getenv_int("NMEA_TCP_PORT",   10110)
RATE_HZ          = getenv_float("RATE_HZ",       2.0)
MAX_TCP_CLIENTS  = getenv_int("MAX_TCP_CLIENTS", 20)
LOG_LEVEL        = getenv_str("LOG_LEVEL",       "INFO").upper()
UDP_TARGETS_STR  = getenv_str("NMEA_UDP_TARGETS","127.0.0.1:10110")

UDP_TARGETS: List[Tuple[str,int]] = []
for part in UDP_TARGETS_STR.split(","):
    part = part.strip()
    if not part or ":" not in part: continue
    host, port = part.rsplit(":", 1)
    try: UDP_TARGETS.append((host, int(port)))
    except: pass

def log(level, msg):
    levels = ["DEBUG","INFO","WARN","ERROR"]
    if level not in levels: level = "INFO"
    if levels.index(level) >= levels.index(LOG_LEVEL):
        now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"{now} [{level}] {msg}", flush=True)

state = {"lat":None,"lon":None,"alt":None,"hdg":None,"cog":None,"sog_mps":None,"hdop":None,"sats":None,"fix":False}
lock = threading.Lock()

try:
    from pymavlink import mavutil
except Exception as e:
    log("ERROR", f"pymavlink no disponible: {e} (instala con: pip install pymavlink)")
    sys.exit(1)

def to_degmin(n: float, is_lat: bool) -> Tuple[str,str]:
    sign = 1 if n >= 0 else -1
    nabs = abs(n); d = int(nabs); m = (nabs - d) * 60.0
    dStr = f"{d:02d}" if is_lat else f"{d:03d}"
    mStr = f"{m:06.3f}"
    hemi = ("N" if sign >= 0 else "S") if is_lat else ("E" if sign >= 0 else "W")
    return f"{dStr}{mStr}", hemi

def nmea_cs(s: str) -> str:
    c = 0
    for ch in s[1:]: c ^= ord(ch)
    return f"{c:02X}"

def build_nmea() -> Optional[bytes]:
    with lock: s = dict(state)
    if s["lat"] is None or s["lon"] is None: return None

    now = datetime.now(timezone.utc)
    t = now.strftime("%H%M%S"); d = now.strftime("%d%m%y")

    latdm, lathemi = to_degmin(s["lat"], True)
    londm, lonhemi = to_degmin(s["lon"], False)

    sog_kn = s["sog_mps"]*1.943844 if s["sog_mps"] is not None else None
    sogN = f"{sog_kn:.2f}" if sog_kn is not None else ""
    kmh_val = s["sog_mps"]*3.6 if s["sog_mps"] is not None else None
    kmh_str = f"{kmh_val:.2f}" if kmh_val is not None else ""
    cog = f"{s['cog']:.1f}" if s.get("cog") is not None else ""
    hdop = f"{s['hdop']:.1f}" if s.get("hdop") is not None else ""
    sats = f"{s['sats']}" if s.get("sats") is not None else ""
    altM = f"{s['alt']:.1f}" if s.get("alt") is not None else ""
    hdg = f"{s['hdg']:.1f}" if s.get("hdg") is not None else ""
    status = "A" if s.get("fix") else "V"

    rmc = f"$GPRMC,{t}.00,{status},{latdm},{lathemi},{londm},{lonhemi},{sogN},{cog},{d},,,A"
    rmc = f"{rmc}*{nmea_cs(rmc)}\r\n"
    gga = f"$GPGGA,{t}.00,{latdm},{lathemi},{londm},{lonhemi},{'1' if status=='A' else '0'},{sats},{hdop},{altM},M,,M,,"
    gga = f"{gga}*{nmea_cs(gga)}\r\n"
    vtg = f"$GPVTG,{cog},T,,M,{sogN},N,{kmh_str},K"
    vtg = f"{vtg}*{nmea_cs(vtg)}\r\n"
    hdt = f"$GPHDT,{hdg},T"
    hdt = f"{hdt}*{nmea_cs(hdt)}\r\n"
    return (rmc+gga+vtg+hdt).encode("ascii")

_tcp_broadcast = None
def tcp_server(port: int):
    global _tcp_broadcast
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(8)
    srv.settimeout(1.0)
    clients = set()
    def safe_send(data: bytes):
        dead=[]
        for c in list(clients):
            try: c.sendall(data)
            except Exception:
                dead.append(c)
        for c in dead:
            try: c.close()
            except: pass
            clients.discard(c)
    _tcp_broadcast = safe_send
    print(f"[mav2nmea] TCP NMEA server :{port}")
    try:
        while True:
            try:
                c, addr = srv.accept()
                if len(clients) >= MAX_TCP_CLIENTS:
                    try: c.close()
                    except: pass
                    continue
                c.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                c.settimeout(10)
                clients.add(c)
                print(f"[mav2nmea] client {addr} connected (clients={len(clients)})")
            except socket.timeout:
                pass
            except OSError:
                time.sleep(0.05)
    finally:
        for c in list(clients):
            try: c.close()
            except: pass
        srv.close()

def nmea_loop(rate_hz: float, udp_targets: List[Tuple[str,int]]):
    period = 1.0 / max(0.1, rate_hz)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[mav2nmea] NMEA loop {rate_hz} Hz, UDP targets={udp_targets}")
    while True:
        pkt = build_nmea()
        if pkt:
            if _tcp_broadcast: _tcp_broadcast(pkt)
            for (host,port) in udp_targets:
                try: udp_sock.sendto(pkt, (host, port))
                except Exception as e: print("WARN:", e)
        time.sleep(period)

def mavlink_loop(udp_port: int):
    conn = mavutil.mavlink_connection(f"udp:0.0.0.0:{udp_port}", source_system=1, source_component=1, dialect="ardupilotmega")
    print(f"[mav2nmea] Listening MAVLink on udp:0.0.0.0:{udp_port}")
    while True:
        try: m = conn.recv_match(blocking=True, timeout=1.0)
        except Exception: m = None
        if m is None: continue
        try: msg = m.to_dict()
        except Exception: continue
        name = msg.get('mavpackettype') or msg.get('_type') or ""
        with lock:
            if name == "GLOBAL_POSITION_INT":
                lat = msg.get('lat'); lon = msg.get('lon')
                if isinstance(lat,(int,float)) and isinstance(lon,(int,float)):
                    state['lat'] = lat/1e7; state['lon'] = lon/1e7; state['fix'] = True
                alt = msg.get('alt'); 
                if isinstance(alt,(int,float)): state['alt'] = alt/1000.0
                hdg = msg.get('hdg')
                if isinstance(hdg,(int,float)) and hdg >= 0: state['hdg'] = hdg/100.0
                vx = msg.get('vx'); vy = msg.get('vy')
                if isinstance(vx,(int,float)) and isinstance(vy,(int,float)):
                    gs = ((vx*vx + vy*vy) ** 0.5 ) / 100.0
                    state['sog_mps'] = gs
                    import math
                    state['cog'] = (math.degrees(math.atan2(vy, vx)) + 360.0) % 360.0
            elif name == "GPS_RAW_INT":
                lat = msg.get('lat'); lon = msg.get('lon')
                if isinstance(lat,(int,float)) and isinstance(lon,(int,float)) and lat!=0 and lon!=0:
                    state['lat'] = lat/1e7; state['lon'] = lon/1e7; state['fix'] = (msg.get('fix_type',0) >= 2)
                h_acc = msg.get('h_acc')
                if isinstance(h_acc,(int,float)): state['hdop'] = (h_acc/100.0)/5.0
                sats = msg.get('satellites_visible')
                if isinstance(sats,(int,float)): state['sats'] = int(sats)
            elif name == "ATTITUDE":
                yaw = msg.get('yaw')
                if isinstance(yaw,(int,float)):
                    import math
                    state['hdg'] = (math.degrees(yaw)+360.0)%360.0
            elif name == "VFR_HUD":
                gs = msg.get('groundspeed')
                if isinstance(gs,(int,float)): state['sog_mps'] = float(gs)
                heading = msg.get('heading')
                if isinstance(heading,(int,float)): state['hdg'] = float(heading)

def main():
    print(f"[mav2nmea] Config: MAVLINK_UDP_PORT={MAVLINK_UDP_PORT}, NMEA_TCP_PORT={NMEA_TCP_PORT}, RATE_HZ={RATE_HZ}, UDP_TARGETS={UDP_TARGETS}")
    th = []
    t1 = threading.Thread(target=mavlink_loop, args=(MAVLINK_UDP_PORT,), daemon=True); t1.start(); th.append(t1)
    if NMEA_TCP_PORT and int(NMEA_TCP_PORT) > 0:
        t2 = threading.Thread(target=tcp_server, args=(NMEA_TCP_PORT,), daemon=True); t2.start(); th.append(t2)
    t3 = threading.Thread(target=nmea_loop, args=(RATE_HZ, UDP_TARGETS), daemon=True); t3.start(); th.append(t3)
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted, exiting...")

if __name__ == "__main__":
    main()
