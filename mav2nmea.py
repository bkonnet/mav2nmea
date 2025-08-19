#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, socket, threading, time, math, sys
from typing import Optional, Tuple, List
from pymavlink import mavutil

# =========================
# Config vía variables de entorno
# =========================
# Ejemplos:
#   MAVLINK_ENDPOINT="udp:0.0.0.0:14552"
#   MAVLINK_ENDPOINT="udp:127.0.0.1:14550"
#   MAVLINK_ENDPOINT="tcp:192.168.1.10:5760"
#   MAVLINK_ENDPOINT="serial:/dev/ttyACM0:115200"
MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udp:0.0.0.0:14552")

NMEA_TCP_PORT = int(os.getenv("NMEA_TCP_PORT", "10110"))
# coma-separados, p.ej.: "127.0.0.1:10110,192.168.1.100:9999"
NMEA_UDP_TARGETS = [t.strip() for t in os.getenv("NMEA_UDP_TARGETS", "").split(",") if t.strip()]

RATE_HZ = float(os.getenv("RATE_HZ", "2.0"))   # cuántas oraciones/segundo como máx
TALKER = os.getenv("TALKER", "GP")             # GP (GPS), GN (multiconstelación), EC (eCompass), etc.
SENTENCES = os.getenv("SENTENCES", "RMC,VTG").upper().split(",")  # qué emitir

MAX_TCP_CLIENTS = int(os.getenv("MAX_TCP_CLIENTS", "20"))
TCP_BACKLOG = 16

# =========================
# Estado global
# =========================
tcp_clients: List[socket.socket] = []
udp_socks: List[Tuple[socket.socket, Tuple[str, int]]] = []
lock = threading.Lock()

# Estado de navegación más reciente
class NavState:
    def __init__(self):
        self.lat_deg: Optional[float] = None     # grados
        self.lon_deg: Optional[float] = None     # grados
        self.sog_ms: Optional[float] = None      # m/s
        self.cog_deg: Optional[float] = None     # deg (0-360)
        self.has_fix: bool = False               # fix válido
        self.date_utc: Optional[Tuple[int,int,int]] = None  # (dd, mm, yy)
        self.time_utc: Optional[Tuple[int,int,float]] = None # (hh, mm, ss.s)

state = NavState()

# =========================
# Utilidades
# =========================
def nmea_checksum(s: str) -> str:
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f"{c:02X}"

def deg_to_nmea_lat(lat_deg: float) -> Tuple[str, str]:
    hemi = 'N' if lat_deg >= 0 else 'S'
    absdeg = abs(lat_deg)
    d = int(absdeg)
    m = (absdeg - d) * 60.0
    # DDMM.MMMMM (5 decimales ~ 0.00001 min ~ 1.85 m)
    return f"{d:02d}{m:07.5f}", hemi

def deg_to_nmea_lon(lon_deg: float) -> Tuple[str, str]:
    hemi = 'E' if lon_deg >= 0 else 'W'
    absdeg = abs(lon_deg)
    d = int(absdeg)
    m = (absdeg - d) * 60.0
    # DDDMM.MMMMM
    return f"{d:03d}{m:07.5f}", hemi

def now_utc_hhmmss() -> Tuple[int,int,float,Tuple[int,int,int]]:
    t = time.gmtime()
    return t.tm_hour, t.tm_min, t.tm_sec + 0.0, (t.tm_mday, t.tm_mon, int(str(t.tm_year)[2:]))

def build_rmc(lat: float, lon: float, sog_ms: float, cog_deg: float,
              talker: str = "GP",
              dmy: Optional[Tuple[int,int,int]] = None,
              hms: Optional[Tuple[int,int,float]] = None) -> str:
    if hms is None or dmy is None:
        hh, mm, ss, (dd, mo, yy) = now_utc_hhmmss()
    else:
        hh, mm, ss = hms
        dd, mo, yy = dmy

    lat_str, ns = deg_to_nmea_lat(lat)
    lon_str, ew = deg_to_nmea_lon(lon)
    sog_kn = sog_ms * 1.94384
    body = f"{talker}RMC,{hh:02d}{mm:02d}{ss:06.3f},A,{lat_str},{ns},{lon_str},{ew},{sog_kn:.2f},{cog_deg:.1f},{dd:02d}{mo:02d}{yy:02d},,,A"
    return f"${body}*{nmea_checksum(body)}"

def build_vtg(cog_deg: float, sog_ms: float, talker: str = "GP") -> str:
    sog_kn = sog_ms * 1.94384
    sog_kmh = sog_ms * 3.6
    body = f"{talker}VTG,{cog_deg:.1f},T,,M,{sog_kn:.2f},N,{sog_kmh:.2f},K"
    return f"${body}*{nmea_checksum(body)}"

def clamp_heading(degval: float) -> float:
    return (degval % 360.0 + 360.0) % 360.0

# =========================
# Emisión NMEA
# =========================
def broadcast_lines(lines: List[str]):
    if not lines:
        return
    data = ("\r\n".join(lines) + "\r\n").encode("ascii", errors="ignore")
    # TCP
    with lock:
        for c in tcp_clients[:]:
            try:
                c.sendall(data)
            except Exception:
                try:
                    c.close()
                finally:
                    tcp_clients.remove(c)
    # UDP
    for s, addr in udp_socks:
        try:
            s.sendto(data, addr)
        except Exception:
            # no detenemos todo por un fallo puntual
            pass

# =========================
# Lector MAVLink
# =========================
def mavlink_loop():
    global state
    print(f"[mav2nmea] Conectando MAVLink: {MAVLINK_ENDPOINT}", flush=True)
    m = mavutil.mavlink_connection(MAVLINK_ENDPOINT, dialect="ardupilotmega")
    # Espera heartbeat para saber que hay tráfico
    try:
        m.wait_heartbeat(timeout=10)
        print("[mav2nmea] Heartbeat OK", flush=True)
    except Exception:
        print("[mav2nmea] WARNING: sin heartbeat inicial, continúo esperando mensajes…", flush=True)

    min_period = 1.0 / max(RATE_HZ, 0.1)
    last_sent = 0.0

    while True:
        try:
            msg = m.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue

            mtype = msg.get_type()

            # Preferimos GPS_RAW_INT cuando está disponible (tiene tiempo GPS)
            if mtype == "GPS_RAW_INT":
                # Validación de fix
                fix_type = getattr(msg, "fix_type", 0) or 0
                state.has_fix = fix_type >= 2  # 2D o mejor

                lat = getattr(msg, "lat", None)
                lon = getattr(msg, "lon", None)
                if lat is not None and lon is not None:
                    state.lat_deg = lat / 1e7
                    state.lon_deg = lon / 1e7

                vel = getattr(msg, "vel", None)  # cm/s (horizontal)
                if vel is not None and vel >= 0:
                    state.sog_ms = vel / 100.0

                cog = getattr(msg, "cog", None)  # centideg
                if cog is not None and cog != 65535:
                    state.cog_deg = clamp_heading(cog / 100.0)

                # Tiempo UTC desde time_usec si parece plausible (epoch μs)
                tu = getattr(msg, "time_usec", None)
                if isinstance(tu, int) and tu > 10**12:  # > ~2001-09-09
                    tt = time.gmtime(tu / 1e6)
                    state.time_utc = (tt.tm_hour, tt.tm_min, float(tt.tm_sec))
                    state.date_utc = (tt.tm_mday, tt.tm_mon, int(str(tt.tm_year)[2:]))

            elif mtype == "GLOBAL_POSITION_INT":
                # Posición principal
                state.lat_deg = getattr(msg, "lat", 0) / 1e7
                state.lon_deg = getattr(msg, "lon", 0) / 1e7
                state.has_fix = True

                # Velocidad horizontal con vx,vy (cm/s)
                vx = getattr(msg, "vx", None)
                vy = getattr(msg, "vy", None)
                if vx is not None and vy is not None:
                    state.sog_ms = math.hypot(vx, vy) / 100.0

                # Rumbo: primero hdg (cdeg), si inválido, derivar de vx/vy
                hdg = getattr(msg, "hdg", None)
                if hdg is not None and hdg != 65535:
                    state.cog_deg = clamp_heading(hdg / 100.0)
                elif vx is not None and vy is not None:
                    state.cog_deg = clamp_heading(math.degrees(math.atan2(vy, vx)))

                # Tiempo: si no tenemos de GPS, usa del sistema
                if state.time_utc is None or state.date_utc is None:
                    hh, mm, ss, (dd, mo, yy) = now_utc_hhmmss()
                    state.time_utc = (hh, mm, ss)
                    state.date_utc = (dd, mo, yy)

            # Rate limit de salida
            now = time.time()
            if now - last_sent >= min_period:
                last_sent = now
                if state.has_fix and state.lat_deg is not None and state.lon_deg is not None \
                   and state.sog_ms is not None and state.cog_deg is not None:
                    lines = []
                    if "RMC" in SENTENCES:
                        lines.append(
                            build_rmc(state.lat_deg, state.lon_deg, state.sog_ms, state.cog_deg,
                                      talker=TALKER,
                                      dmy=state.date_utc, hms=state.time_utc)
                        )
                    if "VTG" in SENTENCES:
                        lines.append(build_vtg(state.cog_deg, state.sog_ms, talker=TALKER))
                    if lines:
                        broadcast_lines(lines)

        except Exception as e:
            # No matar el hilo por un mensaje inesperado
            print(f"[mav2nmea] WARN procesando MAVLink: {e}", file=sys.stderr, flush=True)
            time.sleep(0.05)

# =========================
# Servidores de salida
# =========================
def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", NMEA_TCP_PORT))
    srv.listen(TCP_BACKLOG)
    print(f"[mav2nmea] NMEA TCP escuchando en 0.0.0.0:{NMEA_TCP_PORT}", flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with lock:
            if len(tcp_clients) >= MAX_TCP_CLIENTS:
                try:
                    conn.close()
                finally:
                    continue
            tcp_clients.append(conn)
        print(f"[mav2nmea] TCP cliente conectado: {addr}", flush=True)

def setup_udp_targets():
    global udp_socks
    for t in NMEA_UDP_TARGETS:
        try:
            ip, port = t.split(":")
            ip = ip.strip()
            if ip.startswith("::ffff:"):
                ip = ip[7:]  # normaliza IPv4-mapeado
            port = int(port)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # forzamos IPv4
            udp_socks.append((s, (ip, port)))
            print(f"[mav2nmea] NMEA UDP destino: {ip}:{port}", flush=True)
        except Exception as e:
            print(f"[mav2nmea] WARNING destino UDP inválido '{t}': {e}", flush=True)

# =========================
# Main
# =========================
if __name__ == "__main__":
    print(f"[mav2nmea] Endpoint MAVLink: {MAVLINK_ENDPOINT}", flush=True)
    print(f"[mav2nmea] RATE_HZ={RATE_HZ}  TALKER={TALKER}  SENTENCES={SENTENCES}", flush=True)
    if NMEA_UDP_TARGETS:
        print(f"[mav2nmea] UDP targets: {NMEA_UDP_TARGETS}", flush=True)
    setup_udp_targets()
    t1 = threading.Thread(target=mavlink_loop, daemon=True)
    t2 = threading.Thread(target=tcp_server, daemon=True)
    t1.start(); t2.start()
    # Mantener vivo el proceso principal
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
