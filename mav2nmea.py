#!/usr/bin/env python3
import os, socket, threading, time
from pymavlink import mavutil

MAVLINK_UDP_PORT = int(os.getenv("MAVLINK_UDP_PORT", "14552"))
NMEA_TCP_PORT = int(os.getenv("NMEA_TCP_PORT", "10110"))
NMEA_UDP_TARGETS = os.getenv("NMEA_UDP_TARGETS", "").split(",") if os.getenv("NMEA_UDP_TARGETS") else []
RATE_HZ = float(os.getenv("RATE_HZ", "2.0"))
MAX_TCP_CLIENTS = int(os.getenv("MAX_TCP_CLIENTS", "20"))

tcp_clients = []
lock = threading.Lock()

def mavlink_loop():
    m = mavutil.mavlink_connection(f"udp:0.0.0.0:{MAVLINK_UDP_PORT}")
    last_sent = 0
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            continue
        now = time.time()
        if now - last_sent < 1.0 / RATE_HZ:
            continue
        last_sent = now
        if msg.get_type() == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            sog = msg.vel / 100.0 * 1.94384
            cog = msg.hdg / 100.0 if msg.hdg != 65535 else 0
            nmea = f"$GPRMC,{time.strftime('%H%M%S')},A,{abs(lat):.5f},{'N' if lat>=0 else 'S'},{abs(lon):.5f},{'E' if lon>=0 else 'W'},{sog:.2f},{cog:.2f},{time.strftime('%d%m%y')},,,A*00\r\n"
            broadcast(nmea.encode())

def broadcast(data: bytes):
    with lock:
        for c in tcp_clients[:]:
            try:
                c.sendall(data)
            except:
                tcp_clients.remove(c)
    for target in NMEA_UDP_TARGETS:
        if not target.strip():
            continue
        ip, port = target.split(":")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(data, (ip, int(port)))
        s.close()

def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", NMEA_TCP_PORT))
    srv.listen(MAX_TCP_CLIENTS)
    while True:
        conn, addr = srv.accept()
        with lock:
            tcp_clients.append(conn)

if __name__ == "__main__":
    t1 = threading.Thread(target=mavlink_loop, daemon=True)
    t2 = threading.Thread(target=tcp_server, daemon=True)
    t1.start(); t2.start()
    t1.join(); t2.join()
