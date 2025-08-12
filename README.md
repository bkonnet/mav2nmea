# mav2nmea (con fixes)
Puente MAVLink → NMEA (TCP/UDP).

## Instalación rápida
```bash
chmod +x install.sh
sudo ./install.sh
sudo systemctl status mav2nmea
```

## Variables (opcional)
- `MAVLINK_UDP_PORT` (default 14552)
- `NMEA_TCP_PORT` (default 10110; 0 = deshabilitado)
- `NMEA_UDP_TARGETS` (CSV, ej: "192.168.1.255:10110,10.0.0.5:10110")
- `RATE_HZ` (default 2.0)
- `MAX_TCP_CLIENTS` (default 20)
