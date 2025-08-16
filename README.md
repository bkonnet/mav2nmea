# mav2nmea — MAVLink → NMEA bridge

Convierte telemetría MAVLink en frases NMEA para usar en OpenCPN.

## Instalación

```bash
git clone https://github.com/bkonnet/mav2nmea.git
cd mav2nmea
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Ejecución

```bash
source venv/bin/activate
python mav2nmea.py
```

## Variables de entorno

- MAVLINK_UDP_PORT (default 14552)
- NMEA_TCP_PORT (default 10110)
- NMEA_UDP_TARGETS ("ip:port,ip:port")

## Servicio systemd

Copia `mav2nmea.service` a `/etc/systemd/system/` y habilita con:

```bash
sudo systemctl daemon-reload
sudo systemctl enable mav2nmea
sudo systemctl start mav2nmea
```

