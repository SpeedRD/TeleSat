# TelemetrySat Ground Station

Flask + SocketIO dashboard for the TELESAT-1 OBC. Connects over USB serial,
decodes CCSDS TM packets, and sends telecommands.

## Setup

```bash
pip3 install -r requirements.txt
python3 app.py
```

Open `http://<raspberry-pi-ip>:5000` in any browser.

## Serial port

Default port is `/dev/ttyUSB0` at 115200 baud. Override with an environment variable:

```bash
SERIAL_PORT=/dev/ttyACM0 python3 app.py
```

Check which port the ESP32 appears on with `ls /dev/tty*` after plugging in USB.
