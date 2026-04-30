from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import os
from serial_reader import SerialReader
from tc_builder import build_tc_packet

PORT = os.environ.get('SERIAL_PORT', '/dev/ttyUSB1')
BAUD = 115200

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', os.urandom(24).hex())
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

reader = None
connected_clients = 0
boot_time = time.time()


def on_packet(pkt):
    socketio.emit('telemetry', pkt)


@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def on_connect():
    global connected_clients
    connected_clients += 1


@socketio.on('disconnect')
def on_disconnect():
    global connected_clients
    connected_clients = max(0, connected_clients - 1)


@socketio.on('command')
def on_command(data):
    cmd = data.get('cmd', '')
    try:
        tc = build_tc_packet(cmd)
        reader.send(tc)
        socketio.emit('cmd_ack', {'cmd': cmd, 'status': 'sent'})
    except Exception as e:
        socketio.emit('cmd_ack', {'cmd': cmd, 'status': 'error', 'msg': str(e)})


if __name__ == '__main__':
    reader = SerialReader(PORT, BAUD, on_packet)
    reader.start()
    print(f'[GS] TelemetrySat ground station starting on http://0.0.0.0:5000')
    print(f'[GS] Serial port: {PORT} at {BAUD} baud')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
