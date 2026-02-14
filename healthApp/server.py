from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import eventlet
import random
import socket

def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

local_ip = get_ip()
app = Flask(__name__, static_folder='.', static_url_path='')
sio = SocketIO(app, cors_allowed_origins='*')

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    return send_from_directory('.', path)

def simulator():
    while True:
        # Simulation
        hr = random.randint(72, 76)
        br = random.randint(14, 16)
        
        sio.emit('update_data', {'hr': hr, 'br': br})
        eventlet.sleep(1)

if __name__ == '__main__':
    eventlet.spawn(simulator)
    
    print("\n" + "="*30)
    print(f"BIOSTAT HUD: ONLINE")
    print(f"URL: http://{local_ip}:5000")
    print("="*30 + "\n")
    
    sio.run(app, host='0.0.0.0', port=5000)