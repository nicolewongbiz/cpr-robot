import socketio
import eventlet
import eventlet.wsgi
from flask import Flask, send_from_directory
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
sio = socketio.Server(cors_allowed_origins='*')
app = Flask(__name__)

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

def simulator():
    while True:
        # Simulation
        hr = random.randint(72, 76)
        br = random.randint(14, 16)
        
        sio.emit('update_data', {'hr': hr, 'br': br})
        eventlet.sleep(1)

if __name__ == '__main__':
    app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)
    eventlet.spawn(simulator)
    
    print("\n" + "="*30)
    print(f"BIOSTAT HUD: ONLINE")
    print(f"URL: http://{local_ip}:5000")
    print("="*30 + "\n")
    
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 5000)), app)