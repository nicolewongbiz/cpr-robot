import serial
import socketio
import eventlet
import time
from flask import Flask, send_from_directory

# Initialize Socket.io and Flask
sio = socketio.Server(cors_allowed_origins='*')
app = Flask(__name__)

# --- CONFIGURATION ---
# Ensure this matches the port in your Arduino IDE (Tools > Port)
SERIAL_PORT = '/dev/cu.usbmodem101' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    # Give the Metro Mini time to reboot after opening the port
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"✅ Arduino connected on {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"⚠️ Serial Connection Error: {e}")

# --- ROUTES ---

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/models/<path:filename>')
def serve_models(filename):
    return send_from_directory('models', filename)

def read_arduino():
    print("Loop started: Waiting for Arduino data...")
    while True:
        if ser and ser.is_open:
            try:
                raw_data = ser.readline()
                if raw_data:
                    line = raw_data.decode('utf-8', errors='ignore').strip()
                    
                    # Splitting the 3 values: hr, ir, beat
                    if "," in line:
                        parts = line.split(',')
                        if len(parts) >= 3:
                            hr = int(parts[0])
                            ir = int(parts[1])
                            beat = int(parts[2]) # This is the 1 or 0 pulse signal

                            # Debugging: You'll see "BEAT!" in your terminal when a pulse happens
                            pulse_msg = " <--- BEAT!" if beat == 1 else ""
                            print(f"BPM: {hr} | IR: {ir} | Pulse: {beat}{pulse_msg}")

                            sio.emit('update_data', {
                                'hr': hr,
                                'ir': ir,
                                'beat': beat
                            })
            except Exception as e:
                print(f"Read Error: {e}")
        eventlet.sleep(0.01)

if __name__ == '__main__':
    app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)
    eventlet.spawn(read_arduino)
    
    print("🚀 Server starting at http://localhost:5000")
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 5000)), app)