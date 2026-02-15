import serial
import socketio
import eventlet
import time
import os
from flask import Flask, send_from_directory
from dotenv import load_dotenv
from twilio.rest import Client

# Load variables from .env file
load_dotenv()

sio = socketio.Server(cors_allowed_origins='*')
app = Flask(__name__)

# --- CONFIGURATION ---
SERIAL_PORT = os.getenv('SERIAL_PORT', '/dev/cu.usbmodem1101') 
BAUD_RATE = 115200

# Twilio Setup
TWILIO_SID = os.getenv('TWILIO_ACCOUNT_SID')
TWILIO_TOKEN = os.getenv('TWILIO_AUTH_TOKEN')
TWILIO_FROM = os.getenv('TWILIO_PHONE_NUMBER')
TWILIO_TO = os.getenv('TARGET_PHONE_NUMBER')

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"✅ Arduino connected on {SERIAL_PORT}")
except Exception as e:
    ser = None
    print(f"⚠️ Serial Connection Error: {e}")

# --- TWILIO VOICE HELPER ---
def make_emergency_call(bpm_value):
    """Triggers a phone call with a text-to-speech message."""
    try:
        if all([TWILIO_SID, TWILIO_TOKEN, TWILIO_FROM, TWILIO_TO]):
            client = Client(TWILIO_SID, TWILIO_TOKEN)
            
            # TwiML defines what the robot says when you answer
            twiml_content = f'''
            <Response>
                <Say voice="alice">
                    Attention. This is an automated emergency alert from your heart monitor. 
                    A flatline has been detected for fifteen seconds. 
                    The last recorded heart rate was {bpm_value} beats per minute. 
                    Please check the patient immediately.
                </Say>
            </Response>
            '''
            
            call = client.calls.create(
                twiml=twiml_content,
                from_=TWILIO_FROM,
                to=TWILIO_TO
            )
            print(f"📞 Twilio Call Placed: {call.sid}")
        else:
            print("⚠️ Twilio credentials missing in .env")
    except Exception as e:
        print(f"❌ Twilio Call Failed: {e}")

# --- ROUTES ---
@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/models/<path:filename>')
def serve_models(filename):
    return send_from_directory('models', filename)

def read_arduino():
    print("Loop started: Waiting for Arduino data...")
    zero_hr_start_time = None
    emergency_triggered = False

    while True:
        if ser and ser.is_open:
            try:
                raw_data = ser.readline()
                if raw_data:
                    line = raw_data.decode('utf-8', errors='ignore').strip()
                    if "," in line:
                        parts = line.split(',')
                        if len(parts) >= 3:
                            hr = int(parts[0])
                            ir = int(parts[1])
                            beat = int(parts[2])
                            
                            # Local Dashboard Update
                            sio.emit('update_data', {'hr': hr, 'ir': ir, 'beat': beat})

                            # --- EMERGENCY LOGIC ---
                            if hr == 0:
                                if zero_hr_start_time is None:
                                    zero_hr_start_time = time.time()
                                
                                elapsed = time.time() - zero_hr_start_time
                                if elapsed >= 15 and not emergency_triggered:
                                    print("🚨 ALERT: 15s of 0 BPM. Calling Emergency Contact...")
                                    sio.emit('emergency_alert', {'active': True})
                                    emergency_triggered = True
                                    # Trigger the Voice Call
                                    make_emergency_call(hr)
                            else:
                                # Heartbeat detected - reset everything
                                if emergency_triggered:
                                    sio.emit('emergency_alert', {'active': False})
                                    emergency_triggered = False
                                zero_hr_start_time = None

            except Exception as e:
                print(f"Read Error: {e}")
        eventlet.sleep(0.01)

if __name__ == '__main__':
    app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)
    eventlet.spawn(read_arduino)
    print("🚀 Server starting at http://localhost:5000")
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 5000)), app)