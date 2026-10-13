import cv2
import numpy as np
from pymavlink import mavutil
import time

import cv2
import numpy as np
from pymavlink import mavutil
import time

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters_create()

TARGET_ALT = 2  # meters above the takeoff point
TARGET_COORDS = (130.3, -20.2, -10.0)

start_lat = None
start_lon = None
start_yaw = None

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# preso dal comando che invio da terminale per avviare lo streaming:
# gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! ximagesink sync=false
# unica differenza è che al posto di ximagesink sync=false ho messo appsink per poterlo leggere con OpenCV invece che aprire una finestra
pipeline = (
    "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
    "! rtph264depay ! avdec_h264 ! videoconvert "
    "! appsink name=appsink sync=false drop=true max-buffers=1"
)

master.mav.command_long_send( # per leggere le informazioni di posizione globale (latitudine, longitudine, altitudine)
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # messaggio di posizione globale da leggere successivamente
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # messaggio di attitudine da leggere successivamente per avere roll, pitch e yaw
    100000,  # interval in microseconds (100000 = 10 Hz)
    0, 0, 0, 0, 0
)

print("In attesa della posizione di partenza...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2.0)
if msg:
    start_lat = msg.lat / 1e7
    start_lon = msg.lon / 1e7
    print(f"Posizione di partenza ricevuta: lat={start_lat}, lon={start_lon}")

print("In attesa dell'angolo di partenza...")
msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2.0)
if msg:
    start_yaw = msg.yaw
    print(f"Angolo di partenza ricevuto: yaw={msg.yaw}") # ci interessa solo lo yaw, visto che pitch e roll saranno 0 quando il drone è a terra

# 1. Switch to GUIDED mode
mode_id = master.mode_mapping()['GUIDED']
master.set_mode('GUIDED')
t0 = time.time()
while master.flightmode != 'GUIDED':
    master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if time.time() - t0 > 10:
        raise RuntimeError("Could not switch to GUIDED")

print("Switched to GUIDED mode")

# 2. Arm and wait until the motors are actually armed
master.arducopter_arm()
t0 = time.time()
while not master.motors_armed():
    m = master.recv_match(type=['HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK'],
                          blocking=True, timeout=1)
    if m is not None:
        if m.get_type() == 'STATUSTEXT':
            print("Autopilot:", m.text)
        elif m.get_type() == 'COMMAND_ACK':
            print("ACK:", m)
    if time.time() - t0 > 30:
        raise RuntimeError("Arming failed or timed out")
print("Armed")
# 3. Take off
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,                # confirmation
    0, 0, 0, 0,       # param1-4: pitch, empty, empty, yaw (unused for copters)
    0, 0,             # param5-6: lat, lon (0 = current position)
    TARGET_ALT        # param7: altitude in meters
)

# 4. Wait until the target altitude is (almost) reached
last_hb = 0
alt = 0.0
t0 = time.time()

while True:
    # heartbeat once per second
    if time.time() - last_hb >= 1:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = time.time()

    # read all pending messages and keep the latest values
    while (m := master.recv_match(blocking=False)):
        t = m.get_type()
        if t == 'GLOBAL_POSITION_INT':
            alt = m.relative_alt / 1000
            last_pos = (m.lat / 1e7, m.lon / 1e7, alt)
        elif t == 'STATUSTEXT':
            print("Autopilot:", m.text)

    print(f"Altitude: {alt:.1f} m")
    if alt >= TARGET_ALT * 0.95:
        print("Target altitude reached")
        break

    if time.time() - t0 > 60:
        raise RuntimeError("Takeoff timeout")

    time.sleep(0.1)  # avoid spinning the CPU at 100%
ARRIVAL_RADIUS = 1.0     # meters: how close counts as "reached"
GOTO_TIMEOUT = 120       # seconds: give up if it never arrives

# Make sure LOCAL_POSITION_NED is streamed at 10 Hz
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    100000, 0, 0, 0, 0, 0
)

def send_target():
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,              # position + yaw
        TARGET_COORDS[0], TARGET_COORDS[1], TARGET_COORDS[2],
        0, 0, 0,
        0, 0, 0,
        20, 0
    )

# 5. Go to the target and wait until it is actually reached
send_target()
t0 = time.time()
last_hb = 0
last_send = time.time()
reached = False

while time.time() - t0 < GOTO_TIMEOUT:
    # keep the GCS heartbeat alive
    if time.time() - last_hb >= 1:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = time.time()

    # resend the setpoint every second (safe in GUIDED, avoids timeouts)
    if time.time() - last_send >= 1:
        send_target()
        last_send = time.time()

    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg is None:
        print("No position message, waiting...")
        continue

    dist = ((msg.x - TARGET_COORDS[0])**2 +
            (msg.y - TARGET_COORDS[1])**2 +
            (msg.z - TARGET_COORDS[2])**2) ** 0.5
    print(f"Distance to target: {dist:.1f} m")

    if dist <= ARRIVAL_RADIUS:
        reached = True
        print("Target reached")
        break

if not reached:
    print("Target NOT reached within timeout; returning to launch anyway for safety")

# 6. Return to launch (only after arrival, or on timeout as a failsafe)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    0,
    0, 0, 0, 0, 0, 0, 0
)

#master.set_mode('LAND')
master.mav.command_long_send(
    master.target_system,       # ID Sistema Target
    master.target_component,    # ID Componente Target
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, # Comando
    0,                              # Conferma
    0, 0, 0, 0, 0, 0, 0             # Parametri da 1 a 7 (non usati per RTL)
)

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    msg2 = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg and msg2:
        alt = msg.relative_alt / 1000
        print(f"Altitude: {alt:.1f} m")
        if alt == 0:
            print("Atterraggio completato, partito a coordinate: ", start_lat, start_lon, "e terminato a coordinate:", msg.lat / 1e7, msg.lon / 1e7)
            yaw = msg2.yaw
            print(f"Yaw di partenza: {start_yaw:.2f} rad, yaw di arrivo {yaw:.2f} rad")
            break
    else:
        print("No message received, waiting...")

cv2.destroyAllWindows()