from pymavlink import mavutil
import threading
import time

import stream_video
import aruco_landing
import inizializzazione

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

stream = stream_video.StreamVideo()
aruco_lander = aruco_landing.ArucoLanding(master, stream, flag_simulazione=True)
inizializzatore = inizializzazione.Inizializzazione(master, altitudine_iniziale=20, coordinate_iniziali=(-13, -43, -20)) # la z è negativa perché il drone parte da un'altezza di 10 metri e quindi la posizione locale NED è -10

stop = threading.Event()

# Richiesta dei messaggi a 10 Hz (una chiamata per messaggio)
for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
               mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, 100000, 0, 0, 0, 0, 0)

def heartbeat_loop():
    while not stop.is_set():
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)

def missione():
    try:
        inizializzatore.movimento_drone()
        aruco_lander.avvicinamento_zona_stazione()
        aruco_lander.fase_1()
        aruco_lander.fase_2()
        aruco_lander.fase_3()
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        stop.set()          # chiude la finestra video a fine missione o in caso di errore

threading.Thread(target=heartbeat_loop, daemon=True).start()
threading.Thread(target=missione, daemon=True).start()
stream.avvio_streaming(stop)    # bloccante, nel thread principale (per via di imshow)
