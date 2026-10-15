from pymavlink import mavutil
import threading

import aruco_landing
import inizializzazione

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

aruco_lander = aruco_landing.ArucoLanding(master, flag_simulazione=True)
inizializzatore = inizializzazione.Inizializzazione(master, altitudine_iniziale=10, coordinate_iniziali=(-13, -43, -20)) # la z è negativa perché il drone parte da un'altezza di 10 metri e quindi la posizione locale NED è -10

stop = threading.Event()

master.mav.command_long_send( # per leggere le informazioni di posizione globale (latitudine, longitudine, altitudine)
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # messaggio di posizione globale da leggere successivamente
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # messaggio di attitudine da leggere successivamente per avere roll, pitch e yaw
    100000,  # interval in microseconds (100000 = 10 Hz)
    0, 0, 0, 0, 0
)

def missione():
    try:
        inizializzatore.movimento_drone() 
        aruco_lander.avvicinamento_zona_stazione() # avvicinamento alla zona della stazione di atterraggio
        # qui poi: discesa, allineamento coi marker, LAND...
    except Exception as e:
        print("Errore nella missione:", e)
        stop.set()

#threading.Thread(target=missione, daemon=True).start()
aruco_lander.avvio_streaming(stop)   # bloccante, nel thread principale. avvio dello streaming video



