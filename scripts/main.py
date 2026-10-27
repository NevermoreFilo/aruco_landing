from pymavlink import mavutil
import threading
import time
import argparse
import random
import traceback
import os
import csv

import stream_video
import aruco_landing
import inizializzazione

COORDINATE_STAZIONE = [7, 15] # letto da iris_runway.sdf
LAT_ERR =-35.36326217651367 # letto da MAVLink con comando param show SIM_OPOS_LAT


argomenti = argparse.ArgumentParser()
argomenti.add_argument("--sigma", type=float, default=0.0) # usato per errore gps (m)
argomenti.add_argument("--seed", type=int, default=0) # seed per generazione randomica
argomenti.add_argument("--output", default="risultati.csv")
args=argomenti.parse_args()
random.seed(args.seed)

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

risultato = {} # dizionario in cui mantenere una serie di informazioni sull'esecuzione del codice
print("MAIN - Attesa heartbeat")
master.wait_heartbeat()
print("MAIN - Heartbeat ricevuto")
stream = stream_video.StreamVideo()
aruco_lander = aruco_landing.ArucoLanding(master, stream, COORDINATE_STAZIONE)
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
    print("Inizio missione")
    try:
        while not stream.flag_stream_attivo:
            time.sleep(0.5)
        err_nord = random.gauss(0, args.sigma)
        err_est = random.gauss(0, args.sigma)

        inizializzatore.avvio_drone()
        aruco_lander.set_origine_coordinate()
        aruco_lander.set_gps_bias(err_nord, err_est, LAT_ERR)
        time.sleep(2)
        inizializzatore.decollo_drone()
        aruco_lander.avvicinamento_zona_stazione()
        aruco_lander.fase_1()
        aruco_lander.fase_2()
        risultato.update(aruco_lander.fase_3())

    except RuntimeError:
        risultato["esito"] = None
    except Exception:
        traceback.print_exc()
        risultato["esito"] = "fallito"
    finally:
        stop.set()

threading.Thread(target=heartbeat_loop, daemon=True).start()
threading.Thread(target=missione, daemon=True).start()
stream.avvio_streaming(stop)    # bloccante, nel thread principale (per via di imshow)

nuovo = not os.path.exists(args.output)
with open(args.output, "a", newline="",) as f:
    writer = csv.DictWriter(f, fieldnames=["sigma", "seed", "esito", "errore", "lat", "long", "yaw"]) # testa del file csv
    if nuovo:
        writer.writeheader() # se il file è nuovo scrivo la testa, altrimenti no
    if risultato.get("esito")  == "fallito":
        writer.writerow({"sigma":args.sigma, "seed":args.seed, "esito":"fallito", "errore":"N/A", "lat":"N/A", "long":"N/A", "yaw":"N/A"}) #scrivo i risultati registrati nel dizionario, nel file csv 
    elif risultato.get("esito") == None:
        writer.writerow({"sigma":args.sigma, "seed":args.seed, "esito":"Errore nel lancio", "errore":"N/A", "lat":"N/A", "long":"N/A", "yaw":"N/A"}) #scrivo i risultati registrati nel dizionario, nel file csv 
    else:
        writer.writerow({"sigma":args.sigma, "seed":args.seed, "esito":"ok", "errore":risultato["errore"], "lat":risultato["lat"], "long":risultato["long"], "yaw":risultato["yaw"]}) #scrivo i risultati registrati nel dizionario, nel file csv 


master.close()