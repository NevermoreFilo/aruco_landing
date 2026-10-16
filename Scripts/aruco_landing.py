from doctest import master
import math
import threading
import cv2
import numpy as np
from pymavlink import mavutil
import time
import random

import utils

class ArucoLanding:
    def __init__(self, master, flag_simulazione):
        self.master = master
        self.flag_simulazione = flag_simulazione

        self.pipeline = ( # per lo stream video
            "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
            "! rtph264depay ! avdec_h264 ! videoconvert "
            "! appsink name=appsink sync=false drop=true max-buffers=1"
        )

        # Creazione del dizionario ArUco e dei parametri del rilevatore
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.marker_size_1 = 1  # metri
        self.marker_size_2 = 0.4  # metri
        self.marker_size_3 = 0.1  # metri


        # Costanti intrinseche del drone e della camera
        self.dist_coeffs = np.zeros((5, 1))  # Coefficienti di distorsione (essendo una telecamera simulata, non ci sono distorsioni)
        
        h1 = self.marker_size_1 / 2       
        h2 = self.marker_size_2 / 2
        h3 = self.marker_size_3 / 2

        self.marker_points_1 = np.array([[-h1,  h1, 0],  # alto sinistra
                                         [ h1,  h1, 0],  # alto destra ...
                                         [ h1, -h1, 0],
                                         [-h1, -h1, 0]], dtype=np.float32)
        self.marker_points_2 = np.array([[-h2,  h2, 0],  # alto sinistra
                                         [ h2,  h2, 0],  # alto destra ...
                                         [ h2, -h2, 0],
                                         [-h2, -h2, 0]], dtype=np.float32)
        self.marker_points_3 = np.array([[-h3,  h3, 0],  # alto sinistra
                                         [ h3,  h3, 0],  # alto destra ...
                                         [ h3, -h3, 0],
                                         [-h3, -h3, 0]], dtype=np.float32)
        # Di quanto si scosta il primo marker rispetto la stazione
        self.ox_dx = 0
        self.ox_av = 1.5

        self.coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
        self.coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}
        self.flag_atterraggio = False

        self.HFOV = 1.047 # letto dal .sdf



        # Altre costanti 

        # coordinate (grossolane) della zona della stazione di atterraggio (in metri)
        self.ZONA_STAZIONE = [4, 20, -15] # la z viene messa a 15 metri 
        # Nota: le coordinate in Gazebo usano il sistema di riferimento NED (North-East-Down), mentre MAVLink usa il sistema di riferimento ENU (East-North-Up)

        self.ID_ATTESI_1 = {0} # marker da riconoscere nella prima fase
        self.ID_ATTESI_2 = [1, 2, 3, 4] # marker da riconoscere nella seconda fase
        self.ID_ATTESI_3 = [5, 6, 7, 8] # marker da riconoscere nella terza fase

        self.KP_1 = 0.8 #m/s, proportional gain
        self.VMAX_1 = 0.5 # m/s
        self.ANGOLO_CALIBRAZIONE_YAW_1 = 0            # angolo tra due spigoli di un marker quando questo è allineato con la camera 
        self.TOL_YAW_1 = np.radians(10) # tolleranza errore nello yaw
        self.TOL_POS_1 = 0.5 # tolleranza nell'errore di x/y per la fase 1

        self.KP_2 = 0.5 #m/s
        self.VMAX_2 = 0.5 # m/s
        self.ANGOLO_CALIBRAZIONE_YAW_2 = 0  # angolo
        self.TOL_YAW_2 = np.radians(3) # tolleranza errore nello yaw
        self.TOL_POS_2 = 0.05 # tolleranza nell'errore di x/y per la fase 2

        self.ERRORE_ALLINEAMENTO_3 = 0.05 # m, soglia di allineamento

        self.KP_YAW = 0.5        # velocità di correzzione (1/s)
        self.YAW_RATE_MAX = 1        # rad/s 

        # Variabili globali

        self.lock = threading.Lock()    # lock per marker_individuati
        self.marker_individuati = {}    # {id: dict(tvec=..., err_yaw=..., t=...)}

        self.flag_atterraggio = False
        self.stato = {} # mantiene lo stato dei messaggi (viene usato poi per svuotare il buffer)


    def punti_per_id(self, mid):
        match mid:
            case 0:
                return self.marker_points_1
            case 1 | 2 | 3 | 4:
                return self.marker_points_2
            case 5 | 6 | 7 | 8:
                return self.marker_points_3
            case _: 
                return None 

    # Svuota il buffer, tenendo l'ultimo messaggio di ogni tipo
    def aggiorna_stato(self):
        while (m := self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE',
                                                'GLOBAL_POSITION_INT'], blocking=False)):
            self.stato[m.get_type()] = m

    def get_local_pos(self, timeout=2):
        t0 = time.time()
        while True:
            self.aggiorna_stato()
            m = self.stato.get('LOCAL_POSITION_NED')
            if m is not None:
                return m.x, m.y, m.z
            if time.time() - t0 > timeout:
                raise TimeoutError("Nessun LOCAL_POSITION_NED ricevuto")
            time.sleep(0.02)

    # Funzione per avvicinarsi alla generale zona della stazione di atterraggio
    def avvicinamento_zona_stazione(self): 
        print("Avvicinamento alla zona della stazione di atterraggio...")
        _,_,z = self.get_local_pos() # prelevo la quota attuale
        if z > self.ZONA_STAZIONE[2]: # se sono più basso della zona della stazione, rimango basso
            quota = z
        else:
            quota = self.ZONA_STAZIONE[2] # se sono più alto della zona della stazione, scendo a quota 15 metri
        print("Drone scarico, avvicinamento alla zona della stazione...")
        self.master.mav.set_position_target_local_ned_send(
        0,
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,              # posizione
        self.ZONA_STAZIONE[0],  self.ZONA_STAZIONE[1],  quota,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
        
        # Attesa fino a quando la zona target è (quasi raggiunta)
        last_hb = 0
        alt = 0.0
        t0 = time.time()

        while True:
            x,y,z = self.get_local_pos() # prelevo la quota attuale
            dist = math.sqrt((x-self.ZONA_STAZIONE[0])**2 + (y-self.ZONA_STAZIONE[1])**2 + (z-quota)**2)
            if dist < 0.5:
                print("Zona stazione raggiunta")
                break
            time.sleep(0.1) # per evitare di saturare la CPU con un loop troppo veloce


    def avvio_streaming(self, stop_event):
        cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("Errore: impossibile aprire lo stream video")
            exit(1)
        last_hb = 0
        _, frame = cap.read()
        # Faccio questi calcoli una sola volta, non ad ogni frame
        W, H = frame.shape[1], frame.shape[0]      # Larghezza ed altezza della telecamera  
                                                                        #                                            [fx  0  cx]
        fx = (W / 2) / np.tan(self.HFOV / 2)            # formula per lunghezza focale                               [0  fy  cy]
        camera_matrix = np.array([[fx, 0, W / 2],  # matrice della camera con la forma standard     ->               [0   0   1]
                                [0, fx, H / 2],
                                [0,  0,   1]], dtype=np.float64)
        dist_coeffs = np.zeros((5, 1))             # in gazebo la camera è ideale quindi non c'è distorsione

        while not stop_event.is_set():
            if time.time() - last_hb >= 1:
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = time.time()

            ret, frame = cap.read()
            if not ret:
                    print("NO frame")
                    continue
            corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.detector_params
            )
            marker_frame_attuale = {}
            if ids is not None:
                for corner, marker_id in zip(corners, ids):
                    mid = int(marker_id[0]) # converto l'id del marker in intero
                    cv2.polylines(frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)
                    pts = self.punti_per_id(mid) # 
                    if pts is None:
                        continue
                    ok, rvec, tvec = cv2.solvePnP(pts, corner[0], camera_matrix, dist_coeffs,
                                                flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        continue
                    dx, dy = corner[0][1] - corner[0][0]
                    marker_frame_attuale[mid] = dict(tvec=tvec.flatten(),
                                    angolo=np.arctan2(dy, dx),
                                    t=time.time())
            with self.lock:
                self.marker_individuati = marker_frame_attuale              # sostituisce: se non vede nulla, resta vuoto
            
            cv2.imshow("Drone Camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

        # def set_param(nome, valore):
        #     master.mav.param_set_send(
        #         self.master.target_system, self.master.target_component,
        #         nome.encode(), valore, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # def set_gps_bias(err_n_m, err_e_m, lat_deg):
        #     dlat = err_n_m / 111_000.0
        #     dlon = err_e_m / (111_000.0 * math.cos(math.radians(lat_deg)))
        #     set_param('SIM_GPS1_GLTCH_X', dlat)   # o SIM_GPS_GLITCH_X nelle versioni vecchie
        #     set_param('SIM_GPS1_GLTCH_Y', dlon)


# Funzione che riposiziona il drone (x/y) 
    def allinea_drone_singolo_marker(self, vx, vy, yaw_rate):
        self.master.mav.set_position_target_local_ned_send(
        0, self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,          # velocità + yaw_rate
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, yaw_rate)                 # yaw ignorato, yaw_rate usato

        


# Funzione ausiliaria che porta il drone ad una determinata quota, mantenendolo fermo sulle altre coordinate
    def discesa_drone(self, quota):
        x, y, _ = self.get_local_pos()
        yaw = self.stato['ATTITUDE'].yaw     
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b100111111000,          # posizione + yaw (yaw_rate ignorato)
            x, y, quota,
            0, 0, 0,
            0, 0, 0,
            yaw, 0)
        
    def attendi_quota(self, quota, tol=0.15, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.aggiorna_stato()
            m = self.stato.get('LOCAL_POSITION_NED')
            if m is not None and abs(m.z - quota) < tol:
                print("Altitudine target raggiunta")
                return
            time.sleep(0.1)
        print("Timeout: quota non raggiunta")

# Funzione per la gestione della fase 1: allineamento grossolano e prima discesa di quota
    def fase_1(self):
        while True:
            with self.lock:
                d = self.marker_individuati.get(0)
            if d is None or time.time() - d['t'] > 0.5:
                self.allinea_drone_singolo_marker(0,0,0)             # marker perso: fermo
                time.sleep(0.1)
                continue

            print("Allineamento fase 1")
            err_yaw = utils.wrap(d['angolo'] - self.ANGOLO_CALIBRAZIONE_YAW_1)                         
            tvec = d['tvec']
            # offset nel sistema marker: (ox_dx, ox_av) = (destra, avanti)
            offset_destra = self.ox_dx
            offset_avanti = self.ox_av
            err_destra = tvec[0] - offset_destra
            err_avanti = -tvec[1] - offset_avanti

            yaw_ok = abs(err_yaw) < self.TOL_YAW_1
            # se tutti i parametri sono nella soglia di tolleranza, sono allineato
            if abs (err_avanti) < self.TOL_POS_1 and abs(err_destra) < self.TOL_POS_1 and yaw_ok:
                print("Primo allineamento completato, discesa...")
                self.discesa_drone(-5)                    # scendo a 5m per la fase 2
                self.attendi_quota(-5)
                print("Fase 1 completata")
                return
            else:
                vx = np.clip(self.KP_1 * err_avanti, -self.VMAX_1, self.VMAX_1)
                vy = np.clip(self.KP_1 * err_destra, -self.VMAX_1, self.VMAX_1)
                yaw_rate = np.clip(self.KP_YAW * err_yaw, -self.YAW_RATE_MAX, self.YAW_RATE_MAX)
                # se la posizione è giusta ma lo yaw no, allineo e basta
                if not yaw_ok:
                    vx = vy = 0
                else:
                    yaw_rate = 0
            self.allinea_drone_singolo_marker(vx,vy,yaw_rate)
            self.aggiorna_stato()
            time.sleep(0.1)                       # 10 Hz

    # Funzione per la gestione della fase 2: allineamento più preciso ed ultima discesa di quota prima dell'atterraggio
    def fase_2(self):
        ids = self.ID_ATTESI_2
        while True:
            with self.lock:
                snap = dict(self.marker_individuati)
            ora = time.time()
            if not all(i in snap and ora - snap[i]['t'] < 0.5 for i in ids):
                self.allinea_drone_singolo_marker(0,0,0)             # marker perso: fermo
                time.sleep(0.1)
                continue

            p = {i: np.array(snap[i]['tvec']) for i in ids}   
            cx, cy, cz = np.mean([p[i] for i in ids], axis=0) # centro stimato
            print("Allineamento fase 2")
            # Coppia parallela
            v = (p[1] - p[3]) + (p[2] - p[4])

            err_destra = cx
            err_avanti = -cy
            err_yaw = utils.wrap(np.arctan2(v[1], v[0]) - self.ANGOLO_CALIBRAZIONE_YAW_2)                        

            # se tutti i parametri sono nella soglia di tolleranza, sono allineato
            if abs (err_avanti) < self.TOL_POS_2 and abs(err_destra) < self.TOL_POS_2 and abs(err_yaw) < self.TOL_YAW_2:
                print("Secondo allineamento completato, discesa...")
                self.discesa_drone(-0.8)                    # scendo a 0.8m per la fase 3
                self.attendi_quota(-0.8)
                print("Fase 2 completata")
                return
            else :
                vx = np.clip(self.KP_2 * err_avanti, -self.VMAX_2, self.VMAX_2)
                vy = np.clip(self.KP_2 * err_destra, -self.VMAX_2, self.VMAX_2)
                yaw_rate = np.clip(self.KP_YAW * err_yaw, -self.YAW_RATE_MAX, self.YAW_RATE_MAX)
                # se la posizione è giusta ma lo yaw no, allineo e basta
                if abs(err_avanti) < self.TOL_POS_2 and abs(err_destra) < self.TOL_POS_2:
                    vx = vy = 0
                # Viceversa
                if abs(err_yaw) < self.TOL_YAW_2:
                    yaw_rate = 0
            self.allinea_drone_singolo_marker(vx,vy,yaw_rate)
            self.aggiorna_stato()
            time.sleep(0.1)                       # 10 Hz

