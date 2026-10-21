from doctest import master
import math
import threading
import numpy as np
from pymavlink import mavutil
import time
import random

import utils

class ArucoLanding:
    def __init__(self, master, stream, stazione):
        self.master = master
        self.stream = stream


        # Di quanto si scosta il primo marker rispetto la stazione
        self.ox_dx = 0
        self.ox_av = 1.5

        self.HFOV = 1.047 # letto dal .sdf

        # Altre costanti 

        #self.COORDINATE_ESATTE_STAZIONE = [7, 15] # Mantengo le esatte coordinate della stazione per confrontarle con quelle ottenute all'atterraggio
        # coordinate (grossolane) della zona della stazione di atterraggio (in metri)
        #self.ZONA_STAZIONE = [random.randrange(1, 15, 1), random.randrange(5, 20), -15] # la z viene messa a 15 metri 
        # Nota: le coordinate in Gazebo usano il sistema di riferimento NED (North-East-Down), mentre MAVLink usa il sistema di riferimento ENU (East-North-Up)
        
        self.ZONA_STAZIONE = [stazione[0], stazione[1], -15]

        self.COORDINATE_ESATTE_STAZIONE = stazione


        self.ID_ATTESI_1 = {0} # marker da riconoscere nella prima fase
        self.ID_ATTESI_2 = [1, 2, 3, 4] # marker da riconoscere nella seconda fase
        self.ID_ATTESI_3 = [5, 6, 7, 8] # marker da riconoscere nella terza fase

        self.KP_1 = 0.6 #m/s, proportional gain
        self.VMAX_1 = 0.8 # m/s
        self.ANGOLO_CALIBRAZIONE_YAW_1 = 0            # angolo tra due spigoli di un marker quando questo è allineato con la camera 
        self.TOL_YAW_1 = np.radians(10) # tolleranza errore nello yaw
        self.TOL_POS_1 = 0.5 # tolleranza nell'errore di x/y per la fase 1

        self.KP_2 = 0.4 #m/s
        self.VMAX_2 = 0.4 # m/s
        self.ANGOLO_CALIBRAZIONE_YAW_2 = 0  # angolo
        self.TOL_YAW_2 = np.radians(4) # tolleranza errore nello yaw
        self.TOL_POS_2 = 0.1 # tolleranza nell'errore di x/y per la fase 2

        self.KP_3 = 0.3 #m/s
        self.VMAX_3 = 0.3 # m/s
        self.ANGOLO_CALIBRAZIONE_YAW_3 = 0  # angolo
        self.TOL_YAW_3 = np.radians(2) # tolleranza errore nello yaw
        self.TOL_POS_3 = 0.05 # tolleranza nell'errore di x/y per la fase 3

        self.KP_YAW = 0.5        # velocità di correzzione (1/s)
        self.YAW_RATE_MAX = 0.8        # rad/s 

        # Variabili globali

        self.lock = threading.Lock()    # lock per marker_individuati
        self.marker_individuati = {}    # {id: dict(tvec=..., err_yaw=..., t=...)}

        self.stato = {} # mantiene lo stato dei messaggi (viene usato poi per svuotare il buffer)


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


    def set_gps_bias(self, err_n_m, err_e_m, lat_deg):
        dlat = err_n_m / 111_000.0
        dlon = err_e_m / (111_000.0 * math.cos(math.radians(lat_deg)))
        self.set_param('SIM_GPS1_GLTCH_X', dlat)   # o SIM_GPS_GLITCH_X nelle versioni vecchie
        self.set_param('SIM_GPS1_GLTCH_Y', dlon)


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

# Funzione ausiliaria che imposta il drone in modalità di atterraggio e attende che i motori siano disarmati
    def atterraggio_drone(self, timeout=60):
        self.master.set_mode('LAND')
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and hb.get_srcSystem() == self.master.target_system:
                armato = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not armato:
                    print("Drone disarmato, atterraggio completato")
                    return True
        print("Timeout: il drone non si è disarmato")
        return False

    # Funzione per impostare un certo parametro su ARDUPILOT (usato per cambiare la velocità di discesa)

    def set_param(self, nome, valore):
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            nome.encode(), float(valore), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

# Funzione per la gestione della fase 1: allineamento grossolano e prima discesa di quota
    def fase_1(self):
        print("Allineamento fase 1")
        t0 = time.time()
        while True:
            d = self.stream.snapshot().get(0)
            if d is None or time.time() - d['t'] > 0.5:
                self.allinea_drone_singolo_marker(0,0,0)             # marker perso: fermo
                time.sleep(0.1)
                continue

            err_yaw = utils.wrap(d['angolo'] - self.ANGOLO_CALIBRAZIONE_YAW_1)                         
            tvec = d['tvec']
            # offset nel sistema marker: (ox_dx, ox_av) = (destra, avanti)
            offset_destra = self.ox_dx
            offset_avanti = self.ox_av
            err_destra = tvec[0] - offset_destra
            err_avanti = -tvec[1] - offset_avanti

            x_ok = abs (err_avanti) < self.TOL_POS_1
            y_ok = abs(err_destra) < self.TOL_POS_1
            yaw_ok = abs(err_yaw) < self.TOL_YAW_1
            # se tutti i parametri sono nella soglia di tolleranza, sono allineato
            if x_ok and y_ok and yaw_ok:
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
        self.set_param('LAND_SPEED', 50)   # cm/s
        ids = self.ID_ATTESI_2
        print("Allineamento fase 2")
        while True:
            snap = self.stream.snapshot()
            ora = time.time()
            if not all(i in snap and ora - snap[i]['t'] < 0.5 for i in ids):
                self.allinea_drone_singolo_marker(0,0,0)             # marker perso: fermo
                time.sleep(0.1)
                continue

            p = {i: np.array(snap[i]['tvec']) for i in ids}   
            cx, cy, cz = np.mean([p[i] for i in ids], axis=0) # centro stimato
            # Coppia parallela
            v = (p[1] - p[3]) + (p[2] - p[4])

            err_destra = cx
            err_avanti = -cy
            err_yaw = utils.wrap(np.arctan2(v[1], v[0]) - self.ANGOLO_CALIBRAZIONE_YAW_2)                        

            # se tutti i parametri sono nella soglia di tolleranza, sono allineato
            if abs (err_avanti) < self.TOL_POS_2 and abs(err_destra) < self.TOL_POS_2 and abs(err_yaw) < self.TOL_YAW_2:
                print("Secondo allineamento completato, discesa...")
                self.discesa_drone(-0.7)                    # scendo a 0.7m per la fase 3
                self.attendi_quota(-0.7)
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


    # Funzione per la gestione della fase 3: allineamento finale ed atterraggio
    def fase_3(self):
        self.set_param('LAND_SPEED', 20)   # cm/s
        time.sleep(2)
        ids = self.ID_ATTESI_3
        print("Allineamento fase 3")
        while True:
            snap = self.stream.snapshot()
            ora = time.time()
            if not all(i in snap and ora - snap[i]['t'] < 0.5 for i in ids):
                self.allinea_drone_singolo_marker(0,0,0)             # marker perso: fermo
                time.sleep(0.1)
                continue

            p = {i: np.array(snap[i]['tvec']) for i in ids}   
            cx, cy, cz = np.mean([p[i] for i in ids], axis=0) # centro stimato
            # Coppia parallela
            v = (p[5] - p[7]) + (p[6] - p[8])
            err_destra = cx
            err_avanti = -cy
            err_yaw = utils.wrap(np.arctan2(v[1], v[0]) - self.ANGOLO_CALIBRAZIONE_YAW_3)                        

            # se tutti i parametri sono nella soglia di tolleranza, sono allineato
            if abs (err_avanti) < self.TOL_POS_3 and abs(err_destra) < self.TOL_POS_3 and abs(err_yaw) < self.TOL_YAW_3:
                print("Terzo allineamento completato, atterraggio...")
                atterrato = self.atterraggio_drone()
                if (not atterrato):
                    return(dict(esito="fallito"))
                x,y = utils.posizione_reale_gazebo()
                yaw = self.stato['ATTITUDE'].yaw
                errore = math.hypot(x-self.COORDINATE_ESATTE_STAZIONE[1], y-self.COORDINATE_ESATTE_STAZIONE[0])
                print("Atterraggio avvenuto alle coordinate x: ", x, " y: ", y, " e con yaw: ", yaw)
                print("Errore nell'atterraggio: ", errore)
                print("Fase 3 completata")
                return dict(esito="ok", errore=errore, x=x, y=y, yaw=yaw)
            else :
                vx = np.clip(self.KP_3 * err_avanti, -self.VMAX_3, self.VMAX_3)
                vy = np.clip(self.KP_3 * err_destra, -self.VMAX_3, self.VMAX_3)
                yaw_rate = np.clip(self.KP_YAW * err_yaw, -self.YAW_RATE_MAX, self.YAW_RATE_MAX)
                # se la posizione è giusta ma lo yaw no, allineo e basta
                if abs(err_avanti) < self.TOL_POS_3 and abs(err_destra) < self.TOL_POS_3:
                    vx = vy = 0
                # Viceversa
                if abs(err_yaw) < self.TOL_YAW_3:
                    yaw_rate = 0
            self.allinea_drone_singolo_marker(vx,vy,yaw_rate)
            self.aggiorna_stato()
            time.sleep(0.1)                       # 10 Hz

