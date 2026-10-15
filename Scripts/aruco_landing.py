from doctest import master
import math
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
        self.coordinate_marker_1 = []   # coordinate [x, y, z] del marker 0
        self.coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
        self.coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}
        self.flag_atterraggio = False

        self.HFOV = 1.047 # letto dal .sdf



        # Altre costanti 

        # coordinate (grossolane) della zona della stazione di atterraggio (in metri)
        self.ZONA_STAZIONE = [4, 20, -15] # la z viene messa a 5 metri 
        # Nota: le coordinate in Gazebo usano il sistema di riferimento NED (North-East-Down), mentre MAVLink usa il sistema di riferimento ENU (East-North-Up)

        self.ID_ATTESI_1 = {0} # marker da riconoscere nella terza fase
        self.ID_ATTESI_2 = {2, 3, 4, 5} # marker da riconoscere nella terza fase
        self.ID_ATTESI_3 = {6, 7, 8, 9} # marker da riconoscere nella terza fase

        self.KP_YAW = 0.5        # velocità di correzzione (1/s)
        self.YAW_RATE_MAX = 0.5 # rad/s (circa 30°/s)

        self.ERRORE_ALLINEAMENTO_3 = 0.05 # m, soglia di allineamento
        self.ANGOLO_CALIBRAZIONE_YAW = 2*np.pi            # angolo tra due spigoli di un marker quando questo è allineato con la camera 
        self.TOL_YAW = np.radians(1)


        # Variabili globali

        self.coordinate_marker_1 = []   # coordinate globali conosciute per dare una prima direzione
        self.coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
        self.coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}

        self.flag_atterraggio = False
        self.flag_calibrazione = False
        

    def get_local_pos(self,timeout=2):
        m = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
        if m is None:
            raise TimeoutError("Nessun LOCAL_POSITION_NED ricevuto")
        return m.x, m.y, m.z


    # Funzione per avvicinarsi alla generale zona della stazione di atterraggio
    def avvicinamento_zona_stazione(self): # @TODO: mettere un controllo che si sia arrivati in zona e switchare il flag
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
            if ids is not None and self.flag_calibrazione:
                for corner, marker_id in zip(corners, ids):
                    mid = int(marker_id[0]) # converto l'id del marker in intero
                    cv2.polylines(frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)

                    if mid in self.ID_ATTESI_1:
                        ok, rvec, tvec = cv2.solvePnP(self.marker_points_1, corner[0], camera_matrix,
                                                    dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        print("Debug")
                        x, y, z = tvec.flatten() # coordinate del marker rispetto alla camera, in metri
                        self.coordinate_marker_1 = [x, y, z] 

                        dx, dy = corner[0][1] - corner[0][0] 
                        self.err_yaw_1 = utils.wrap(np.arctan2(dy, dx) - self.ANGOLO_CALIBRAZIONE_YAW)
                        # Calcolo della velocità di movimento sull'asse yaw
                        yaw_rate = np.clip(self.KP_YAW * self.err_yaw_1, -self.YAW_RATE_MAX, self.YAW_RATE_MAX) 

                        if abs(self.err_yaw_1) < self.TOL_YAW:
                            print("Allineato, discesa...")
                            # @TODO: far scendere di quota (nell'intorno dei 5/8 metri) e passare alla seconda fase di allineamento
                        else:
                            self.allinea_drone(yaw_rate)

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


    def allinea_drone(self, yaw_rate):
        self.master.mav.set_position_target_local_ned_send(
        0, self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,          # velocità + yaw_rate
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, yaw_rate)                 # yaw ignorato, yaw_rate usato