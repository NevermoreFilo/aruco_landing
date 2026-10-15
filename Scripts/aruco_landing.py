import cv2
import numpy as np
from pymavlink import mavutil
import time

class ArucoLanding:
    def __init__(self, master):
        self.master = master

        self.pipeline = ( # per lo stream video
            "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
            "! rtph264depay ! avdec_h264 ! videoconvert "
            "! appsink name=appsink sync=false drop=true max-buffers=1"
        )

        # Creazione del dizionario ArUco e dei parametri del rilevatore
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.marker_size_3 = 0.05  # metri
        self.marker_size_3 = 0.05  # metri
        self.marker_size_3 = 0.05  # metri


        # Costanti intrinseche del drone e della camera
        self.dist_coeffs = np.zeros((5, 1))  # Coefficienti di distorsione (essendo una telecamera simulata, non ci sono distorsioni)
        h3 = self.marker_size_3 / 2
        self.marker_points_3 = np.array([[-h3,  h3, 0],  # alto sinistra
                                         [ h3,  h3, 0],  # alto destra ...
                                         [ h3, -h3, 0],
                                         [-h3, -h3, 0]], dtype=np.float32)
        self.coordinate_marker_1 = []   # coordinate globali conosciute per dare una prima direzione
        self.coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
        self.coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}
        self.flag_atterraggio = False

        self.HFOV = 1.047 # letto dal .sdf



        # Altre costanti 

        # coordinate (grossolane) della zona della stazione di atterraggio (in metri)
        self.ZONA_STAZIONE = [0, 0, -5] # la z viene messa a 15 metri 

        self.ID_ATTESI_1 = {1} # marker da riconoscere nella terza fase
        self.ID_ATTESI_2 = {2, 3, 4, 5} # marker da riconoscere nella terza fase
        self.ID_ATTESI_3 = {6, 7, 8, 9} # marker da riconoscere nella terza fase

        self.KP_3 = 0.5        # velocità di correzzione (1/s)

        self.VMAX_3 = 0.4      # m/s di correzione massima

        self.ERRORE_ALLINEAMENTO_3 = 0.05 # m, soglia di allineamento

        # Variabili globali

        self.coordinate_marker_1 = []   # coordinate globali conosciute per dare una prima direzione
        self.coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
        self.coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}

        self.flag_atterraggio = False

    def get_local_pos(self,timeout=2):
        m = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
        if m is None:
            raise TimeoutError("Nessun LOCAL_POSITION_NED ricevuto")
        return m.x, m.y, m.z


    # Funzione per avvicinarsi alla generale zona della stazione di atterraggio
    def avvicinamento_zona_stazione(self):
        _,_,z = self.get_local_pos() # rimango alla stessa altezza
        if z < self.ZONA_STAZIONE[2]: # se sono più basso della zona della stazione, rimango basso
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

    def avvio_streaming(self):
        cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("Errore: impossibile aprire lo stream video")
            exit(1)
        last_hb = 0
        while True:
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
            W, H = frame.shape[1], frame.shape[0]      # Larghezza ed altezza della telecamera  
                                                                    #                                            [fx  0  cx]
            fx = (W / 2) / np.tan(self.HFOV / 2)            # formula per lunghezza focale                               [0  fy  cy]
            camera_matrix = np.array([[fx, 0, W / 2],  # matrice della camera con la forma standard     ->          [0   0   1]
                                    [0, fx, H / 2],
                                    [0,  0,   1]], dtype=np.float64)
            dist_coeffs = np.zeros((5, 1))             # in gazebo la camera è ideale quindi non c'è distorsione

            if corners:
                for corner, marker_id in zip(corners, ids):
                    cv2.polylines(
                        frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)

            cv2.imshow("Drone Camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()