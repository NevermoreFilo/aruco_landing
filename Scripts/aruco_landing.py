import cv2
import numpy as np
from pymavlink import mavutil
import time


# Creazione del dizionario ArUco e dei parametri del rilevatore
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters_create()

# Costanti intrinseche del drone e della camera
dist_coeffs = np.zeros((5,1)) # coefficenti di distorsione (essendo una telecamera simulata, non ci sono distorsioni)
marker_size_1 = 0.05  # metri
marker_size_2 = 0.05  # metri
marker_size_3 = 0.05  # metri


h3 = marker_size_3 / 2
marker_points_3 = np.array([[-h3,  h3, 0], # alto sinistra
                       [ h3,  h3, 0], # alto destra ...
                       [ h3, -h3, 0],
                       [-h3, -h3, 0]], dtype=np.float32)

# Costanti
ID_ATTESI_1 = {1} # marker da riconoscere nella terza fase
ID_ATTESI_2 = {2, 3, 4, 5} # marker da riconoscere nella terza fase
ID_ATTESI_3 = {6, 7, 8, 9} # marker da riconoscere nella terza fase

KP_3 = 0.5        # velocità di correzzione (1/s)

VMAX_3 = 0.4      # m/s di correzione massima

ERRORE_ALLINEAMENTO_3 = 0.05 # m, soglia di allineamento

# Variabili globali

coordinate_marker_1 = {}   # dizionario {id: [x, y, z]}
coordinate_marker_2 = {}   # dizionario {id: [x, y, z]}
coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}

flag_atterraggio = False
