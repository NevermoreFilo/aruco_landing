import cv2
import numpy as np
from pymavlink import mavutil
import time

import utils 

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters_create()

# Parametri intrinseci della camera (vanno calibrati - vedi nota sotto)
#camera_matrix = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
dist_coeffs = np.zeros((5,1)) # coefficenti di distorsione (vanno calibrati - vedi nota sotto)
marker_size = 0.1  # metri
h = marker_size / 2
marker_points = np.array([[-h,  h, 0], # alto sinistra
                       [ h,  h, 0], # alto destra ...
                       [ h, -h, 0],
                       [-h, -h, 0]], dtype=np.float32)
flag_atterraggio = False
TARGET_ALT = 10  # meters above the takeoff point

coordinate_marker_3 = {}   # dizionario {id: [x, y, z]}
ID_ATTESI_3 = {6, 7, 8, 9} # marker da riconoscere nella terza fase


# # Funzione per ottenere le coordinate attuali (in metri) del drone
# def get_local_pos(timeout=2):
#     m = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
#     if m is None:
#         raise TimeoutError("Nessun LOCAL_POSITION_NED ricevuto")
#     return m.x, m.y, m.z

# def calibrazione(coordinate): 
#     _,_,z = get_local_pos() # rimango alla stessa altezza
#     master.mav.set_position_target_local_ned_send(
#             0,                                   # time_boot_ms (ignorato)
#             master.target_system, master.target_component,
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#             0b0000111111111000,                  # usa solo posizione x, y, z
#             coordinate[0], coordinate[1], z,     # mantengo z uguale
#             0, 0, 0,                             # velocità (ignorate)
#             0, 0, 0,                             # accelerazioni (ignorate)
#             0, 0)                                # yaw, yaw_rate (ignorati)


KP = 0.5        # guadagno (1/s)
VMAX = 0.5      # m/s
TOL = 0.05      # m, soglia di allineamento
ULTIMO_INVIO = 0
INTERVALLO = 0.1   # invia a 10 Hz, non a ogni frame
KP_YAW = 0.5       # 1/s
YAW_RATE_MAX = 0.5 # rad/s (circa 30°/s)

ANGOLO_RIF = np.pi / 2           # angolo di 6→7 quando è allineato
TOL_YAW = np.radians(1)

HFOV = 1.047                               # letto dal .sdf

def wrap(a): # funzione per limitare l'angolo tra -pi e pi
    return (a + np.pi) % (2 * np.pi) - np.pi


def invia_comando(vx, vy, yaw_rate):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,          # velocità + yaw_rate
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, yaw_rate)                 # yaw ignorato, yaw_rate usato

def send_target():
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000101111111000,              # position + yaw
        0.3, 0.2, -1,
        0, 0, 0,
        0, 0, 0,
        20, 0
    )

print("Starting video capture from drone...")
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
#send_target()  # invio il primo target per far partire il drone

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Errore: impossibile aprire lo stream video")
    exit(1)
last_hb = 0

while True:
    coordinate_marker_3 = {} # azzero ad ogni frame
    if time.time() - last_hb >= 1:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = time.time()
    while (m := master.recv_match(blocking=False)):
        if m.get_type() == 'GLOBAL_POSITION_INT':
            last_pos = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000)
    ret, frame = cap.read()
    if not ret:
        print("NO frame")
        continue
    corners, ids, rejected = cv2.aruco.detectMarkers(
    frame,
    aruco_dict,
    parameters=detector_params
)
    W, H = frame.shape[1], frame.shape[0]      # Larghezza ed altezza della telecamera  
                                                               #                                            [fx  0  cx]
    fx = (W / 2) / np.tan(HFOV / 2)            # formula per lunghezza focale                               [0  fy  cy]
    camera_matrix = np.array([[fx, 0, W / 2],  # matrice della camera con la forma standard     ->          [0   0   1]
                            [0, fx, H / 2],
                            [0,  0,   1]], dtype=np.float64)
    dist_coeffs = np.zeros((5, 1))             # in gazebo la camera è ideale quindi non c'è distorsione
    
    if corners:
        for i, corner in enumerate(corners):
            pts = corner[0]
            print("Punti marker (pixel)", i, ": ", pts[0], pts[1], pts[2], pts[3])
        #centroide1 = utils.centroide_marker(corners[0][0])
        #centroide2 = utils.centroide_marker(corners[0][0])
        flag_atterraggio = True
        for corner, marker_id in zip(corners, ids):
            cv2.polylines(
                frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)
            ok, rvec, tvec = cv2.solvePnP(marker_points, corner[0], camera_matrix, # conversione da pixel a coordinate
                                  dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                coordinate_marker_3[int(marker_id[0])] = tvec.flatten().tolist()
                x,y,z = tvec.flatten()
                print(f"Marker {marker_id[0]}: x={x:.2f} m, y={y:.2f} m, z={z:.2f} m")

            # if ids is not None and ID_ATTESI_3.issubset(set(ids.flatten().tolist())):
            #     print("Tutti e 4 i marker visibili, calibrazione...")
            #     centro_stimato = np.mean(list(coordinate_marker_3.values()), axis=0) # media delle x, y e z dei marker visibili
            #     calibrazione(centro_stimato)
            #     break
            # else:
            #     mancanti = ID_ATTESI_3 - set(ids.flatten().tolist() if ids is not None else [])
            #     print("Mancano:", mancanti)
            
        visibili = set(ids.flatten().tolist()) if ids is not None else set()
        if ID_ATTESI_3.issubset(visibili) and time.time() - ULTIMO_INVIO > INTERVALLO:
            punti = [coordinate_marker_3[i] for i in ID_ATTESI_3]   # solo i 4 attesi
            cx, cy, cz = np.mean(punti, axis=0)

            err_avanti = -cy
            err_destra = cx
            p = coordinate_marker_3
            v1 = np.array(p[7]) - np.array(p[6])
            v2 = np.array(p[9]) - np.array(p[8])        # coppia parallela, per ridurre il rumore
            v = v1 + v2
            err_yaw = wrap(np.arctan2(v[1], v[0]) - ANGOLO_RIF)

            yaw_rate = np.clip(KP_YAW * err_yaw, -YAW_RATE_MAX, YAW_RATE_MAX)
        #     print(f"err_avanti={err_avanti:.2f} err_destra={err_destra:.2f} "
        #         f"err_yaw={np.degrees(err_yaw):.1f}° yaw_rate={yaw_rate:.2f}")

        #     if abs(err_avanti) < TOL and abs(err_destra) < TOL and abs(err_yaw) < TOL_YAW:
        #         invia_comando(0, 0, 0)
        #         print("Allineato")
        #     else:
        #         vx = np.clip(KP * err_avanti, -VMAX, VMAX)
        #         vy = np.clip(KP * err_destra, -VMAX, VMAX)
        #         # se la posizione è già a posto ma lo yaw no, ruota soltanto
        #         if abs(err_avanti) < TOL and abs(err_destra) < TOL:
        #             vx = vy = 0
        #         if abs(err_yaw) < TOL_YAW:
        #             yaw_rate = 0
        #         invia_comando(vx, vy, yaw_rate)
        #     ULTIMO_INVIO = time.time()
        # elif not ID_ATTESI_3.issubset(visibili):
        #     invia_comando(0, 0, 0)     # marker persi: fermati

    cv2.imshow("Drone Camera", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('l') and flag_atterraggio:
        flag_atterraggio = False
        master.set_mode('LAND')
    if key == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

