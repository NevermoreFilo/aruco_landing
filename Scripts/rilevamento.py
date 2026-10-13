import cv2
import numpy as np
from pymavlink import mavutil
import time

import utils 

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters_create()

# Parametri intrinseci della camera (vanno calibrati - vedi nota sotto)
#camera_matrix = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
dist_coeffs = np.zeros((5,1))
marker_size = 1  # metri
flag_atterraggio = False
TARGET_ALT = 10  # meters above the takeoff point


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

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Errore: impossibile aprire lo stream video")
    exit(1)
last_hb = 0
while True:
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
    if corners:
        print("Marker 1: ", corners[3])
        centroide1 = utils.centroide_marker(corners[0][0])
        centroide2 = utils.centroide_marker(corners[0][0])
        #cv2.SOLVEPNP_IPPE_SQUARE()
        flag_atterraggio = True
        for corner, marker_id in zip(corners, ids):
            cv2.polylines(
                frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)
    cv2.imshow("Drone Camera", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('l') and flag_atterraggio:
        flag_atterraggio = False
        master.set_mode('LAND')
    if key == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

