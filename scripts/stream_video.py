#export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

import threading
import time
import cv2
import numpy as np
import subprocess
import os

class StreamVideo:
    def __init__(self):
        # self.pipeline = ( # non più usata dopo il passaggio a FFMPEG
        #     "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 "
        #     "! rtph264depay ! avdec_h264 ! videoconvert "
        #     "! appsink name=appsink sync=false drop=true max-buffers=1"
        # )
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "protocol_whitelist;file,udp,rtp|"
            "fflags;nobuffer|"
            "flags;low_delay|"
            "analyzeduration;0|"
            "threads;1|"
            "max_delay;0|"
            "reorder_queue_size;0"
        )
        self.sdp_path = "/tmp/stream.sdp" # necessario per usare FFMPEG
        sdp = (
            "v=0\n"
            "o=- 0 0 IN IP4 127.0.0.1\n"
            "s=No Name\n"
            "c=IN IP4 127.0.0.1\n"
            "t=0 0\n"
            "a=tool:libavformat\n"
            "m=video 5600 RTP/AVP 96\n"
            "a=rtpmap:96 H264/90000\n"
        )
        with open(self.sdp_path, "w") as f: # creo il file se non già presente
            f.write(sdp)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary( # dizionario dei marker
            cv2.aruco.DICT_4X4_50
        )

        self.detector_params = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.detector_params
        )

        self.dist_coeffs = np.zeros((5, 1)) # coefficienti distorsione della telecamera
        self.HFOV = 1.047 # fov telecamera (letto da sdf)
        self.TOPIC = ("/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/"
         "pitch_link/sensor/camera/image/enable_streaming")

        self.flag_stream_attivo = False # flag che evita che il drone parta fino a quando lo stream non è online

        def quadrato(lato):
            h = lato / 2
            return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)

        self.marker_points_1 = quadrato(1.0)    # ID 0, 1 x 1 metri
        self.marker_points_2 = quadrato(0.4)    # ID 1-4, 40 x 40 cm
        self.marker_points_3 = quadrato(0.1)    # ID 5-8, 10 x 10

        self._lock = threading.Lock()
        self._marker = {}                        # {id: dict(tvec, angolo, t)}

    def punti_per_id(self, mid):
        if mid == 0:
            return self.marker_points_1
        if mid in (1, 2, 3, 4):
            return self.marker_points_2
        if mid in (5, 6, 7, 8):
            return self.marker_points_3
        return None

    # Funzione per la copia dei marker dell'ultimo frame, sicura da qualsiasi thread
    def snapshot(self):
        with self._lock:
            return dict(self._marker)


    def abilita_streaming(self):
        print("Pubblicazione topic...")
        r = subprocess.run(["gz", "topic", "-t", self.TOPIC, "-m", "gz.msgs.Boolean", "-p", "data: 1"], # comando che altrimenti lancerei da terminale
                        capture_output=True, text=True, timeout=10)
        if r.returncode != 0 or r.stderr:
            print("gz topic:", r.returncode, r.stderr.strip())

    # Funzione che continua a provare a pubblicare il topic finché lo stream non parte
    def richiedi_stream(self, stop_event, frame_arrivato): 
        while not stop_event.is_set() and not frame_arrivato.is_set():
            try:
                self.abilita_streaming()
            except subprocess.TimeoutExpired:
                pass
            frame_arrivato.wait(3)          # riprova ogni 3s finché non arriva un frame

    def avvio_streaming(self, stop_event, timeout=90):
        print("Avvio streaming...")
        cap_pronto = threading.Event()
        cap_holder = {}
        def apri():
            cap_holder["cap"] = cv2.VideoCapture(self.sdp_path, cv2.CAP_FFMPEG)
            cap_pronto.set()

        try:
            self.abilita_streaming()
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1)

        # # apre la pipeline
        threading.Thread(target=apri, daemon=True).start()
        time.sleep(2)                      # lascio a GStreamer il tempo di mettersi in ascolto

        # accendo lo stream, riprovando finché non arriva un frame
        frame_arrivato = threading.Event()
        threading.Thread(target=self.richiedi_stream, args=(stop_event, frame_arrivato),
                        daemon=True).start()

        if not cap_pronto.wait(timeout):
            print("Errore: pipeline non aperta")
            stop_event.set()
            return
        cap = cap_holder["cap"]
        if not cap.isOpened():
            print("Errore: impossibile aprire lo stream video")
            stop_event.set()
            return
        frame = None
        t0 = time.time()
        while frame is None and not stop_event.is_set():
            ret, f = cap.read()
            if ret:
                frame = f
                frame_arrivato.set()
            elif time.time() - t0 > timeout:
                print("Errore: nessun frame ricevuto")
                stop_event.set()
        if frame is None:
            cap.release()
            return
        self.flag_stream_attivo = True
        H, W = frame.shape[:2]
        fx = (W / 2) / np.tan(self.HFOV / 2)
        camera_matrix = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]], dtype=np.float64)

        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue
            corners, ids, _ = self.detector.detectMarkers(frame)

            attuali = {}
            if ids is not None:
                for corner, marker_id in zip(corners, ids):
                    mid = int(np.ravel(marker_id)[0])
                    cv2.polylines(frame, [corner.astype(np.int32)], True, (0, 255, 255), 3, cv2.LINE_AA)
                    pts = self.punti_per_id(mid)
                    if pts is None:
                        continue
                    ok, rvec, tvec = cv2.solvePnP(pts, corner[0], camera_matrix, self.dist_coeffs,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        continue
                    dx, dy = corner[0][1] - corner[0][0]
                    attuali[mid] = dict(tvec=tvec.flatten(), angolo=np.arctan2(dy, dx), t=time.time())
            with self._lock:
                self._marker = attuali

            cv2.imshow("Drone Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
        cap.release()
        cv2.destroyAllWindows()