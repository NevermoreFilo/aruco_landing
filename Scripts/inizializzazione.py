from pymavlink import mavutil
import time

# Classe per gestire l'inizializzazione del drone, compreso il decollo e il movimento verso una posizione iniziale

class Inizializzazione():

    # Richiede la connessione a MAVLINK, altitudine di e coordinate (x,y,z) iniziali
    def __init__(self, master, altitudine_iniziale, coordinate_iniziali):
        self.master = master
        self.ALTITUDINE_INIZIALE = altitudine_iniziale
        self.COORDINATE_INIZIALI = coordinate_iniziali


    # Funzione per spostare il drone in una posizione target da cui iniziare la simulazione
    def movimento_drone(self):

        # 1. Switch a GUIDED mode
        mode_id = self.master.mode_mapping()['GUIDED']
        self.master.set_mode('GUIDED')
        t0 = time.time()
        while self.master.flightmode != 'GUIDED':
            self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if time.time() - t0 > 10:
                raise RuntimeError("Errore nello switch a GUIDED")
        print("Modalità GUIDED attiva")

        # 2. Arm dei motori (ed attesa che siano effettivamente armati)
        self.master.arducopter_arm()
        t0 = time.time()
        while not self.master.motors_armed():
            m = self.master.recv_match(type=['HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK'],
                                blocking=True, timeout=1)
            if m is not None:
                if m.get_type() == 'STATUSTEXT':
                    print("Autopilot:", m.text)
                elif m.get_type() == 'COMMAND_ACK':
                    print("ACK:", m)
            if time.time() - t0 > 30:
                raise RuntimeError("Errore nell'armamento dei motori o timeout")
        print("Motori armati")

        # 3. Partenza
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                              # conferma
            0, 0, 0, 0,                     # param1-4: pitch, empty, empty, yaw (non usati per i copter)
            0, 0,                           # param5-6: lat, lon (0 = posizione attuale)
            self.ALTITUDINE_INIZIALE        # param7: altitude in metri
        )

        # 4. Attesa fino a quando l'altitudine target è (quasi) raggiunta
        last_hb = 0
        alt = 0.0
        t0 = time.time()

        while True:
            # heartbeat una volta al secondo
            if time.time() - last_hb >= 1:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = time.time()

            # lettura dei messaggi in arrivo dal drone
            while (m := self.master.recv_match(blocking=False)):
                t = m.get_type()
                if t == 'GLOBAL_POSITION_INT':
                    alt = m.relative_alt / 1000

            print(f"Altitude: {alt:.1f} m")
            if alt >= self.ALTITUDINE_INIZIALE * 0.95:
                print("Altitudine target raggiunta")
                break

            if time.time() - t0 > 60:
                raise RuntimeError("Takeoff timeout")
            time.sleep(0.1) # per evitare di saturare la CPU con un loop troppo veloce

        # Ci si assicura che LOCAL_POSITION_NED venga trasmesso a 10 Hz
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            100000, 0, 0, 0, 0, 0
        )

        # 5. Movimento verso la posizione target da cui iniziare la simulazione
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,              # posizione
            self.COORDINATE_INIZIALI[0], self.COORDINATE_INIZIALI[1], self.COORDINATE_INIZIALI[2],  # x, y, z
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        while True:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1) # attendo il messaggio di posizione locale
            dist = ((msg.x - self.COORDINATE_INIZIALI[0])**2 +
                    (msg.y - self.COORDINATE_INIZIALI[1])**2 +
                    (msg.z - self.COORDINATE_INIZIALI[2])**2) ** 0.5
            if(dist < 1): # se la distanza dalla posizione target è inferiore a 1 metro, esco dal loop
                print("Posizione target raggiunta")
                break
