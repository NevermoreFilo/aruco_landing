
import subprocess
import os
import time
import signal
from pymavlink import mavutil
import csv

ARDUPILOT_DIR = os.path.expanduser("~/ardupilot/Tools/autotest")   # adatta
PARM = os.path.expanduser("~/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm")
LIVELLI_SIGMA = [0, 0.5, 1, 2, 3]
RIPETIZIONI = 10
CSV_OUT = "risultati.csv"
CAMPI = ["sigma", "seed", "esito", "errore", "x", "y", "yaw"]

# Funzione che crea l'ambiente di simulazione, attivando gazebo e SITL
def crea_ambiente_simulazione():
    procs = []
    procs.append(subprocess.Popen(
        ["gz", "sim", "-v4", "-r", "iris_runway.sdf"],
        preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(10)
    procs.append(subprocess.Popen(
        ["./sim_vehicle.py", "-D", "-v", "ArduCopter", "-f", "JSON", "-w", "--daemon",
         f"--add-param-file={PARM}"],
        cwd=ARDUPILOT_DIR, preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    return procs

def distruggi_ambiente_simulazione(processi):
    for p in reversed(processi): # reversed per chiudere prima SITL di gazebo
        try: os.killpg(os.getpgid(p.pid), signal.SIGTERM) # invio un segnale di terminazione all'ambiente
        except ProcessLookupError: pass
    time.sleep(3)
    for p in processi:
        try: os.killpg(os.getpgid(p.pid), signal.SIGKILL) # se qualcuno non è terminato, lo distruggo forzatamente
        except ProcessLookupError: pass
    subprocess.run(["pkill", "-9", "-f", "arducopter"]) # debug / misura di sicurezza
    subprocess.run(["pkill", "-9", "-f", "gz sim"])
    time.sleep(5)

def ambiente_pronto(timeout=120):
    m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    try:
        if m.wait_heartbeat(timeout=timeout) is None: # aspetto heartbeat
            return False
        t0 = time.time()
        while time.time() - t0 < timeout:
            g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2) # aspetto di ricevere un messaggio con lo stato del gps
            if g and g.fix_type >= 3: # se è >=3 vuoldire che la qualità è almeno 3D 
                return True
        return False
    except Exception:
        return False
    finally:
        m.close()          # in ogni caso la porta viene liberata perchè pubblica nella stessa del main

# Funzione per segnare nel file csv quando un tentativo fallisce a monte
def registra_fallito(sigma, seed, motivo=""):
    nuovo = not os.path.exists(CSV_OUT)
    with open(CSV_OUT, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CAMPI)
        if nuovo:
            w.writeheader()
        w.writerow({"sigma": sigma, "seed": seed, "esito": "fallito",
                    "errore": "N/A", "x": "N/A", "y": "N/A", "yaw": "N/A"})
        

for sigma in LIVELLI_SIGMA:
    for seed in range(RIPETIZIONI):
        processi = crea_ambiente_simulazione()
        try:
            if not ambiente_pronto():
                registra_fallito(sigma, seed, "Ambiente di simulazione non pronto")
                continue
            subprocess.run(["python3", "main.py", "--sigma", str(sigma), "--seed", str(seed), "--output", CSV_OUT], timeout=300)
        except subprocess.TimeoutExpired:
            continue
        finally:
            distruggi_ambiente_simulazione(processi)
        if not riga_presente(sigma, seed): # se non c'è la riga nel file vuoldire che il main non è arrivato a fine blocco e c'è stato un arresto anomalo, come un timeout
            registra_fallito(sigma, seed, "Arresto anomalo")

