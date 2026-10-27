import numpy as np
import subprocess
import re

# # Funzione che dato un ArucoMarker calcola la media dei 4 angoli / dati i 4 centroidi triangola la posizione intermedia
# def centroide_marker(coords):
#     x = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) /4
#     y = (coords[0][1] + coords[1][1] + coords[2][1] + coords[3][1]) /4
#     return [x,y]

def wrap(a): # funzione per limitare l'angolo tra -pi e pi
    return (a + np.pi) % (2 * np.pi) - np.pi

# Funzione per ritornare la posizione reale (letta da Gazebo e quindi priva del bias nel GPS)
def posizione_reale_gazebo(modello="iris_with_gimbal", mondo="iris_runway", timeout=10):
    out = subprocess.run( # interroga il topic gazebo per avere tutte le pose di tutti 
        ["gz", "topic", "-e", "-t", f"/world/{mondo}/dynamic_pose/info", "-n", "1"],
        capture_output=True, text=True, timeout=timeout).stdout
    i = out.find(f'name: "{modello}"') # cerco le info per iris_with_gimbal
    if i < 0:
        raise RuntimeError(f"Modello {modello} non trovato nel topic delle pose") # failsafe
    blocco = out[i:] # ignoro le info prima di quelle su iris_with_gimbal
    fine = blocco.find("orientation")          # la posizione precede l'orientamento quindi tronco
    if fine > 0:
        blocco = blocco[:fine] 
    def campo(nome):
        m = re.search(rf"\b{nome}:\s*(-?[\d.]+(?:e[-+]?\d+)?)", blocco)
        return float(m.group(1)) if m else 0.0
    return campo("x"), campo("y"), campo("z")  