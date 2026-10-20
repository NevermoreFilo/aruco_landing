import numpy as np

# Funzione che dato un ArucoMarker calcola la media dei 4 angoli / dati i 4 centroidi triangola la posizione intermedia
def centroide_marker(coords):
    x = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) /4
    y = (coords[0][1] + coords[1][1] + coords[2][1] + coords[3][1]) /4
    return [x,y]

def wrap(a): # funzione per limitare l'angolo tra -pi e pi
    return (a + np.pi) % (2 * np.pi) - np.pi

# Funzione per ritornare l'errore totale dato dalla differenza tra due coordinate
def confronta_coordinate(coord1, coord2):
    err_x = abs(coord1[0] - coord2[0])
    err_y = abs(coord1[1] - coord2[1])
    return err_x + err_y 
