import csv
import os
import statistics
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st

FILE = "risultati2.csv"
if not os.path.exists(FILE):
    print(f"Errore - file '{FILE}' non trovato nella cartella corrente")
    sys.exit(1)

totale = 0
errati = 0
errori = defaultdict(list)   # sigma -> lista errori in cm
errori_tot = []

with open(FILE, newline="") as f:
    for row in csv.DictReader(f):
        totale += 1
        if row["esito"] != "ok":
            errati += 1
        else:
            errori[float(row["sigma"])].append(float(row["errore"]) * 100)  # m -> cm
            errori_tot.append(float(row["errore"])*100)

if errati == totale:
    print("Nessun atterraggio riuscito: impossibile calcolare le statistiche sull'errore")
    sys.exit(1)

print(f"Risultati corretti: {totale - errati}, risultati errati: {errati}")

plt.figure()
for sigma in sorted(errori):
    dati = errori[sigma]
    media = statistics.mean(dati)
    mediana = statistics.median(dati)
    dev = statistics.stdev(dati) if len(dati) > 1 else 0.0
    print(f"sigma={sigma}: n={len(dati)}, media={media:.2f} cm, "
          f"mediana={mediana:.2f} cm, dev.std={dev:.2f} cm")

    if dev > 0:
        # x = np.linspace(media - 3 * dev, media + 3 * dev, 200)
        # plt.plot(x, st.rayleigh.pdf(x, media, dev), label=f"σ = {sigma}")
        loc, scale = st.rayleigh.fit(dati, floc=0)   # floc=0: parte da zero
        x = np.linspace(0, max(dati) * 1.2, 200)
        plt.plot(x, st.rayleigh.pdf(x, loc, scale), label=f"σ = {sigma}")
        #plt.hist(dati, bins=20, density=True, alpha=0.3)

plt.xlabel("Errore di atterraggio [cm]")
plt.ylabel("Densità")
plt.legend()
plt.show()

media_tot = statistics.mean(errori_tot)
mediana_tot = statistics.median(errori_tot)
dev_tot = statistics.stdev(errori_tot) if len(errori_tot) > 1 else 0.0
print(f"media={media_tot:.2f} cm, "
        f"mediana={mediana_tot:.2f} cm, dev.std={dev_tot:.2f} cm")
plt.figure()
loc, scale = st.rayleigh.fit(errori_tot, floc=0)   # floc=0: parte da zero
x = np.linspace(0, max(errori_tot) * 1.2, 200)
plt.plot(x, st.rayleigh.pdf(x, loc, scale), label=f"σ = {sigma}")
plt.xlabel("Errore di atterraggio [cm]")
plt.ylabel("Densità")
plt.legend()
plt.show()
