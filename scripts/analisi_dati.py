import csv
import os
import statistics as stats
import sys

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
import math


if not os.path.exists("risultati.csv"):
    print("Errore - file 'risultati.csv' non trovato nella cartella corrente")
    sys.exit(1)

totale = 0
counter_esiti_errati = 0
errori_sigma_0 = []
errori_sigma_0_5 = []
errori_sigma_1 = []
errori_sigma_2 = []
errori_sigma_3 = []


with open("risultati.csv", newline="") as file:
    reader = csv.DictReader(file)
    for row in reader:
        totale += 1
        if row["esito"] != "ok":
            counter_esiti_errati += 1
        else:
            match row["sigma"]:
                case 0.0:
                    errori_sigma_0.append(float(row["errore"]))
                case 0.5:
                    errori_sigma_0_5.append(float(row["errore"]))
                case 1.0:
                    errori_sigma_1.append(float(row["errore"]))
                case 2.0:
                    errori_sigma_2.append(float(row["errore"]))
                case 3.0:
                    errori_sigma_3.append(float(row["errore"]))
if not counter_esiti_errati == totale:
    print("Nessun atterraggio riuscito: impossibile calcolare le statistiche sull'errore")
    sys.exit(1)

media_errore_sigma0 = stats.mean(errori_sigma_0) * 100
mediana_errore_sigma_0 = stats.median(errori_sigma_0) * 100 # moltiplico per 100 per passare a centimetri
dev_std_errore_sigma_0 = stats.stdev(errori_sigma_0) if len(errori_sigma_0) > 1 else 0.0
varianza_errore_sigma_0 = np.var(errori_sigma_0)

print("Risultati corretti: ", totale - counter_esiti_errati, ", risultati errati: ", counter_esiti_errati)
print("Errore medio: ", media_errore_sigma0, ", mediana errore: ", mediana_errore_sigma_0, ", deviazione standard: ", dev_std_errore_sigma_0)

sig_0 = math.sqrt(varianza_errore_sigma_0)
x = np.linspace(media_errore_sigma0 - 3*sig_0, media_errore_sigma0 + 3*sig_0, 100)
plt.plot(x, stats.norm.pdf(x, media_errore_sigma0, sig_0))
plt.show()