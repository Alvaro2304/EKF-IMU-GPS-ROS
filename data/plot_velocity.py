#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd

# Leer los datos desde el archivo
file_name = "velocity_zigzag_2.txt"
data = pd.read_csv(file_name, header=0, names=['Timestamp', 'Linear_Velocity', 'Angular_Velocity'])

# Convertir las marcas de tiempo a objetos datetime
data['Timestamp'] = pd.to_datetime(data['Timestamp'])

# Calcular el tiempo en segundos desde el inicio
start_time = data['Timestamp'].iloc[0]
data['Time'] = (data['Timestamp'] - start_time).dt.total_seconds()

# Crear los subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Graficar la velocidad lineal en el tiempo
ax1.plot(data['Time'], data['Linear_Velocity'], label='Velocidad lineal')
ax1.set_ylabel('Velocidad lineal (m/s)')
ax1.legend()
ax1.grid(True)

# Graficar la velocidad angular en el tiempo
ax2.plot(data['Time'], data['Angular_Velocity'], label='Velocidad angular', color='orange')
ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Velocidad angular (rad/s)')
ax2.legend()
ax2.grid(True)

# Mostrar la gráfica
plt.show()
