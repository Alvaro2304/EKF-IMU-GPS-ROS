#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Leer los datos desde el archivo
file_name = "sinusoid_2.txt"
data = pd.read_csv(file_name, header=0, names=['Timestamp', 'EKF_X', 'EKF_Y', 'Covariance[0_0]', 'Covariance[0_1]', 'Covariance[1_0]', 'Covariance[1_1]', 'Target_X', 'Target_Y'])

# Convertir las marcas de tiempo a objetos datetime
data['Timestamp'] = pd.to_datetime(data['Timestamp'])

# Calcular el tiempo en segundos desde el inicio
start_time = data['Timestamp'].iloc[0]
data['Time'] = (data['Timestamp'] - start_time).dt.total_seconds()

# Filtrar solo las filas donde Target_X o Target_Y cambian
target_changed = data[(data['Target_X'] != data['Target_X'].shift(1)) | (data['Target_Y'] != data['Target_Y'].shift(1))]

# Calcular las desviaciones estándar a partir de las covarianzas
data['Std_X'] = data['Covariance[0_0]'].apply(np.sqrt)
data['Std_Y'] = data['Covariance[1_1]'].apply(np.sqrt)


# Crear los subplots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

# Graficar EKF_X y Target_X en el tiempo
ax1.plot(data['Time'], data['EKF_X'], label='X EKF')
ax1.plot(target_changed['Time'], target_changed['Target_X'], '-', label='X deseado', color='red')

# Graficar las bandas de confianza de EKF_X como líneas punteadas
ax1.fill_between(data['Time'], data['EKF_X'] + data['Std_X'], data['EKF_X'] - data['Std_X'], color='blue', alpha=0.3, label='Desviacion estandar')

ax1.set_ylabel('X (m)')
ax1.legend()
ax1.grid(True)

# Graficar EKF_Y y Target_Y en el tiempo
ax2.plot(data['Time'], data['EKF_Y'], label='Y EKF', color='orange')
ax2.plot(target_changed['Time'], target_changed['Target_Y'], '-', label='Y deseado', color='green')

# Graficar las bandas de confianza de EKF_Y como líneas punteadas
ax2.fill_between(data['Time'], data['EKF_Y'] + data['Std_Y'], data['EKF_Y'] - data['Std_Y'], color='purple', alpha=0.3, label='Desviacion estandar')

ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Y (m)')
ax2.legend()
ax2.grid(True)
# Calcular e imprimir la media de la desviación estándar para cada eje
mean_std_x = np.mean(data['Std_X'])
mean_std_y = np.mean(data['Std_Y'])

print 'Media de la desviación estándar en X: {:.4f} m'.format(mean_std_x)
print 'Media de la desviación estándar en Y: {:.4f} m'.format(mean_std_y)
# Mostrar la gráfica
plt.show()

