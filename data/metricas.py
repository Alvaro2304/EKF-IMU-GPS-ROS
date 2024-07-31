#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Cargar los datos
data = pd.read_csv('zigzag_2.txt')  # Ajusta la ruta y el nombre del archivo

# Asegurarse de que los nombres de las columnas no tengan espacios extra
data.columns = [col.strip() for col in data.columns]

# Calcular los errores
error_x = data['EKF_X'] - data['Target_X']
error_y = data['EKF_Y'] - data['Target_Y']
error_total = np.sqrt(error_x**2 + error_y**2)

# Calcular las métricas
mae_x = np.mean(np.abs(error_x))
mae_y = np.mean(np.abs(error_y))
mae_total = np.mean(error_total)

rmse_x = np.sqrt(np.mean(error_x**2))
rmse_y = np.sqrt(np.mean(error_y**2))
rmse_total = np.sqrt(np.mean(error_total**2))

# Imprimir los resultados
print('MAE en X: {} m'.format(mae_x))
print('MAE en Y: {} m'.format(mae_y))
print('MAE Total: {} m'.format(mae_total))

print('RMSE en X: {} m'.format(rmse_x))
print('RMSE en Y: {} m'.format(rmse_y))
print('RMSE Total: {} m'.format(rmse_total))

# Comparar con el ancho del mecanismo de limpieza
if mae_total <= 0.43:
    print('El error medio absoluto total es tolerable.')
else:
    print('El error medio absoluto total NO es tolerable.')

if rmse_total <= 0.43:
    print('El error cuadrático medio total es tolerable.')
else:
    print('El error cuadrático medio total NO es tolerable.')


# Visualización
plt.figure()
plt.plot(data['Target_X'], data['Target_Y'], 'r--', label=u'Trayectoria deseada')
plt.plot(data['EKF_X'], data['EKF_Y'], 'b-', label=u'Posición estimada EKF')

# Añadir la banda de limpieza alrededor de la trayectoria deseada
upper_bound = data['Target_Y'] + 0.215
lower_bound = data['Target_Y'] - 0.215
plt.fill_between(data['Target_X'], lower_bound, upper_bound, color='yellow', alpha=0.3, label=u'Banda de limpieza (43 cm)')
# Añadir la banda de limpieza alrededor de la trayectoria deseada
upper_bound_x = data['Target_X'] + 0.215
lower_bound_x = data['Target_X'] - 0.215
plt.fill_betweenx(data['Target_Y'], lower_bound_x, upper_bound_x, color='yellow', alpha=0.5)

# Añadir la banda de limpieza alrededor de la trayectoria estimada
upper_bound_ekf = data['EKF_Y'] + 0.215
lower_bound_ekf = data['EKF_Y'] - 0.215
plt.fill_between(data['EKF_X'], lower_bound_ekf, upper_bound_ekf, color='lime', alpha=1., label=u'Banda de limpieza (43 cm) - Estimada')
# Añadir la banda de limpieza alrededor de la trayectoria estimada
upper_bound_ekf = data['EKF_X'] + 0.215
lower_bound_ekf = data['EKF_X'] - 0.215
plt.fill_betweenx(data['EKF_Y'], lower_bound_ekf, upper_bound_ekf, color='lime', alpha=1.)


plt.xlabel(u'X (m)')
plt.ylabel(u'Y (m)')
plt.title(u'Seguimiento de Trayectoria')
plt.legend()
plt.show()
