#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import pandas as pd
import os
import numpy as np

def plot_ekf_data(file_path):
    # Leer los datos del archivo
    data = pd.read_csv(file_path)

    # Imprimir los nombres de las columnas para depuración
    print("Columnas disponibles en el archivo CSV:")
    print(data.columns)

    # Extraer los datos de las columnas
    ekf_x = data[' EKF_X']
    ekf_y = data[' EKF_Y']
    cov_00 = data[' Covariance[0_0]']
    cov_01 = data[' Covariance[0_1]']
    cov_10 = data[' Covariance[1_0]']
    cov_11 = data[' Covariance[1_1]']
    target_x = data[' Target_X']
    target_y = data[' Target_Y']

    # Calcular el error máximo en X y Y
    error_x = np.abs(ekf_x - target_x)
    error_y = np.abs(ekf_y - target_y)
    max_error_x = np.max(error_x)
    max_error_y = np.max(error_y)

    # Imprimir el error máximo en X y Y
    print('Error máximo en X: {:.4f} m'.format(max_error_x))
    print('Error máximo en Y: {:.4f} m'.format(max_error_y))

    # Graficar los datos
    plt.figure(figsize=(10, 6))
    plt.plot(ekf_x, ekf_y, marker='o', linestyle='-', color='royalblue', label="Posicion estimada EKF")
    plt.plot(target_x, target_y, marker='x', linestyle='--', color='r', label="Trayectoria deseada")

    # Añadir las elipses de covarianza cada cierto número de puntos
    step = 50  # Ajustar este valor según sea necesario
    for i in range(0, len(ekf_x), step):
        cov_matrix = np.array([[cov_00[i], cov_01[i]], [cov_10[i], cov_11[i]]])
        plot_covariance_ellipse(ekf_x[i], ekf_y[i], cov_matrix, ekf_x, ekf_y)  # Pasar ekf_x y ekf_y

    # Etiquetas y título del gráfico
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title("Posicion estimada y covarianza")
    plt.legend()
    plt.grid(True)

    # Mostrar el gráfico
    plt.show()

def plot_covariance_ellipse(x, y, cov_matrix, ekf_x, ekf_y):
    # Calcular los valores propios y vectores propios de la matriz de covarianza
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    # Calcular el ángulo de rotación de la elipse
    angle = np.arctan2(*eigenvectors[:, 0][::-1])

    # Calcular el ancho y alto de la elipse
    width, height = 2 * np.sqrt(eigenvalues)

    # Dibujar la elipse
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=np.degrees(angle), edgecolor='orange', fc='None', lw=2, label="Elipse de la covarianza" if x == ekf_x[0] and y == ekf_y[0] else "")
    plt.gca().add_patch(ellipse)

if __name__ == "__main__":
    # Especificar el nombre del archivo manualmente
    file_name = "sinusoid_2.txt"  # Ajusta este nombre según sea necesario

    # Obtener la ruta completa del archivo en la misma carpeta donde está este script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, file_name)

    # Verificar si el archivo existe
    if os.path.exists(file_path):
        plot_ekf_data(file_path)
    else:
        print("File not found: {}. Please check the file path and try again.".format(file_path))


