"""
datos_carne.py — Valores del examen a partir del carné (DADO, no modificar).

Con los últimos 4 dígitos del carné [d1 d2 d3 d4] se generan la geometría del
robot, la configuración del RPLIDAR A3 y la meta de navegación. Así cada
estudiante tiene un problema distinto.

    ros2 run mi_mecanum_control probar_offline <carne>     # muestra sus datos
"""
import math

# Metas alcanzables (x, y) en metros, validadas con el mapa (mapa.py)
METAS = [(3.0, 0.5), (3.2, -2.8), (-3.0, -3.0), (-3.0, 0.5), (-0.5, 3.0),
         (2.2, -0.5), (1.0, -3.2), (0.5, 2.6), (2.6, 1.0), (-1.0, 3.3)]
THETAS = [0.0, math.pi / 2, math.pi, -math.pi / 2]



def digitos(carne):
    s = ''.join(ch for ch in str(carne) if ch.isdigit())
    if len(s) < 4:
        raise ValueError(f'Carné inválido: "{carne}". Debe tener al menos 4 dígitos.')
    return [int(ch) for ch in s[-4:]]


def datos(carne):
    d1, d2, d3, d4 = digitos(carne)
    modo = 'Enhanced' if d4 % 2 == 0 else 'Outdoor'
    return {
        'carne': str(carne),
        # Geometría del robot (m)
        'rueda_radio': [0.040, 0.045, 0.050, 0.055, 0.060][d1 % 5],
        'distancia_ejes': [0.26, 0.28, 0.30, 0.32, 0.34][d2 % 5],      # wheelbase
        'separacion_ruedas': [0.24, 0.26, 0.28, 0.30, 0.32][d3 % 5],   # track
        # RPLIDAR A3
        'lidar_modo': modo,
        'barrido_hz': [5, 8, 10][(d1 + d2) % 3],
        'lidar_x': [-0.04, 0.0, 0.04][d3 % 3],
        'lidar_z': [0.10, 0.12, 0.14][d2 % 3],
        # Navegación: el robot inicia en (0, 0, 0)
        'meta': METAS[d4],
        'theta_meta': THETAS[(d1 + d3) % 4],
    }


def texto(D):
    return (f"Carné {D['carne']}\n"
            f"  Rueda: radio a = {D['rueda_radio']} m | wheelbase = {D['distancia_ejes']} m"
            f" | track = {D['separacion_ruedas']} m\n"
            f"  RPLIDAR A3: modo {D['lidar_modo']} | barrido {D['barrido_hz']} Hz"
            f" | posición (x, z) = ({D['lidar_x']}, {D['lidar_z']}) m\n"
            f"  Meta: (x, y) = {D['meta']} m | θ = {math.degrees(D['theta_meta']):.0f}°")
