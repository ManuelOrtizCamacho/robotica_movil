#!/usr/bin/env python3
"""
probar_offline.py — Prueba sus datos, Q, W, J y A* SIN Gazebo (DADO).

    ros2 run mi_mecanum_control probar_offline <carne>

Muestra los valores de su carné, revisa W·Q = I, corre A* y simula el
recorrido con la cinemática ideal: ω = Q ξ,  ξ = W ω,  η̇ = J(φ) ξ (Euler).
"""
import math
import sys

import numpy as np

from mi_mecanum_control.datos_carne import datos, texto
from mi_mecanum_control.mapa import RES, construir_grilla, a_celda, a_metros, simplificar
from mi_mecanum_control.astar import astar
from mi_mecanum_control.cinematica import matriz_Q, matriz_W, integrar
from mi_mecanum_control.control import ley_control


def main():
    if len(sys.argv) < 2:
        print('Uso: ros2 run mi_mecanum_control probar_offline <carne>')
        return
    D = datos(sys.argv[1])
    np.set_printoptions(precision=4, suppress=True)
    print(texto(D))

    a, lx, ly = D['rueda_radio'], D['distancia_ejes'] / 2, D['separacion_ruedas'] / 2
    Q = matriz_Q(a, lx, ly)
    W = matriz_W(Q)
    print(f'\nQ =\n{Q}\nW =\n{W}')
    ok_wq = np.allclose(W @ Q, np.eye(3), atol=1e-9)
    print(f'W·Q = I ? {"SÍ" if ok_wq else "NO"}')

    G = construir_grilla()
    camino, costo = astar(G, a_celda(0, 0), a_celda(*D['meta']))
    if not camino:
        print('\nA*: no encontró camino. Revise astar.py')
        return
    wps = simplificar(G, camino)
    puntos = [(*a_metros(c), D['theta_meta']) for c in wps[1:]]
    puntos[-1] = (*D['meta'], D['theta_meta'])
    print(f'\nA*: {len(camino)} celdas | costo = {costo * RES:.3f} m')
    print('Waypoints:', [(round(p[0], 2), round(p[1], 2)) for p in puntos])

    eta, dt, t, i = np.zeros(3), 0.02, 0.0, 0
    while i < len(puntos) and t < 120:
        u, v, r, ok = ley_control(tuple(eta), puntos[i], i == len(puntos) - 1)
        if ok:
            i += 1
            continue
        omega = Q @ np.array([u, v, r])          # inversa
        eta = integrar(eta, W @ omega, dt)       # directa + odometría
        t += dt
    if i == len(puntos):
        print(f'\nSimulación ideal: meta alcanzada en {t:.1f} s | pose final = '
              f'({eta[0]:.3f}, {eta[1]:.3f}, {math.degrees(eta[2]):.1f}°)')
    else:
        print('\nSimulación ideal: NO llegó a la meta. Revise Q, W y J.')


if __name__ == '__main__':
    main()
