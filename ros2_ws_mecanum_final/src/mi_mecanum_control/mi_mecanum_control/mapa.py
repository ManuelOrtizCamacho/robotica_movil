"""
mapa.py — Mapa del mundo `pista` para el planificador A* (DADO, no modificar).

El mundo de Gazebo (worlds/pista.sdf) y esta grilla salen de la MISMA lista de
obstáculos, así que lo que ve el LiDAR coincide con lo que planea A*.

Grilla: cuarto de 8 m x 8 m centrado en el origen, celdas de RES = 0.1 m.
Una celda vale 1 si está ocupada (obstáculo inflado con el radio del robot).
"""
import math

RES = 0.10            # [m] tamaño de celda
LADO = 8.0            # [m] el cuarto va de -4 a 4 en x y en y
N = int(round(LADO / RES))   # 80 x 80 celdas
INFLADO = 0.35        # [m] radio del robot + margen: A* trata al robot como un punto

# Obstáculos: (nombre, cx, cy, ancho_x, alto_y)  en metros
OBSTACULOS = [
    ('muro_este',    1.5,  0.5, 0.2, 3.0),
    ('muro_norte',  -1.0,  1.8, 3.0, 0.2),
    ('muro_oeste',  -1.8, -1.0, 0.2, 2.4),
    ('muro_sur',     0.5, -2.0, 2.4, 0.2),
    ('caja_roja',    2.8, -1.4, 0.6, 0.6),
    ('caja_verde',   2.6,  2.6, 0.6, 0.6),
    ('caja_azul',   -2.8,  2.8, 0.6, 0.6),
]
PARED = 0.1           # espesor de las paredes del cuarto


def a_celda(x, y):
    """Metros -> (i, j) de la grilla.  i: columna (x), j: fila (y)."""
    return (int(math.floor((x + LADO / 2) / RES)), int(math.floor((y + LADO / 2) / RES)))


def a_metros(c):
    """(i, j) -> centro de la celda en metros."""
    return ((c[0] + 0.5) * RES - LADO / 2, (c[1] + 0.5) * RES - LADO / 2)


def construir_grilla(inflado=INFLADO):
    """Devuelve grilla[j][i] con 1 = ocupado (inflado), 0 = libre."""
    cajas = [(cx, cy, sx / 2 + inflado, sy / 2 + inflado) for _, cx, cy, sx, sy in OBSTACULOS]
    lim = LADO / 2 - PARED / 2 - inflado          # paredes del cuarto
    grilla = [[0] * N for _ in range(N)]
    for j in range(N):
        for i in range(N):
            x, y = a_metros((i, j))
            if abs(x) > lim or abs(y) > lim:
                grilla[j][i] = 1
                continue
            for cx, cy, hx, hy in cajas:
                if abs(x - cx) <= hx and abs(y - cy) <= hy:
                    grilla[j][i] = 1
                    break
    return grilla


def libre(grilla, c):
    i, j = c
    return 0 <= i < N and 0 <= j < N and grilla[j][i] == 0


def vecinos(grilla, c):
    """8 vecinos libres con su costo de movimiento (1 recto, √2 diagonal).
    No se permite cortar esquinas en diagonal."""
    i, j = c
    out = []
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            n = (i + di, j + dj)
            if not libre(grilla, n):
                continue
            if di != 0 and dj != 0 and not (libre(grilla, (i + di, j)) and libre(grilla, (i, j + dj))):
                continue
            out.append((n, math.sqrt(2.0) if di != 0 and dj != 0 else 1.0))
    return out


def _visible(grilla, a, b):
    """True si el segmento entre los centros de las celdas a y b no toca celdas ocupadas."""
    (xa, ya), (xb, yb) = a_metros(a), a_metros(b)
    pasos = max(1, int(math.hypot(xb - xa, yb - ya) / (RES / 4)))
    for k in range(pasos + 1):
        t = k / pasos
        if not libre(grilla, a_celda(xa + t * (xb - xa), ya + t * (yb - ya))):
            return False
    return True


def simplificar(grilla, camino):
    """Convierte el camino de A* (celda a celda) en pocos waypoints:
    desde cada waypoint salta a la celda más lejana del camino que se vea en
    línea recta sin tocar obstáculos inflados."""
    if len(camino) <= 2:
        return list(camino)
    out = [camino[0]]
    k = 0
    while k < len(camino) - 1:
        m = len(camino) - 1
        while m > k + 1 and not _visible(grilla, camino[k], camino[m]):
            m -= 1
        out.append(camino[m])
        k = m
    return out
