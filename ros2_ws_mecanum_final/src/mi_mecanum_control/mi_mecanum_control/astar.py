"""
astar.py — Planificador A* sobre la grilla del mapa.

    f(n) = g(n) + h(n)
    g(n): costo acumulado desde el inicio (en celdas)
    h(n): heurística, distancia estimada de n a la meta (en celdas)

Se usa la grilla 8-conectada de mapa.py: moverse recto cuesta 1 y en
diagonal cuesta √2.
"""
import heapq
import math

from mi_mecanum_control.mapa import vecinos


def heuristica(a, b):
    """Distancia estimada (en celdas) entre las celdas a=(i, j) y b=(i, j)."""
    # TODO (A*-1): distancia euclidiana entre a y b
    return 0.0


def reconstruir_camino(vino_de, actual):
    """Recorre vino_de hacia atrás desde la meta y devuelve [inicio, ..., meta]."""
    # TODO (A*-3): agregar a la lista cada celda anterior usando vino_de
    #              hasta llegar al inicio, y devolver la lista en orden inicio -> meta
    camino = [actual]
    return camino


def astar(grilla, inicio, meta):
    """Devuelve (camino, costo) con camino = lista de celdas y costo en celdas.
    Si no hay camino devuelve ([], inf)."""
    abiertos = [(heuristica(inicio, meta), 0.0, inicio)]   # (f, g, celda)
    g = {inicio: 0.0}
    vino_de = {}
    cerrados = set()

    while abiertos:
        _, g_actual, actual = heapq.heappop(abiertos)
        if actual in cerrados:
            continue
        if actual == meta:
            return reconstruir_camino(vino_de, actual), g_actual
        cerrados.add(actual)

        for vecino, costo_mov in vecinos(grilla, actual):
            if vecino in cerrados:
                continue
            # TODO (A*-2): costo g del vecino pasando por 'actual'.
            #   Si es menor que el que tenía: guardar g[vecino], vino_de[vecino],
            #   calcular f = g + h y agregarlo a 'abiertos' con heapq.heappush
            pass
    return [], math.inf
