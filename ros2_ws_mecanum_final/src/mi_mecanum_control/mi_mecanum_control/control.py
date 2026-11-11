"""
control.py — Ley de control a puntos de la clase, ec. (21) con η̇d = 0 (DADO).

    ξ = J(φ)⁺ Λ (ηd − η),   J(φ)⁺ = J(φ)ᵀ  (rotación)

El robot es holonómico: va a (x, y) y a θ al mismo tiempo, sin girar en el sitio.
"""
import math

L1, L2, L3 = 1.0, 1.0, 1.5     # Λ = diag(λ1, λ2, λ3)
V_MAX = 0.4                    # [m/s]   saturación de la norma de (u, v)
R_MAX = 1.0                    # [rad/s]
TOL_PASO = 0.10                # [m]     tolerancia en waypoints intermedios
TOL_POS = 0.05                 # [m]     tolerancia en la meta
TOL_ANG = 0.05                 # [rad]


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def ley_control(pose, meta, final=True):
    """pose = (x, y, φ), meta = (xd, yd, θd). Devuelve (u, v, r, alcanzado)."""
    x, y, phi = pose
    xd, yd, thd = meta
    ex, ey = xd - x, yd - y
    e_phi = envolver(thd - phi)

    if final:
        if math.hypot(ex, ey) < TOL_POS and abs(e_phi) < TOL_ANG:
            return 0.0, 0.0, 0.0, True
    elif math.hypot(ex, ey) < TOL_PASO:
        return 0.0, 0.0, 0.0, True

    c, s = math.cos(phi), math.sin(phi)
    u = c * L1 * ex + s * L2 * ey
    v = -s * L1 * ex + c * L2 * ey
    r = L3 * e_phi

    norma = math.hypot(u, v)
    if norma > V_MAX:
        u, v = u * V_MAX / norma, v * V_MAX / norma
    r = max(-R_MAX, min(R_MAX, r))
    return u, v, r, False
