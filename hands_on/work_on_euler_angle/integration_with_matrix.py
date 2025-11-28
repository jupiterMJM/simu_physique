import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

def hat(vec):
    """Matrice antisymétrique (hat) d'un vecteur 3D."""
    x, y, z = vec
    return np.array([[0, -z, y],
                     [z, 0, -x],
                     [-y, x, 0]], dtype=float)

def orthonormalize(R):
    """Ré-orthonormalise R (proche d'une rotation) par SVD."""
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    # assurer det = +1 (rotation propre)
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt
    return Rn

def integrate_euler_explicit(R0, omega0, I_body, torque_body, dt, n_steps, renormalize=True):
    """
    Intègre la dynamique rotationnelle par Euler explicite.
    R0        : matrice 3x3 (base locale -> base inertielle) initiale
    omega0    : vecteur 3 (vitesse angulaire exprimée dans la base locale)
    I_body    : matrice d'inertie 3x3 exprimée dans la base locale (constante)
    torque_body: vecteur 3 (constante) ou callable(t) -> vecteur 3, exprimé dans la base locale
    dt        : pas de temps
    n_steps   : nombre de pas d'intégration
    renormalize: si True, ré-orthonormalise R périodiquement
    Retour : (Rs, omegas) listes numpy des tailles (n_steps+1,3,3) et (n_steps+1,3)
    """
    R = np.array(R0, dtype=float).reshape(3,3)
    omega = np.array(omega0, dtype=float).reshape(3)
    I = np.array(I_body, dtype=float).reshape(3,3)
    Iinv = np.linalg.inv(I)

    Rs = np.zeros((n_steps+1, 3, 3), dtype=float)
    omegas = np.zeros((n_steps+1, 3), dtype=float)
    Rs[0] = R
    omegas[0] = omega

    for k in tqdm(range(n_steps)):
        t = k * dt
        # obtenir le couple dans la base locale
        tau = torque_body(t) if callable(torque_body) else np.array(torque_body, dtype=float)

        # équation d'Euler en base locale : I * domega/dt = tau - omega x (I omega)
        Iomega = I @ omega
        omega_dot = Iinv @ (tau - np.cross(omega, Iomega))

        # intégration explicite d'Euler
        omega_new = omega + omega_dot * dt

        # mise à jour de la matrice de rotation (R : body -> inertial)
        # R_dot = R * hat(omega)  => Euler explicite :
        R_new = R + R @ hat(omega) * dt

        # optionnel : ré-orthonormalisation pour limiter la dérive numérique
        if renormalize and ((k+1) % 10 == 0):
            R_new = orthonormalize(R_new)

        # stockage et préparation pour l'itération suivante
        Rs[k+1] = R_new
        omegas[k+1] = omega_new
        R = R_new
        omega = omega_new

    return Rs, omegas

# Exemple d'utilisation simple
if __name__ == "__main__":
    # Inertie diag (exprimée dans la base du drone)
    I_body = np.diag([1, 2, 3])  # [kg m^2], par ex.

    # état initial
    R0 = np.eye(3)                      # base locale alignée initialement
    omega0 = np.array([1, 0.2, 0])  # rad/s exprimé dans la base locale

    # couple constant appliqué dans la base locale
    torque_const = np.array([0.00, 0.0, 0])

    dt = 0.001
    n_steps = 100000

    Rs, omegas = integrate_euler_explicit(R0, omega0, I_body, torque_const, dt, n_steps, renormalize=True)

    # affichage d'un extrait
    print("omega final (body) :", omegas[-1])
    print("R final :\n", Rs[-1])
    # Affichage des composantes de omega au cours du temps
    time = np.arange(n_steps+1) * dt
    plt.figure(figsize=(10,6))
    plt.plot(time, omegas[:,0], label=r'$\omega_1$')
    plt.plot(time, omegas[:,1], label=r'$\omega_2$')
    plt.plot(time, omegas[:,2], label=r'$\omega_3$')
    plt.xlabel('Temps [s]')
    plt.ylabel('Vitesse angulaire [rad/s]')
    plt.title('int_with_matrix: Intégration Euler Explicite - Vitesse Angulaire dans la Base Locale')
    plt.legend()
    plt.grid()
    plt.show()