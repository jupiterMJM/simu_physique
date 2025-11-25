import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# ------------------------
# Paramètres du problème
# ------------------------

# quaternion initial (scalar_first=True) : identité
q = R.from_quat([1.0, 0.0, 0.0, 0.0], scalar_first=True)


historique_angles = []

dt = 0.01
T = 50.0
N = int(T / dt)

# stockage des quaternions
trajectory = np.zeros((N+1, 4))
trajectory[0] = q.as_quat(scalar_first=True)

# ------------------------
# Boucle d'intégration avec moment constant dans la base locale
# ------------------------

# I = np.eye(3)  # matrice d'inertie (exemple : sphère)
I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 5]])
omega = np.array([0.0, 0.0, .1])  # vitesse angulaire initiale
torque_local = np.array([1, 0.0, 0.0])  # moment constant dans la base locale

q = R.from_quat([1, 0, 0, 0])  # quaternion initial
trajectory = np.zeros((N+1, 4))
trajectory[0] = q.as_quat(scalar_first=True)
historique_angles = [q.as_euler('ZYX')]

for i in range(1, N):
    # Calcul du moment dans la base globale
    torque_global = q.apply(torque_local)
    # Mise à jour de la vitesse angulaire (Euler direct)
    omega += np.linalg.inv(I) @ torque_global * dt

    # rotation élémentaire sur dt
    dq = R.from_rotvec(omega * dt)

    # mise à jour de l'orientation :
    q = q * dq

    # stockage
    trajectory[i] = q.as_quat(scalar_first=True)
    historique_angles.append(q.as_euler('ZYX'))

# Affichage du dernier quaternion
print("Quaternion final :", trajectory[-1])
print(historique_angles)

# ------------------------
# Visualisation des angles d'Euler
historique_angles = np.array(historique_angles)
plt.figure()
plt.plot(np.arange(N)*dt, historique_angles[:, 0], label='Yaw (Z)')
plt.plot(np.arange(N)*dt, historique_angles[:, 1], label='Pitch (Y)')
plt.plot(np.arange(N)*dt, historique_angles[:, 2], label='Roll (X)')
plt.xlabel('Temps (s)')
plt.ylabel('Angle (rad)')
plt.title("Angles d'Euler au cours du temps")
plt.legend()
plt.grid()

# Calcul des dérivées des angles d'Euler
derivatives = np.gradient(np.unwrap(historique_angles, axis=0), dt, axis=0)

# Visualisation des dérivées des angles d'Euler
plt.figure()
plt.plot(np.arange(N)*dt, derivatives[:, 0], label="d(Yaw)/dt (Z)")
plt.plot(np.arange(N)*dt, derivatives[:, 1], label="d(Pitch)/dt (Y)")
plt.plot(np.arange(N)*dt, derivatives[:, 2], label="d(Roll)/dt (X)")
plt.xlabel('Temps (s)')
plt.ylabel('Vitesse angulaire (rad/s)')
plt.title("Dérivées des angles d'Euler au cours du temps")
plt.legend()
plt.grid()

plt.show()
