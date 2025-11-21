import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# ------------------------
# Paramètres du problème
# ------------------------

# quaternion initial (scalar_first=True) : identité
q = R.from_quat([1.0, 0.0, 0.0, 0.0], scalar_first=True)

# vitesse angulaire locale constante : rotation de 2π rad/s autour de z
omega = np.array([1, 1, 10])
historique_angles = []

dt = 0.01
T = 10.0
N = int(T / dt)

# stockage des quaternions
trajectory = np.zeros((N+1, 4))
trajectory[0] = q.as_quat(scalar_first=True)

# ------------------------
# Boucle d'intégration
# ------------------------

for i in range(1, N+1):

    # rotation élémentaire sur dt
    dq = R.from_rotvec(omega * dt)

    # mise à jour de l'orientation :
    # q_new = q * dq  → omega exprimé en base locale
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
plt.show()
