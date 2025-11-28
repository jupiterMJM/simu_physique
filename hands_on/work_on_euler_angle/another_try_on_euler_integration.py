import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ----------------------------
# PARAMÈTRES DU SOLIDE
# ----------------------------
I1, I2, I3 = 1.0, 2.0, 3.0  # tri-axial
w0 = np.array([1.0, 0.2, 0.0])  # vitesse angulaire initiale

# Durée
T = 50.0
N = 10000
t_eval = np.linspace(0, T, N)

# ----------------------------
# Fonction pour les équations d'Euler
# ----------------------------
def euler_rhs(t, w, I1, I2, I3):
    w1, w2, w3 = w
    dw1 = ((I2 - I3)/I1) * w2 * w3
    dw2 = ((I3 - I1)/I2) * w3 * w1
    dw3 = ((I1 - I2)/I3) * w1 * w2
    return [dw1, dw2, dw3]

# ----------------------------
# Intégration numérique
# ----------------------------
sol = solve_ivp(
    fun=lambda t, w: euler_rhs(t, w, I1, I2, I3),
    t_span=(0, T),
    y0=w0,
    t_eval=t_eval,
    method='RK45'
)

w = sol.y  # w[0,:] = ω1(t), w[1,:] = ω2(t), w[2,:] = ω3(t)

# ----------------------------
# Plots des composantes ω
# ----------------------------
plt.figure(figsize=(10,6))
plt.plot(t_eval, w[0], label=r'$\omega_1$')
plt.plot(t_eval, w[1], label=r'$\omega_2$')
plt.plot(t_eval, w[2], label=r'$\omega_3$')
plt.xlabel('Temps [s]')
plt.ylabel('Vitesse angulaire [rad/s]')
plt.title('Another try on euler int: Free Euler Top (tri-axial, M=0)')
plt.legend()
plt.grid()
plt.show()
