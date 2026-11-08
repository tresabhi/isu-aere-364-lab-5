import numpy as np
import control as ct
import matplotlib.pyplot as plt
from scipy.optimize import minimize

s = ct.tf('s')

# Plant: replace with your actual model
K, tau = 10.0, 0.1
G = K / (s * (tau * s + 1))

t = np.linspace(0, 5, 500)

def cost(gains):
    KP, KI, KD = gains
    C = KP/np.pi + (KI/np.pi)/s + KD*s
    T = ct.feedback(G * C, 1)
    _, y = ct.step_response(T, t)
    if not np.all(np.isfinite(y)):
        return 1e6                      # unstable or blew up
    return np.sum(np.abs(1 - y)) * (t[1] - t[0])

res = minimize(cost, [1.0, 0.1, 0.2], bounds=[(0, 20)]*3, method='Nelder-Mead')
KP, KI, KD = np.clip(res.x, 0, 20)
print(f"KP={KP:.3f}  KI={KI:.3f}  KD={KD:.3f}  cost={res.fun:.4f}")

# Step response with the optimized gains
C = KP/np.pi + (KI/np.pi)/s + KD*s
T = ct.feedback(G * C, 1)
_, y = ct.step_response(T, t)
plt.figure()
plt.plot(t, y)
plt.axhline(1, color='gray', linestyle='--')
plt.title("Closed-loop step response")

# Root locus with the optimized gains
plt.figure()
ct.root_locus(G * C)
plt.show()