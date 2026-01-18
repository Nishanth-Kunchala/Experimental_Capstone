import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from statespace import get_cubesat_matrices

'''
# 1. SETUP & PARAMETERS
'''
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.50
DT = 0.01
STEPS = int(7.0 / DT)

# Initial State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])

# Get Model
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)

# Discretize (convert to discrete-time for computer control)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12,12))), DT)

'''
2. PURE LQR CALCULATION
'''
# Q: State Cost (Penalize position error 100x more than effort)
# R: Effort Cost (1.0)
Q = np.eye(12) * 100.0  
R = np.eye(12) * 1.0    

# Solve Discrete Algebraic Riccati Equation
P = solve_discrete_are(A_d, B_d, Q, R)

# Compute Gain Matrix: K = (R + B'PB)^-1 (B'PA)
K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

'''
3. SIMULATION LOOP
'''
X_hist = []
U_hist = []

print("Simulating...")

for _ in range(STEPS):
    # 1. Control Law: u = -Kx
    u = -K @ X
    
    # 2. Simple Physical Constraint (Thrusters can't pull, only push 0 to Max)
    u = np.clip(u, 0, MAX_THRUST)
    
    # 3. Dynamics: x[k+1] = Ax[k] + Bu[k]
    X = A_d @ X + B_d @ u
    
    X_hist.append(X)
    U_hist.append(u)

'''
4. PLOTTING
'''
X_hist = np.array(X_hist)
U_hist = np.array(U_hist)
time = np.arange(STEPS) * DT

# Figure 1: Summary
fig1, axs = plt.subplots(2, 2, figsize=(12, 8))
fig1.suptitle("LQR Summary")

axs[0,0].plot(time, X_hist[:,0:3])
axs[0,0].set_title("Position (x, y, z)")
axs[0,0].grid(True)

axs[0,1].plot(time, X_hist[:,6:9] * 180/np.pi)
axs[0,1].set_title("Attitude (deg)")
axs[0,1].grid(True)

axs[1,0].plot(time, np.linalg.norm(U_hist, axis=1), color='purple')
axs[1,0].set_title("Total Thrust Effort (N)")
axs[1,0].grid(True)

axs[1,1].plot(time, np.cumsum(np.linalg.norm(U_hist, axis=1)) * DT, color='green')
axs[1,1].set_title("Fuel Consumption (Ns)")
axs[1,1].grid(True)

plt.tight_layout()

# Figure 2: Separate Thruster Graphs
fig2, axs2 = plt.subplots(4, 3, figsize=(15, 12), sharex=True, sharey=True)
fig2.suptitle("Individual Thruster Firing Commands")

for i, ax in enumerate(axs2.flatten()):
    ax.plot(time, U_hist[:, i], color='tab:red')
    ax.set_title(f"Thruster {i+1}")
    ax.set_ylim(0, MAX_THRUST * 1.1)
    ax.grid(True)

plt.tight_layout()
plt.show()