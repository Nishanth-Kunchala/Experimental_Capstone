import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from statespace import get_cubesat_matrices

# 1. PARAMETERS
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.50
DT = 0.01
STEPS = int(7.0 / DT)

# PWPF MODULATOR PARAMETERS (Suggested values from the paper)
Km = 4.0      # Filter Gain
Tm = 0.1      # Filter Time Constant
Uon = 0.7     # Turn-on threshold
Uoff = 0.4    # Turn-off threshold
Um = MAX_THRUST # Magnitude of the pulse

# 2. MODEL & LQR INITIALIZATION
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)

Q = np.eye(12) * 100.0  
R = np.eye(B_c.shape[1]) * 1.0
P = solve_discrete_are(A_d, B_d, Q, R)
K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

# 3. PWPF STATE INITIALIZATION
# We need a state for every thruster (B_d.shape[1])
num_thrusters = B_d.shape[1]
X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])
f_states = np.zeros(num_thrusters)  # Internal filter values
u_pwpf = np.zeros(num_thrusters)   # Current on/off state of thrusters

X_hist = []
U_cont_hist = []
U_pwpf_hist = []

# 4. SIMULATION LOOP
for _ in range(STEPS):
    # A. Calculate desired continuous control (u = -Kx)
    u_desired = -K @ X
    
    # B. Apply PWPF modulation to EVERY thruster
    u_next_step = np.zeros(num_thrusters)
    
    for i in range(num_thrusters):
        # 1. Error signal (Desired - Previous Modulated Output)
        e = u_desired[i] - u_pwpf[i]
        
        # 2. Update Filter State (Discrete integration of f' = (Km*e - f) / Tm)
        # This is the "Filter" the paper refers to.
        f_states[i] += (DT / Tm) * (Km * e - f_states[i])
        
        # 3. Schmidt Trigger (Hysteresis Logic)
        if np.abs(f_states[i]) >= Uon:
            u_next_step[i] = Um * np.sign(f_states[i])
        elif np.abs(f_states[i]) <= Uoff:
            u_next_step[i] = 0
        else:
            # Inside the deadband: keep the thruster in its previous state
            u_next_step[i] = u_pwpf[i]
            
    u_pwpf = u_next_step
    
    # C. Apply the pulses to the system dynamics
    X = A_d @ X + B_d @ u_pwpf
    
    X_hist.append(X)
    U_cont_hist.append(u_desired)
    U_pwpf_hist.append(u_pwpf)

'''
4. PLOTTING
'''
X_hist = np.array(X_hist)
U_hist = np.array(U_pwpf_hist)
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