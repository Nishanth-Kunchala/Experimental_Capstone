import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt 
from scipy.signal import cont2discrete
from vacco_statespace import get_vacco8_matrices 

# ==========================================
# 1. PARAMETERS
# ==========================================
MASS = 1.35
MAX_THRUST = 0.025
DT = 0.01
STEPS = int(300.0 / DT) # 2. Increased to 5 Min to allow for full travel

# 3. Generate and discretize matrices
A_c, B_c, _, _ = get_vacco8_matrices()
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)

num_states = 12
num_thrusters = B_d.shape[1] # Will dynamically read 8 thrusters

# ==========================================
# 2. MPC PROBLEM SETUP 
# ==========================================
N = 10  # 4. Increased Prediction Horizon to 0.1s

X_cvx = cp.Variable((num_states, N + 1))
U_cvx = cp.Variable((num_thrusters, N))
x_init = cp.Parameter(num_states)

# 5. Custom Cost Weights prioritizing position heavily over attitude
q_weights = [
    # 1. POSITION (The "Spring")
    1000.0, 1000.0, 1000.0,  # X, Y, Z position 

    # 2. LINEAR VELOCITY (The "Damper" - CRANKED UP)
    5000.0, 5000.0, 5000.0,  # Heavily penalize moving too fast to stop overshoot

    # 3. ATTITUDE
    1000.0,    100.0, 100.0,  # Roll (ignore), Pitch, Yaw

    # 4. ANGULAR VELOCITY (The "Damper" - CRANKED UP)
    5000.0,    500.0, 500.0   # Roll rate (ignore), Pitch rate, Yaw rate
]
Q_mpc = np.diag(q_weights)
Q_mpc = np.diag(q_weights)
R_mpc = np.eye(num_thrusters) * 1.0 
Q_terminal = Q_mpc * 10.0 # Terminal cost

cost = 0
constraints = [X_cvx[:, 0] == x_init] 

for k in range(N):
    cost += cp.quad_form(X_cvx[:, k], Q_mpc) + cp.quad_form(U_cvx[:, k], R_mpc)
    constraints += [X_cvx[:, k+1] == A_d @ X_cvx[:, k] + B_d @ U_cvx[:, k]]
    constraints += [U_cvx[:, k] >= 0]               
    constraints += [U_cvx[:, k] <= MAX_THRUST]      

cost += cp.quad_form(X_cvx[:, N], Q_terminal)
prob = cp.Problem(cp.Minimize(cost), constraints)

# ==========================================
# 3. STATE INITIALIZATION 
# ==========================================
X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])

X_hist = []
U_hist = []

# ==========================================
# 4. SIMULATION LOOP
# ==========================================
for _ in range(STEPS):
    x_init.value = X
    prob.solve(solver=cp.OSQP, warm_start=True)
    
    if prob.status != cp.OPTIMAL:
        print("Solver failed to find an optimal solution!")
        break
        
    u_cmd = U_cvx[:, 0].value
    X = A_d @ X + B_d @ u_cmd
    
    X_hist.append(X)
    U_hist.append(u_cmd)

# ==========================================
# 5. PLOTTING
# ==========================================
X_hist = np.array(X_hist)
U_hist = np.array(U_hist) # 6. Fixed variable name reference
time = np.arange(STEPS) * DT

# Figure 1: Summary
fig1, axs = plt.subplots(2, 2, figsize=(12, 8))
fig1.suptitle("MPC Summary (VACCO 8-Thruster)")

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
# Changed to 4x2 grid to fit 8 thrusters cleanly
fig2, axs2 = plt.subplots(4, 2, figsize=(15, 12), sharex=True, sharey=True)
fig2.suptitle("Individual Thruster Firing Commands (8-T)")

for i, ax in enumerate(axs2.flatten()):
    if i < 8:
        ax.plot(time, U_hist[:, i], color='tab:red')
        ax.set_title(f"Thruster {i+1}")
        ax.set_ylim(-0.005, MAX_THRUST * 1.1)
        ax.grid(True)
    else:
        ax.axis('off')

plt.tight_layout()
plt.show()