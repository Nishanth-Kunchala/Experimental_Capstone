import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
from scipy.signal import cont2discrete
from scipy.io import loadmat
from vacco_statespace import get_vacco8_matrices

# ==========================================
# 1. CORE PARAMETERS & SETUP
# ==========================================
MASS = 1.35
MAX_THRUST = 0.025
DT = 0.01
N = 10              # MPC Horizon
SIM_TIME = 250.0    # 8T Simulation duration
STEPS = int(SIM_TIME / DT)
maneuvers = ["Rota", "Tran"]

# Load matrices for the 8-thruster system
A_c, B_c, _, _ = get_vacco8_matrices(mass=MASS, max_thrust=MAX_THRUST)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, 8))), DT)

# ==========================================
# 2. DEFINE MPC PROBLEM
# ==========================================
X_cvx = cp.Variable((12, N + 1))
U_cvx = cp.Variable((8, N))
x_init_param = cp.Parameter(12)

# Weights to drive states to zero (Updated to match vacco_comparison.py)
q_weights = [
    # 1. POSITION (The "Spring")
    5000.0, 5000.0, 5000.0,  # X, Y, Z position 

    # 2. LINEAR VELOCITY (The "Damper" - CRANKED UP)
    5000.0, 5000.0, 5000.0,  # Heavily penalize moving too fast to stop overshoot

    # 3. ATTITUDE
    0.0,    100.0, 100.0,  # Roll (ignore), Pitch, Yaw

    # 4. ANGULAR VELOCITY (The "Damper" - CRANKED UP)
    0.0,    500.0, 500.0   # Roll rate (ignore), Pitch rate, Yaw rate
]
Q_mpc = np.diag(q_weights)
R_mpc = np.eye(8) * .5
Q_terminal = Q_mpc * 10.0

cost = 0
constraints = [X_cvx[:, 0] == x_init_param]
for k in range(N):
    cost += cp.quad_form(X_cvx[:, k], Q_mpc) + cp.quad_form(U_cvx[:, k], R_mpc)
    constraints += [X_cvx[:, k+1] == A_d @ X_cvx[:, k] + B_d @ U_cvx[:, k]]
    constraints += [U_cvx[:, k] >= 0, U_cvx[:, k] <= MAX_THRUST]
cost += cp.quad_form(X_cvx[:, N], Q_terminal)
prob = cp.Problem(cp.Minimize(cost), constraints)

def solve_mpc_step(current_x):
    x_init_param.value = current_x
    prob.solve(solver=cp.OSQP, warm_start=True)
    if prob.status != cp.OPTIMAL:
        return np.zeros(8)
    return U_cvx[:, 0].value

# Helper function to pad 12T data to 300s
def extend_baseline(data, target_len, fill_value=0):
    if data.ndim == 1:
        data = data[:, np.newaxis]
    
    curr_len = data.shape[0]
    if curr_len < target_len:
        if fill_value is None:
            # Duplicate the last row (useful for states)
            last_row = data[-1:] 
            padding = np.tile(last_row, (target_len - curr_len, 1))
        else:
            # Fill with a specific value (useful for 0 thrust)
            padding = np.full((target_len - curr_len, data.shape[1]), fill_value)
            
        return np.vstack([data, padding])
    return data[:target_len]

# ==========================================
# 3. RUN SIMULATIONS & PLOT
# ==========================================
for m_type in maneuvers:
    # Plotting Configuration
    MARKER_SPACING = 2400  # Using a multiple of 6 for clean division
    # Calculate a sub-step to place each of the 6 lines at a different spot
    SUB_STEP = MARKER_SPACING // 6 

    markers = ['o', '^', '*'] # Circle (X/Roll), Triangle (Y/Pitch), Star (Z/Yaw)
    colors_8t = ['r', 'g', 'b']
    colors_12t = ['r', 'g', 'b']

    # A. Load Baseline Data (12-T)
    data = loadmat(f"sim_3D_{m_type}_2.mat")

    # print(data.keys())
    # data = np.load(f"sim_3D_{m_type}_1.npz")
    # data = np.load(f"sim_data_mod_{m_type}.npz")
    # t_12_orig = data["t_log"]
    # u_12_orig = data["thrust_log"]
    # x_12_orig = data["state_log"]
    t_12_orig = data["t_log"]
    u_12_orig = data["thrust_log"]
    x_12_orig = data["state_log"]

    if x_12_orig.shape[0] == 12: # If states are rows, flip to columns
        x_12_orig = x_12_orig.T
        
    if u_12_orig.shape[0] < u_12_orig.shape[1]:
        u_12_orig = u_12_orig.T

    # print("time")
    # print(t_12_orig)
    # print("thrust")
    # print(u_12_orig)
    # print("state")
    # print(x_12_orig)

    # B. Run MPC Simulation (8-T) independently
    curr_x_8t = x_12_orig[0].copy() # Start from the same initial state
    # print("copy")
    # print(curr_x_8t)
    x_mpc = [curr_x_8t.copy()]
    u_mpc = []
    
    print(f"Simulating independent 8T MPC for {m_type}...")
    for _ in range(STEPS):
        u = solve_mpc_step(curr_x_8t)
        curr_x_8t = A_d @ curr_x_8t + B_d @ u
        u_mpc.append(u)
        x_mpc.append(curr_x_8t.copy())
    
    u_mpc = np.array(u_mpc)
    x_mpc = np.array(x_mpc)
    
    # Setup synchronized time axes for 300s
    t_axis_full = np.arange(STEPS + 1) * DT
    t_axis_u = t_axis_full[:-1]

    # Extend 12-T baseline to 300s by repeating last values
    x_12_ext = extend_baseline(x_12_orig, STEPS + 1)
    u_12_ext = extend_baseline(u_12_orig, STEPS)

    # C. GENERATE FIGURES

    # Figure 1: Thruster Profiles (Added Legend, No Markers)
    fig_u, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax1.plot(t_axis_u, u_12_ext, alpha=0.7)
    ax1.set_title(f"12-Thruster Baseline Profiles ({m_type})")
    ax1.set_ylabel("Thrust (N)")
    ax1.legend([f"T{i+1}" for i in range(12)], loc='right', fontsize='small', ncol=2)
    
    ax2.plot(t_axis_u, u_mpc)
    ax2.set_title(f"8-Thruster MPC Profiles ({m_type})")
    ax2.set_ylabel("Thrust (N)")
    ax2.set_xlabel("Time (s)")
    ax2.legend([f"T{i+1}" for i in range(8)], loc='right', fontsize='small', ncol=2)
    plt.tight_layout()

    # Figure 2: Position (X, Y, Z)
    fig_pos, ax = plt.subplots(figsize=(10, 6))
    labels = ['X', 'Y', 'Z']

    for i in range(3):
        # 12-T Baseline markers start at slots 0, 1, 2
        ax.plot(t_axis_full, x_12_ext[:, i], color=colors_12t[i], linestyle='--', alpha=0.4,
                marker=markers[i], markevery=(i * SUB_STEP, MARKER_SPACING), 
                label=f"{labels[i]} (12-T)")
        
        # 8-T MPC markers start at slots 3, 4, 5
        ax.plot(t_axis_full, x_mpc[:, i], color=colors_8t[i], linestyle='-',
                marker=markers[i], markevery=((i + 3) * SUB_STEP, MARKER_SPACING),
                label=f"{labels[i]} (8-T MPC)")

    ax.set_title(f"Position Comparison ({m_type}) - Fully Staggered Markers")
    ax.set_ylabel("Position (m)")
    ax.set_ylim([-1,0.5])
    ax.legend(loc='upper right', ncol=2)
    ax.grid(True, linestyle=':')

    # Figure 3: Attitude (Roll, Pitch, Yaw)
    fig_att, ax = plt.subplots(figsize=(10, 6))
    labels = ['Roll', 'Pitch', 'Yaw']
    
    for i in range(3):
        idx = i + 6
        # 12-T Baseline markers start at slots 0, 1, 2
        ax.plot(t_axis_full, np.rad2deg(x_12_ext[:, idx]), color=colors_12t[i], linestyle='--', alpha=0.4,
                marker=markers[i], markevery=(i * SUB_STEP, MARKER_SPACING),
                label=f"{labels[i]} (12-T)")
        
        # 8-T MPC markers start at slots 3, 4, 5
        ax.plot(t_axis_full, np.rad2deg(x_mpc[:, idx]), color=colors_8t[i], linestyle='-',
                marker=markers[i], markevery=((i + 3) * SUB_STEP, MARKER_SPACING),
                label=f"{labels[i]} (8-T MPC)")

    ax.set_title(f"Attitude Comparison ({m_type}) - Fully Staggered Markers")
    ax.set_ylabel("Degrees")
    ax.set_ylim([-50,20])
    ax.legend(loc='upper right', ncol=2)
    ax.grid(True, linestyle=':')

    # Figure 4: Total Impulse
    fig_imp, ax = plt.subplots(figsize=(10, 5))

    imp_12t_trace = np.cumsum(np.sum(u_12_ext, axis=1)) * DT
    imp_8t_trace = np.cumsum(np.sum(u_mpc, axis=1)) * DT
    
    ax.plot(t_axis_u, imp_12t_trace, linestyle='--', label="12-T Cumulative")
    ax.plot(t_axis_u, imp_8t_trace, 'k-', label="8-T Cumulative")
    ax.set_title(f"Total Impulse Consumption ({m_type})")
    ax.set_ylabel("Total Impulse (Ns)")
    ax.set_xlabel("Time (s)")
    ax.legend()
    ax.grid(True)

plt.show()

# Final Summary Print
for m_type in maneuvers:
    data = loadmat(f"sim_3D_{m_type}_2.mat")
    # data = np.load(f"sim_3D_{m_type}_1.npz")
    # data = np.load(f"sim_data_mod_{m_type}.npz")
    imp_12_final = np.sum(data["thrust_log"]) * DT
    print(f"{m_type} Maneuver Baseline (12-T) Impulse: {imp_12_final:.4f} Ns")