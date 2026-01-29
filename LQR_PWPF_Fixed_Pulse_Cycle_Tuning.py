import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from collections import deque
from statespace import get_cubesat_matrices

# --- 1. CONFIGURATION ---
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.50
DT = 0.01
SIM_DURATION = 100.0 
STEPS = int(SIM_DURATION / DT)

# SEARCH RANGES
# 1. Q Scales (Aggressiveness)
Q_SCALES = [1, 10, 50, 100, 200]
# 2. R Scales (Damping/Efficiency)
R_SCALES = [10, 50, 100, 200, 500, 1000]
# 3. Pulse Durations (The new dimension)
# Testing from 10ms (super fine) to 100ms (coarse)
PULSE_DURATIONS = [0.01, 0.02, 0.05, 0.08, 0.10] 

# --- 2. SETUP ---
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)
num_thrusters = B_d.shape[1]

def run_simulation(q_mult, r_mult, pulse_duration):
    """
    Runs sim with dynamic Q, R, and Pulse Duration.
    """
    # Recalculate Logic parameters based on this specific Pulse Duration
    impulse_threshold = MAX_THRUST * pulse_duration * 0.5 
    pulse_ticks = int(pulse_duration / DT)
    
    # A. Design LQR
    Q = np.eye(12) * q_mult
    R = np.eye(num_thrusters) * r_mult
    try:
        P = solve_discrete_are(A_d, B_d, Q, R)
        K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
    except:
        return 9999.0

    # B. Initialize
    X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])
    accumulators = np.zeros(num_thrusters)
    firing_queue = deque()
    active_thruster = -1
    timer_remaining = 0
    
    # C. Run Loop
    for _ in range(STEPS):
        u_desired = -K @ X
        
        # Deadzone logic (scaled to max thrust)
        if np.max(np.abs(u_desired)) < (MAX_THRUST * 0.01):
             pass # Skip accumulation for tiny noise
        else:
            for i in range(num_thrusters):
                if u_desired[i] <= 0:
                    accumulators[i] = 0.0 # Drain
                    continue
                
                accumulators[i] += u_desired[i] * DT
                if accumulators[i] > impulse_threshold * 2: # Cap
                    accumulators[i] = impulse_threshold * 2
                
                if accumulators[i] >= impulse_threshold:
                    if i not in firing_queue and active_thruster != i:
                        firing_queue.append(i)
                        accumulators[i] -= (MAX_THRUST * pulse_duration)

        # Apply Firing
        u_actual = np.zeros(num_thrusters)
        if active_thruster != -1:
            u_actual[active_thruster] = MAX_THRUST
            timer_remaining -= 1
            if timer_remaining <= 0: active_thruster = -1
        elif len(firing_queue) > 0:
            active_thruster = firing_queue.popleft()
            timer_remaining = pulse_ticks
            u_actual[active_thruster] = MAX_THRUST

        # Dynamics
        X = A_d @ X + B_d @ u_actual
        if np.linalg.norm(X) > 50.0: return 1000.0 # Crash penalty

    return np.linalg.norm(X) # Return final distance from target

# --- 3. EXECUTE 3D SEARCH ---
best_score = 9999.0
best_params = (0, 0, 0)
results_log = []

print(f"Testing {len(Q_SCALES)*len(R_SCALES)*len(PULSE_DURATIONS)} combinations...")

# To visualize later, we will store the 2D matrix of the BEST pulse found
final_matrix = np.zeros((len(Q_SCALES), len(R_SCALES)))

for p_idx, p_val in enumerate(PULSE_DURATIONS):
    print(f"--- Testing Pulse Duration: {p_val}s ---")
    current_matrix = np.zeros((len(Q_SCALES), len(R_SCALES)))
    
    for q_idx, q_val in enumerate(Q_SCALES):
        for r_idx, r_val in enumerate(R_SCALES):
            score = run_simulation(q_val, r_val, p_val)
            current_matrix[q_idx, r_idx] = score
            
            # Check for global best
            if score < best_score:
                best_score = score
                best_params = (q_val, r_val, p_val)
                print(f"  New Best! Error={score:.4f} [Q={q_val}, R={r_val}, P={p_val}]")
    
    # If this pulse duration contained the global best, save its matrix for plotting
    if best_params[2] == p_val:
        final_matrix = current_matrix

print(f"\nGLOBAL OPTIMUM FOUND:")
print(f"Pulse Duration: {best_params[2]} s")
print(f"Q Multiplier:   {best_params[0]}")
print(f"R Multiplier:   {best_params[1]}")
print(f"Final Error:    {best_score:.5f}")

# --- 4. PLOT THE WINNING SLICE ---
plt.figure(figsize=(10, 8))
plt.imshow(final_matrix, cmap='viridis_r', interpolation='nearest', aspect='auto')
plt.colorbar(label=f'Final Error (Fixed Pulse={best_params[2]}s)')
plt.xticks(ticks=np.arange(len(R_SCALES)), labels=R_SCALES)
plt.yticks(ticks=np.arange(len(Q_SCALES)), labels=Q_SCALES)
plt.xlabel("R Multiplier")
plt.ylabel("Q Multiplier")
plt.title(f"Optimization Landscape\n(At Optimal Pulse Duration: {best_params[2]}s)")
plt.show()