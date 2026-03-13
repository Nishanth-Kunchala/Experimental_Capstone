import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from scipy.optimize import differential_evolution
from collections import deque
from statespace import get_cubesat_matrices 

# ==========================================
# 1. PARAMETERS
# ==========================================
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.025  # Matched to your cycle script
DT = 0.01

SIM_TIME = 300.0     
STEPS = int(SIM_TIME / DT)

# --- QUEUE TIMING PARAMETERS ---
MIN_PULSE_DURATION = 0.01  
MIN_TICKS = int(MIN_PULSE_DURATION / DT)
IMPULSE_THRESHOLD = MAX_THRUST * MIN_PULSE_DURATION * 1.0

# Complex initial state from your cycle script
X0 = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# ==========================================
# 2. MODEL INITIALIZATION
# ==========================================
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)
num_thrusters = B_d.shape[1]

# ==========================================
# 3. SIMULATION WRAPPER (Cycling Physics)
# ==========================================
def run_cycling_sim(K, return_history=False):
    X = X0.copy()
    
    accumulators = np.zeros(num_thrusters)
    firing_queue = deque()
    active_thruster = -1
    timer_remaining = 0
    total_impulse = 0.0
    
    if return_history:
        X_hist = []
        U_hist = []

    for step in range(STEPS):
        # Unipolar LQR demand
        u_desired = -K @ X
        
        u_desired[u_desired < 0.005] = 0.0

        # --- Queue Logic ---
        for i in range(num_thrusters):
            if u_desired[i] <= 0:
                accumulators[i] = 0.0
                continue 

            accumulators[i] += u_desired[i] * DT
            
            # Cap Logic
            max_bucket = IMPULSE_THRESHOLD * 5.0
            if accumulators[i] > max_bucket:
                accumulators[i] = max_bucket
            
            # Trigger Logic
            if accumulators[i] >= IMPULSE_THRESHOLD:
                is_queued = any(item[0] == i for item in firing_queue)
                
                if not is_queued and active_thruster != i:
                    needed_time = accumulators[i] / MAX_THRUST
                    needed_ticks = int(needed_time / DT)
                    
                    if needed_ticks < MIN_TICKS:
                        needed_ticks = MIN_TICKS
                    
                    firing_queue.append((i, needed_ticks))
                    
                    removed_impulse = needed_ticks * DT * MAX_THRUST
                    accumulators[i] -= removed_impulse
                    if accumulators[i] < 0: accumulators[i] = 0
        
        # --- Process Firing ---
        u_actual = np.zeros(num_thrusters)
        
        if active_thruster != -1:
            u_actual[active_thruster] = MAX_THRUST
            timer_remaining -= 1
            if timer_remaining <= 0:
                active_thruster = -1
        elif len(firing_queue) > 0:
            next_job = firing_queue.popleft()
            active_thruster = next_job[0]
            timer_remaining = next_job[1]
            u_actual[active_thruster] = MAX_THRUST

        # Apply to system
        X = A_d @ X + B_d @ u_actual
        total_impulse += np.sum(u_actual) * DT
        
        if return_history:
            X_hist.append(X)
            U_hist.append(u_actual.copy())

        # --- Strict Early Exit Hack (Matches original exactly) ---
        if (np.all(np.abs(X[0:3]) <= 0.005) and 
            np.all(np.abs(X[3:6]) <= 0.001) and 
            np.all(np.abs(X[6:9]) <= 5 * np.pi/180) and 
            np.all(np.abs(X[9:12]) <= 1 * np.pi/180)):
            break

    if return_history:
        return total_impulse, X, np.array(X_hist), np.array(U_hist)
    else:
        return total_impulse, X

# ==========================================
# 4. OBJECTIVE (COST) FUNCTION
# ==========================================
def tuning_cost(params):
    q_pos, q_vel, q_att, q_rate, r_thrust = params
    
    Q = np.diag([q_pos]*3 + [q_vel]*3 + [q_att]*3 + [q_rate]*3)
    R = np.eye(num_thrusters) * r_thrust
    
    try:
        P = solve_discrete_are(A_d, B_d, Q, R)
        K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
    except:
        return 1e9 
        
    total_impulse, X_final = run_cycling_sim(K)
    
    # Calculate Final Errors
    pos_err = np.linalg.norm(X_final[0:3])
    vel_err = np.linalg.norm(X_final[3:6])
    att_err = np.linalg.norm(X_final[6:9])
    rate_err = np.linalg.norm(X_final[9:12])
    
    # Strict tolerances from original code
    TOL_POS = 0.005   
    TOL_VEL = 0.001   
    TOL_ATT = 5 * np.pi/180  
    TOL_RATE = 1 * np.pi/180
    
    penalty = 0.0
    if pos_err > TOL_POS:   penalty += (pos_err - TOL_POS) * 100000.0
    if vel_err > TOL_VEL:   penalty += (vel_err - TOL_VEL) * 100000.0
    if att_err > TOL_ATT:   penalty += (att_err - TOL_ATT) * 100000.0
    if rate_err > TOL_RATE: penalty += (rate_err - TOL_RATE) * 100000.0
    
    return total_impulse + penalty

# ==========================================
# 5. TUNING LOOP (Differential Evolution)
# ==========================================
if __name__ == '__main__':
    print("Starting Global LQR Tuning for Cycling PWPF Queue...")
    print("Using ALL CPU CORES. Please wait...\n")

    bounds = (
        (10.0, 10000.0),      # q_pos
        (100.0, 500000.0),    # q_vel
        (10.0, 10000.0),      # q_att
        (100.0, 500000.0),    # q_rate
        (0.01, 100.0)         # r_thrust
    )

    result = differential_evolution(
        tuning_cost, 
        bounds=bounds,
        strategy='best1bin', 
        maxiter=100,         
        popsize=8,            
        tol=0.05,            
        disp=True,
        workers=-1,          
        updating='deferred'  
    )

    best_q_pos, best_q_vel, best_q_att, best_q_rate, best_r_thrust = result.x

    print("\n--- OPTIMIZED LQR MATRICES WEIGHTS ---")
    print(f"Q Position:    {best_q_pos:.4f}")
    print(f"Q Velocity:    {best_q_vel:.4f}")
    print(f"Q Attitude:    {best_q_att:.4f}")
    print(f"Q Ang. Rate:   {best_q_rate:.4f}")
    print(f"R Thruster:    {best_r_thrust:.4f}")

    # ==========================================
    # 6. FINAL SIMULATION & PLOTS
    # ==========================================
    Q = np.diag([best_q_pos]*3 + [best_q_vel]*3 + [best_q_att]*3 + [best_q_rate]*3)
    R = np.eye(num_thrusters) * best_r_thrust
    P = solve_discrete_are(A_d, B_d, Q, R)
    K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

    total_impulse, X_final, X_hist, U_hist = run_cycling_sim(K, return_history=True)
    
    simulated_seconds = len(X_hist) * DT

    print("\n--- SIMULATION PERFORMANCE METRICS ---")
    print(f"Convergence Time:   {simulated_seconds:.2f} seconds")
    print(f"Total Impulse:      {total_impulse:.4f} Ns")
    print(f"Final Pos Error:    {np.linalg.norm(X_final[0:3]):.4f} m")
    print(f"Final Trans Vel:    {np.linalg.norm(X_final[3:6]):.4f} m/s")
    print(f"Final Att Error:    {np.linalg.norm(X_final[6:9]) * 180/np.pi:.4f} deg")
    print(f"Final Ang Vel:      {np.linalg.norm(X_final[9:12]) * 180/np.pi:.4f} deg/s")
    print("--------------------------------------\n")

    # Dynamic time array based on early exit
    time = np.arange(len(X_hist)) * DT

    fig1, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig1.suptitle(f"Queue Mode Summary (Converged in {simulated_seconds:.2f}s)")

    axs[0,0].plot(time, X_hist[:,0:3])
    axs[0,0].set_title("Position (x, y, z)")
    axs[0,0].grid(True)
    axs[0,0].set_ylabel("Meters")

    axs[0,1].plot(time, X_hist[:,6:9] * 180/np.pi)
    axs[0,1].set_title("Attitude (deg)")
    axs[0,1].grid(True)
    axs[0,1].set_ylabel("Degrees")

    axs[1,0].plot(time, np.linalg.norm(U_hist, axis=1), color='purple')
    axs[1,0].set_title("Instantaneous Force Magnitude")
    axs[1,0].grid(True)

    axs[1,1].plot(time, np.cumsum(np.sum(U_hist, axis=1)) * DT, color='green')
    axs[1,1].set_title("Total Fuel Consumption (Ns)")
    axs[1,1].grid(True)

    plt.tight_layout()
    fig1.savefig("LQR_Cycling_Tuning_Summary.png", dpi=300)
    print("Saved graph to LQR_Cycling_Tuning_Summary.png!")

    fig2, axs2 = plt.subplots(4, 3, figsize=(15, 12), sharex=True, sharey=True)
    fig2.suptitle("Variable Duration Pulses (Queue Saturation View)")
    
    for i, ax in enumerate(axs2.flatten()):
        if i < num_thrusters: # Safety check depending on B_d shape
            ax.plot(time, U_hist[:, i], color='tab:red')
            ax.set_title(f"Thruster {i+1}")
            ax.set_ylim(-0.005, MAX_THRUST * 1.1)
            ax.grid(True)
    
    plt.tight_layout()
    fig2.savefig("LQR_Cycling_Thrusters.png", dpi=300)
    print("Saved graph to LQR_Cycling_Thrusters.png!")
    
    plt.show()