import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from scipy.optimize import differential_evolution
from statespace import get_cubesat_matrices

# ==========================================
# 1. PARAMETERS
# ==========================================
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.025   
DT = 0.01

SIM_TIME = 300.0     
STEPS = int(SIM_TIME / DT)

# PWPF MODULATOR PARAMETERS 
Km = 4.0      
Tm = 0.1      
Uon = 0.7     
Uoff = 0.4    
Um = MAX_THRUST 

X0 = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# ==========================================
# 2. MODEL INITIALIZATION
# ==========================================
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)
num_thrusters = B_d.shape[1]

# ==========================================
# 3. SIMULATION FUNCTION
# ==========================================
def run_simulation(K, return_history=False):
    X = X0.copy()
    
    f_states = np.zeros(num_thrusters)  
    u_pwpf = np.zeros(num_thrusters)    
    
    total_impulse = 0.0
    
    if return_history:
        X_hist = []
        U_pwpf_hist = []

    for step in range(STEPS):
        # 1. Unipolar LQR demand (Negative values become 0)
        u_desired = np.maximum(-K @ X, 0)
        
        # 2. The Hard Deadband Hack
        u_desired[u_desired < 0.005] = 0.0
        
        # 3. PWPF Logic (Unipolar instead of np.sign)
        u_next_step = np.zeros(num_thrusters)
        
        for i in range(num_thrusters):
            e = u_desired[i] - u_pwpf[i]
            f_states[i] += (DT / Tm) * (Km * e - f_states[i])
            
            if f_states[i] >= Uon:
                u_next_step[i] = MAX_THRUST
            elif f_states[i] <= Uoff:
                u_next_step[i] = 0.0
            else:
                u_next_step[i] = u_pwpf[i]
                
        u_pwpf = u_next_step
        total_impulse += np.sum(u_pwpf) * DT
        
        # Apply to system
        X = A_d @ X + B_d @ u_pwpf
        
        if return_history:
            X_hist.append(X)
            U_pwpf_hist.append(u_pwpf.copy())

        # 4. The Early Exit Hack (Stop simulation if target reached)
        if (np.all(np.abs(X[0:3]) <= 0.005) and 
            np.all(np.abs(X[3:6]) <= 0.001) and 
            np.all(np.abs(X[6:9]) <= 5 * np.pi/180) and 
            np.all(np.abs(X[9:12]) <= 1 * np.pi/180)):
            break # Boom! Target reached. Stop burning fuel!

    if return_history:
        return total_impulse, X, np.array(X_hist), np.array(U_pwpf_hist)
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
        
    total_impulse, X_final = run_simulation(K)
    
    # Calculate Final Errors
    pos_err = np.linalg.norm(X_final[0:3])
    vel_err = np.linalg.norm(X_final[3:6])
    att_err = np.linalg.norm(X_final[6:9])
    rate_err = np.linalg.norm(X_final[9:12])
    
    # Strict tolerances 
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
    print("Starting Global LQR Tuning with MATLAB matched physics...")
    print("Using ALL CPU CORES. Please wait...\n")

    # The Sliding Surface Bounds (Allowing astronomical velocity penalties)
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

    total_impulse, X_final, X_hist, U_hist = run_simulation(K, return_history=True)
    
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
    fig1.suptitle(f"LQR Summary (Converged in {simulated_seconds:.2f}s)")

    axs[0,0].plot(time, X_hist[:,0:3])
    axs[0,0].set_title("Position (x, y, z)")
    axs[0,0].grid(True)

    axs[0,1].plot(time, X_hist[:,6:9] * 180/np.pi)
    axs[0,1].set_title("Attitude (deg)")
    axs[0,1].grid(True)

    axs[1,0].plot(time, np.sum(U_hist, axis=1), color='purple')
    axs[1,0].set_title("Total Thrust Effort (N)")
    axs[1,0].grid(True)

    axs[1,1].plot(time, np.cumsum(np.sum(U_hist, axis=1)) * DT, color='green')
    axs[1,1].set_title("Fuel Consumption (Ns)")
    axs[1,1].grid(True)

    plt.tight_layout()
    
    # Save graph directly to avoid popup issues
    fig1.savefig("LQR_Matched_Physics_Summary.png", dpi=300)
    print("Saved graph to LQR_Matched_Physics_Summary.png!")
    
    plt.show()