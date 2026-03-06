import numpy as np

import numpy as np

def compute_pwpf_lqr_thrust(x, K, f_states, u_pwpf, dt=0.01, Km=4.0, Tm=0.1, Uon=0.7, Uoff=0.4, Um=0.50):
    """
    Computes discrete thruster commands using LQR and PWPF modulation.
    
    Args:
        x: Current state vector (1D numpy array)
        K: LQR Gain matrix (2D numpy array)
        f_states: Current internal PWPF filter states (1D numpy array)
        u_pwpf: Previous step's thruster commands (1D numpy array)
        dt, Km, Tm, Uon, Uoff, Um: PWPF tuning parameters.
        
    Returns:
        u_next_step: New thruster commands to apply in the simulation this step.
        f_states: Updated PWPF filter states (to pass into the next step).
    """
    # 1. Calculate desired continuous control effort (u = -K * x)
    u_desired = -K @ x

    # 2. Apply PWPF Modulation
    num_thrusters = len(u_pwpf)
    u_next_step = np.zeros(num_thrusters)

    for i in range(num_thrusters):
        # Error between desired continuous thrust and actual applied discrete thrust
        e = u_desired[i] - u_pwpf[i]
        
        # Discrete integration for the filter state
        f_states[i] += (dt / Tm) * (Km * e - f_states[i])

        # Schmidt Trigger (Hysteresis logic)
        if np.abs(f_states[i]) >= Uon:
            u_next_step[i] = Um * np.sign(f_states[i])
        elif np.abs(f_states[i]) <= Uoff:
            u_next_step[i] = 0
        else:
            # Inside the deadband: keep the thruster in its previous state
            u_next_step[i] = u_pwpf[i]

    return u_next_step, f_states

# ==========================================
# 1. INITIALIZATION (Run once before the loop)
# ==========================================
NUM_STATES = 12
NUM_THRUSTERS = 12

# the pre-computed LQR Gain Matrix
K = np.array([
    [ 0.000000,  0.000000, -4.795644, -0.000000,  0.000000, -5.428841,  1.429911,  1.359754, -0.000000,  1.446508,  1.374915,  0.000000],
    [ 0.000000, -4.795644,  0.000000, -0.000000, -5.428841,  0.000000,  1.183251, -0.000000, -1.821959,  1.196985, -0.000000, -1.842984],
    [-0.000000, -0.000000,  4.795644, -0.000000, -0.000000,  5.428841,  1.429911, -1.359754, -0.000000,  1.446508, -1.374915,  0.000000],
    [-0.000000,  4.795644, -0.000000,  0.000000,  5.428841, -0.000000,  1.183251,  0.000000,  1.821959,  1.196985,  0.000000,  1.842984],
    [-4.795644,  0.000000,  0.000000, -5.428841, -0.000000,  0.000000,  0.000000, -0.846012, -0.000000,  0.000000, -0.855445, -0.000000],
    [-4.795644,  0.000000,  0.000000, -5.428841, -0.000000, -0.000000, -0.000000,  0.846012,  0.000000,  0.000000,  0.855445,  0.000000],
    [ 0.000000,  0.000000, -4.795644,  0.000000,  0.000000, -5.428841, -1.429911, -1.359754, -0.000000, -1.446508, -1.374915, -0.000000],
    [-0.000000,  4.795644, -0.000000,  0.000000,  5.428841, -0.000000, -1.183251, -0.000000, -1.821959, -1.196985, -0.000000, -1.842984],
    [-0.000000, -0.000000,  4.795644, -0.000000, -0.000000,  5.428841, -1.429911,  1.359754,  0.000000, -1.446508,  1.374915,  0.000000],
    [ 0.000000, -4.795644,  0.000000, -0.000000, -5.428841,  0.000000, -1.183251,  0.000000,  1.821959, -1.196985,  0.000000,  1.842984],
    [ 4.795644, -0.000000, -0.000000,  5.428841,  0.000000, -0.000000, -0.000000,  0.846012,  0.000000,  0.000000,  0.855445,  0.000000],
    [ 4.795644, -0.000000, -0.000000,  5.428841,  0.000000, -0.000000,  0.000000, -0.846012, -0.000000, -0.000000, -0.855445, -0.000000]
])

# The variables that must persist between time steps
f_states = np.zeros(NUM_THRUSTERS)
u_pwpf = np.zeros(NUM_THRUSTERS)

# ==========================================
# 2. SIMULATION LOOP (Runs continuously)
# ==========================================
sim_running = True
step = 0

while sim_running:
    # A. Get the current state from Isaac Sim (Position, Velocity, Attitude, Rates)
    # x = get_state_from_isaac_articulation()
    
    # Dummy state for the example
    x = np.array([1.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.75, -0.75, 0.75, 0.0, 0.0, 0.0])
    
    # B. Compute the thruster commands
    u_next, f_states = compute_pwpf_lqr_thrust(
        x=x, 
        K=K, 
        f_states=f_states, 
        u_pwpf=u_pwpf,
        dt=0.01 # Make sure this matches the Isaac Sim physics step
    )
    
    # C. Update the previous thruster state for the next loop iteration
    u_pwpf = u_next 
    
    # D. Apply the forces to the Isaac Sim robot
    # apply_forces_to_thrusters(u_next)
    
    print(f"Step {step} commands:", u_next)
    
    # Safety break for the example loop
    step += 1
    if step > 5:
        sim_running = False