import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from collections import deque
from statespace import get_cubesat_matrices 

# 1. PARAMETERS
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.015  # 15 mN
DT = 0.01
STEPS = int(1000 / DT) 

# --- NEW TIMING PARAMETERS ---
# We no longer enforce a fixed duration, but we need a "Minimum" 
# so we don't clog the queue with microscopic 1-tick pulses.
MIN_PULSE_DURATION = 0.01  
MIN_TICKS = int(MIN_PULSE_DURATION / DT)

# IMPULSE THRESHOLD:
# The "Gatekeeper". You must have at least this much demand to enter the queue.
IMPULSE_THRESHOLD = MAX_THRUST * MIN_PULSE_DURATION * 1.0

# 2. MODEL & LQR INITIALIZATION
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
# Create discrete system
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)

# LQR Gain Calculation
Q = np.eye(12) * 1.0  
R = np.eye(B_c.shape[1]) * 10000.0
P = solve_discrete_are(A_d, B_d, Q, R)
K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

# 3. STATE INITIALIZATION
num_thrusters = B_d.shape[1]
X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])

# --- QUEUE STATE VARIABLES ---
accumulators = np.zeros(num_thrusters) 
firing_queue = deque() # Now stores Tuples: (thruster_index, duration_ticks)
active_thruster = -1   
timer_remaining = 0    

# History Arrays
X_hist = []
U_pwpf_hist = [] 
Queue_len_hist = []

# 4. SIMULATION LOOP
for _ in range(STEPS):
    # A. Calculate desired continuous control
    u_desired = -K @ X
    
    # B. QUEUE LOGIC ------------------------------------------
    
    # 1. Fill the Accumulators
    for i in range(num_thrusters):
        
        # Drain Logic (Clean up negative demand)
        if u_desired[i] <= 0:
            accumulators[i] = 0.0
            continue 

        # Fill Bucket
        accumulators[i] += u_desired[i] * DT
        
        # Cap Logic (Prevent infinite windup)
        # We allow the bucket to hold up to 5x the min pulse.
        # This means a single firing can be between 0.01s and 0.05s long.
        max_bucket = IMPULSE_THRESHOLD * 5.0
        if accumulators[i] > max_bucket:
            accumulators[i] = max_bucket
        
        # 2. Trigger Logic (Variable Duration)
        if accumulators[i] >= IMPULSE_THRESHOLD:
            
            # Check if this thruster is already in the queue or firing
            # We iterate the queue to check if 'i' is already scheduled
            is_queued = any(item[0] == i for item in firing_queue)
            
            if not is_queued and active_thruster != i:
                # CALCULATE DURATION: How many ticks to drain the bucket?
                # Time = Impulse / Force
                needed_time = accumulators[i] / MAX_THRUST
                needed_ticks = int(needed_time / DT)
                
                # Ensure we at least fire for the minimum (floor protection)
                if needed_ticks < MIN_TICKS:
                    needed_ticks = MIN_TICKS
                
                # Add to queue: (Index, Duration)
                firing_queue.append((i, needed_ticks))
                
                # Remove the impulse we just promised to deliver
                # We calculate exact removed impulse to avoid rounding drift
                removed_impulse = needed_ticks * DT * MAX_THRUST
                accumulators[i] -= removed_impulse
                
                # Small cleanup: if tiny remainder, just zero it
                if accumulators[i] < 0: accumulators[i] = 0
    
    # 3. Process the Firing
    u_actual = np.zeros(num_thrusters)
    
    if active_thruster != -1:
        # Currently firing
        u_actual[active_thruster] = MAX_THRUST
        timer_remaining -= 1
        if timer_remaining <= 0:
            active_thruster = -1 # Finished
            
    elif len(firing_queue) > 0:
        # Start next job
        next_job = firing_queue.popleft()
        active_thruster = next_job[0]     # The thruster index
        timer_remaining = next_job[1]     # The calculated duration
        
        u_actual[active_thruster] = MAX_THRUST
        
    else:
        pass

    # C. Apply to system
    X = A_d @ X + B_d @ u_actual
    
    X_hist.append(X)
    U_pwpf_hist.append(u_actual)
    Queue_len_hist.append(len(firing_queue))

'''
5. PLOTTING
'''
X_hist = np.array(X_hist)
U_hist = np.array(U_pwpf_hist)
time = np.arange(STEPS) * DT

# --- FIGURE 1: SUMMARY ---
fig1, axs = plt.subplots(2, 2, figsize=(12, 8))
fig1.suptitle("Summary: Variable-Duration Queue Mode")

axs[0,0].plot(time, X_hist[:,0:3])
axs[0,0].set_title("Position (x, y, z)")
axs[0,0].grid(True)
axs[0,0].set_ylabel("Meters")

axs[0,1].plot(time, X_hist[:,6:9] * 180/np.pi)
axs[0,1].set_title("Attitude (deg)")
axs[0,1].grid(True)
axs[0,1].set_ylabel("Degrees")

# Force Magnitude
axs[1,0].plot(time, np.linalg.norm(U_hist, axis=1), color='purple')
axs[1,0].set_title("Instantaneous Force")
axs[1,0].grid(True)

# Fuel
axs[1,1].plot(time, np.cumsum(np.linalg.norm(U_hist, axis=1)) * DT, color='green')
axs[1,1].set_title("Fuel Consumption (Ns)")
axs[1,1].grid(True)

plt.tight_layout()

# --- FIGURE 2: INDIVIDUAL THRUSTERS ---
fig2, axs2 = plt.subplots(4, 3, figsize=(15, 12), sharex=True, sharey=True)
fig2.suptitle("Variable Duration Pulses (Width varies by demand)")

for i, ax in enumerate(axs2.flatten()):
    ax.plot(time, U_hist[:, i], color='tab:red')
    ax.set_title(f"Thruster {i+1}")
    ax.set_ylim(-0.005, MAX_THRUST * 1.1)
    ax.grid(True)

plt.tight_layout()

# --- FIGURE 3: QUEUE ---
fig3, ax3 = plt.subplots(figsize=(10, 4))
ax3.plot(time, Queue_len_hist, color='orange', linewidth=2)
ax3.set_title("Queue Health")
ax3.set_ylabel("Count")
ax3.set_xlabel("Time (s)")
ax3.grid(True)
ax3.axhline(y=3, color='r', linestyle='--', label='Warning Level') 
ax3.legend()

plt.tight_layout()
plt.show()