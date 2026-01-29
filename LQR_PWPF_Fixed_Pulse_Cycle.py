import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from collections import deque
from statespace import get_cubesat_matrices  # Assuming you have this file locally

# 1. PARAMETERS
MASS = 1.35
DIM = 0.10
MAX_THRUST = 0.50
DT = 0.01
STEPS = int(15 / DT)  # Increased duration to see settling with slower control

# --- QUEUE & PULSE PARAMETERS ---
FIXED_PULSE_DURATION = 0.02       # Duration of one pulse (seconds)
PULSE_TICKS = int(FIXED_PULSE_DURATION / DT)

# IMPULSE THRESHOLD:
# This determines "How full" the requested bucket needs to be to trigger a firing event.
# Lower = More sensitive (better tracking) but higher queue load.
# Higher = Sluggish tracking but safe queue.
IMPULSE_THRESHOLD = MAX_THRUST * FIXED_PULSE_DURATION * 1

# 2. MODEL & LQR INITIALIZATION
A_c, B_c, _, _ = get_cubesat_matrices(MASS, DIM, DIM, DIM, MAX_THRUST)
# Create discrete system
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), DT)

# LQR Gain Calculation
Q = np.eye(12) * 1.0  
R = np.eye(B_c.shape[1]) * 10.0
P = solve_discrete_are(A_d, B_d, Q, R)
K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

# 3. STATE INITIALIZATION
num_thrusters = B_d.shape[1]
X = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])

# --- QUEUE STATE VARIABLES ---
accumulators = np.zeros(num_thrusters)  # Stores accumulated "need" (Force * time)
firing_queue = deque()                  # The line of waiting thrusters
active_thruster = -1                    # -1 = Idle, 0..N = Index of firing thruster
timer_remaining = 0                     # Countdown for current pulse

# History Arrays
X_hist = []
U_pwpf_hist = []  # This will store the actual applied thrust
Queue_len_hist = []

# 4. SIMULATION LOOP
for _ in range(STEPS):
    # A. Calculate desired continuous control (u = -Kx)
    u_desired = -K @ X
    
    # B. QUEUE LOGIC ------------------------------------------
    
    # 1. Fill the Accumulators
    # We integrate the REQUESTED force over time.
    for i in range(num_thrusters):
        
        # LOGIC CHANGE 1: THE DRAIN
        # If LQR is NOT asking for this thruster (value is 0 or negative), 
        # we must dump the bucket. We shouldn't store "positive" force 
        # if the controller is now asking for "negative" force.
        if u_desired[i] <= 0:
            accumulators[i] = 0.0
            continue # Skip to next thruster

        # If we are here, LQR wants this thruster to fire
        accumulators[i] += u_desired[i] * DT
        
        # LOGIC CHANGE 2: THE CAP
        # Don't let the bucket get infinitely full. 
        # Cap it at 2x the threshold (allows for a small buffer, but prevents windup).
        max_bucket = IMPULSE_THRESHOLD * 2.0
        if accumulators[i] > max_bucket:
            accumulators[i] = max_bucket
        
        # 2. Check Thresholds (Trigger Logic) - Unchanged
        if accumulators[i] >= IMPULSE_THRESHOLD:
            # If not already waiting or firing, add to line
            if i not in firing_queue and active_thruster != i:
                firing_queue.append(i)
                # Pay the cost
                accumulators[i] -= (MAX_THRUST * FIXED_PULSE_DURATION)
    
    # 3. Process the Firing (The Traffic Cop)
    u_actual = np.zeros(num_thrusters)
    
    if active_thruster != -1:
        # We are currently firing
        u_actual[active_thruster] = MAX_THRUST
        timer_remaining -= 1
        if timer_remaining <= 0:
            active_thruster = -1 # Pulse finished
            
    elif len(firing_queue) > 0:
        # We are idle, but someone is waiting
        next_in_line = firing_queue.popleft()
        active_thruster = next_in_line
        timer_remaining = PULSE_TICKS
        u_actual[active_thruster] = MAX_THRUST
        
    else:
        # System is completely idle
        pass
    # ---------------------------------------------------------

    # C. Apply to system dynamics
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

# --- FIGURE 1: OLD LQR SUMMARY ---
fig1, axs = plt.subplots(2, 2, figsize=(12, 8))
fig1.suptitle("Summary: Single-Thruster Queue Mode")

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
# Note: In single mode, this should switch between 0 and MAX_THRUST only
axs[1,0].grid(True)
axs[1,0].set_ylabel("Newtons")

axs[1,1].plot(time, np.cumsum(np.linalg.norm(U_hist, axis=1)) * DT, color='green')
axs[1,1].set_title("Fuel Consumption (Ns)")
axs[1,1].grid(True)
axs[1,1].set_ylabel("Impulse (Ns)")

plt.tight_layout()

# --- FIGURE 2: INDIVIDUAL THRUSTERS ---
fig2, axs2 = plt.subplots(4, 3, figsize=(15, 12), sharex=True, sharey=True)
fig2.suptitle("Individual Thruster Firing Commands (Exclusive Firing)")

for i, ax in enumerate(axs2.flatten()):
    ax.plot(time, U_hist[:, i], color='tab:red')
    ax.set_title(f"Thruster {i+1}")
    ax.set_ylim(-0.05, MAX_THRUST * 1.1)
    ax.grid(True)

plt.tight_layout()

# --- FIGURE 3: NEW QUEUE DIAGNOSTICS ---
fig3, ax3 = plt.subplots(figsize=(10, 4))
ax3.plot(time, Queue_len_hist, color='orange', linewidth=2)
ax3.set_title("Queue Health Monitor")
ax3.set_ylabel("Thrusters Waiting in Line")
ax3.set_xlabel("Time (s)")
ax3.grid(True)
ax3.axhline(y=3, color='r', linestyle='--', label='Warning Level') 
# If queue is consistently above ~3-4, the system is saturated
ax3.legend()

plt.tight_layout()
plt.show()