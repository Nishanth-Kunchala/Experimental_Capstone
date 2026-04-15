import numpy as np
import os
import glob

# Define the directory containing the .npz files
# (Update this to the specific folder you want to search through)
folder_path = r"airtablesim/"

# Fetch all .npz files in the specified directory
file_pattern = os.path.join(folder_path, "*.npz")
npz_files = glob.glob(file_pattern)

if not npz_files:
    print(f"No .npz files found in: {folder_path}")

# Iterate through every file found in the folder
for file_path in npz_files:
    filename = os.path.basename(file_path)
    
    # Load data from the current file
    data = np.load(file_path)
    
    try:
        thrust = data["thrust_log"]
        t = data["t_log"]
        state = data["state_log"]
    except KeyError as e:
        print(f"Skipping {filename}: Missing expected data key {e}")
        continue
        
    # --- TRIMMING LOGIC ---
    # Find the index where convergence is reached early
    # (i.e., state doesn't change at all twice in a row)
    trim_idx = len(state)
    for i in range(10, len(state)):
        # Check if current state matches the previous state, AND the previous matches the one before it
        # Note: If you encounter floating-point precision issues in your sim, 
        # swap `np.array_equal(...)` with `np.allclose(state[i], state[i-1], atol=1e-8)`
        if np.array_equal(state[i], state[i-1]) and np.array_equal(state[i-1], state[i-2]) and np.array_equal(state[i-2], state[i-3]) and np.array_equal(state[i-3], state[i-4]):
            trim_idx = i + 1 # Slice up to the point convergence was confirmed
            break
            
    # Trim the arrays
    t_trimmed = t[:trim_idx]
    thrust_trimmed = thrust[:trim_idx]
    state_trimmed = state[:trim_idx]

    # --- CALCULATIONS ---
    # Calculate dt from the time array
    if len(t_trimmed) > 1:
        dt = t_trimmed[1] - t_trimmed[0]
    else:
        dt = 0

    # Calculate Total Impulse (sum of all thrust over the trimmed timeframe)
    tot_impulse = np.sum(thrust_trimmed) * dt
    
    # Total time is the final time step in the trimmed array minus first time step
    total_time = t_trimmed[-1]-t_trimmed[0]
    
    # print(t_trimmed)
    # --- OUTPUT ---
    print(f"--- {filename} ---")
    print(f"Total Time (Convergence): {round(total_time, 3)} s")
    print(f"Total Impulse:            {round(tot_impulse, 3)}")
    print()