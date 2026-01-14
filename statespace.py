import numpy as np

def get_cubesat_matrices(mass, width, depth, height, max_thrust):
    """
    Generates State-Space matrices (A, B, C, D) for the specific 12-thruster CubeSat.
    
    State Vector Order (12x1):
    [x, y, z,          (Position)
     dx, dy, dz,       (Linear Velocity)
     phi, theta, psi,  (Orientation - Euler Angles)
     p, q, r]          (Angular Velocity)
     
    Control Vector Order (12x1):
    [F_A, F_B, ..., F_L] (Normalized 0.0 to 1.0)
    """
    
    # --- 1. Physics Constants ---
    # Calculate Principal Moments of Inertia (Solid Rectangular Block assumption)
    # Ix (Rotation about X): Uses Y and Z dimensions
    Ix = (1/12) * mass * (depth**2 + height**2)
    # Iy (Rotation about Y): Uses X and Z dimensions
    Iy = (1/12) * mass * (width**2 + height**2)
    # Iz (Rotation about Z): Uses X and Y dimensions
    Iz = (1/12) * mass * (width**2 + depth**2)
    
    print(f"Calculated Inertia: Ix={Ix:.6f}, Iy={Iy:.6f}, Iz={Iz:.6f}")

    # --- 2. Construct A Matrix (System Dynamics) ---
    # Relationships:
    # dx/dt = dx (Velocity)
    # d(dx)/dt = 0 (Force comes from B matrix)
    A = np.zeros((12, 12))
    
    # Block 1: Linear Kinematics (Position -> Velocity)
    # x' = dx, y' = dy, z' = dz
    A[0, 3] = 1
    A[1, 4] = 1
    A[2, 5] = 1
    
    # Block 2: Angular Kinematics (Angle -> Rate)
    A[6, 9] = 1
    A[7, 10] = 1
    A[8, 11] = 1

    # --- 3. Construct B Matrix (Control Inputs) ---
    # Map 12 Thrusters -> Force/Torque -> Acceleration
    
    # Thruster Configuration
    thrusters = [
        {'label': 'A', 'pos': [0.0532, -0.0400, 0.0413],  'dir': [0, 0, -1]},
        {'label': 'B', 'pos': [0.0532, 0.0500, 0.0331],   'dir': [0, -1, 0]},
        {'label': 'C', 'pos': [0.0532, 0.0400, -0.0413],  'dir': [0, 0, 1]},
        {'label': 'D', 'pos': [0.0532, -0.0500, -0.0331], 'dir': [0, 1, 0]},
        {'label': 'E', 'pos': [0.0582, 0.0000, 0.0331],   'dir': [-1, 0, 0]},
        {'label': 'F', 'pos': [0.0582, 0.0000, -0.0331],  'dir': [-1, 0, 0]},
        {'label': 'G', 'pos': [-0.0532, 0.040, 0.0413],   'dir': [0, 0, -1]},
        {'label': 'H', 'pos': [-0.0532, -0.0500, 0.0331], 'dir': [0, 1, 0]},
        {'label': 'I', 'pos': [-0.0532, -0.0400, -0.0413], 'dir': [0, 0, 1]},
        {'label': 'J', 'pos': [-0.0532, 0.0500, -0.0331], 'dir': [0, -1, 0]},
        {'label': 'K', 'pos': [-0.0582, 0.0000, 0.0331],  'dir': [1, 0, 0]},
        {'label': 'L', 'pos': [-0.0582, 0.000, -0.0331],  'dir': [1, 0, 0]},
    ]
    
    B = np.zeros((12, 12))
    
    # Loop through each thruster to calculate its column in B
    for i, t in enumerate(thrusters):
        r_vec = np.array(t['pos'])       # Position Vector
        f_dir = np.array(t['dir'])       # Direction Unit Vector
        
        # Linear Force Vector (Newtons)
        force_vec = f_dir * max_thrust
        
        # Torque Vector (N*m) = r x F
        torque_vec = np.cross(r_vec, force_vec)
        
        # Linear Acceleration contributions (a = F/m)
        # Map to rows 3, 4, 5 (dx_dot, dy_dot, dz_dot)
        B[3:6, i] = force_vec / mass
        
        # Angular Acceleration contributions (alpha = Torque/I)
        # Map to rows 9, 10, 11 (p_dot, q_dot, r_dot)
        B[9, i]  = torque_vec[0] / Ix
        B[10, i] = torque_vec[1] / Iy
        B[11, i] = torque_vec[2] / Iz

    # --- 4. Construct C and D Matrices (Output) ---
    # C: Output Matrix - We assume we can measure (or estimate) the full state
    C = np.eye(12)
    
    # D: Feedthrough Matrix - No instantaneous effect of thrust on position/velocity
    D = np.zeros((12, 12))
    
    return A, B, C, D

if __name__ == "__main__":
    # 1. Define your physical parameters
    MASS_KG = 1.35        # Example Mass
    WIDTH_M = 0.1164      # X-axis dim (10cm)
    DEPTH_M = 0.1164      # Y-axis dim (10cm)
    HEIGHT_M = 0.1164     # Z-axis dim (10cm)
    THRUST_N = 1.0        # 1.0 Newtons per thruster

    # 2. Generate Matrices
    A, B, C, D = get_cubesat_matrices(MASS_KG, WIDTH_M, DEPTH_M, HEIGHT_M, THRUST_N)

    # 3. Inspect results
    np.set_printoptions(precision=4, suppress=True, linewidth=200)
    print("\n--- B Matrix (Input to Acceleration Mapping) ---")

    # Only printing the rows that matter (Acceleration rows)
    print("Rows 3-5 (Linear Accel x,y,z):")
    print(B[3:6, :]) 
    print("\nRows 9-11 (Angular Accel p,q,r):")
    print(B[9:12, :])