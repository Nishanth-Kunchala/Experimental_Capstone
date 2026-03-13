import numpy as np

def get_vacco8_matrices(mass=1.063, max_thrust=0.035):
    """
    Generates State-Space matrices for the 8-thruster VACCO Palomar CubeSat configuration.
    Uses real-world specifications from the VACCO MicroPropulsionSystems document.
    """
    in2m = 0.0254
    
    # 1. Hardware Geometry (from Envelope Drawing: 3.50" x 4.22")
    # We divide by 2 to get the offset from the center of mass
    dy = (3.50 / 2) * in2m 
    dz = (4.22 / 2) * in2m 
    
    # 2. Inertia Calculation (Using standard CubeSat formulation)
    # Assuming the CubeSat is a standard 3U size given the Palomar MiPS occupies the center
    Ix = (mass / 12) * ((dy*2)**2 + (dz*2)**2)
    Iy = (mass / 12) * ((0.1)**2 + (dz*2)**2) # Approximating 3U length (0.3m) with center module
    Iz = (mass / 12) * ((0.1)**2 + (dy*2)**2)
    print(f"Calculated Inertia (Palomar spec): Ix={Ix:.6f}, Iy={Iy:.6f}, Iz={Iz:.6f}")

    # 3. Thruster Direction Vectors (Purely Orthogonal)
    # No angles. Forces are applied 100% along a single axis.
    f_dirs = [
        np.array([ 0,  1,  0]),  # T1: Pushes +Y (Right Side)
        np.array([ 0, -1,  0]),  # T2: Pushes -Y (Left Side)
        np.array([ 0,  0,  1]),  # T3: Pushes +Z (Top Side)
        np.array([ 0,  0, -1]),  # T4: Pushes -Z (Bottom Side)
        np.array([ 1,  0,  0]),  # T5: Pushes +X (Axial, Top-Right)
        np.array([ 1,  0,  0]),  # T6: Pushes +X (Axial, Bottom-Left)
        np.array([-1,  0,  0]),  # T7: Pushes -X (Axial, Top-Left)
        np.array([-1,  0,  0])   # T8: Pushes -X (Axial, Bottom-Right)
    ]

    # 4. Thruster Position Vectors
    # Placed on the perimeter of the 3.50" x 4.22" envelope
    r_pos = [
        np.array([0,  dy,   0]), # T1: Right edge center
        np.array([0, -dy,   0]), # T2: Left edge center
        np.array([0,   0,  dz]), # T3: Top edge center
        np.array([0,   0, -dz]), # T4: Bottom edge center
        np.array([0,  dy,  dz]), # T5: Top-Right corner
        np.array([0, -dy, -dz]), # T6: Bottom-Left corner
        np.array([0, -dy,  dz]), # T7: Top-Left corner
        np.array([0,  dy, -dz])  # T8: Bottom-Right corner
    ]

    thrusters = [{'pos': r_pos[i], 'dir': f_dirs[i]} for i in range(8)]
    
    # 5. Construct A Matrix (System Dynamics)
    A = np.zeros((12, 12))
    A[0, 3] = A[1, 4] = A[2, 5] = 1   # Linear Kinematics
    A[6, 9] = A[7, 10] = A[8, 11] = 1 # Angular Kinematics

    # 6. Construct B Matrix (Control Inputs)
    B = np.zeros((12, 8)) 
    
    for i, t in enumerate(thrusters):
        r_vec = t['pos']
        f_dir = t['dir']
        
        force_vec = f_dir * max_thrust
        torque_vec = np.cross(r_vec, force_vec)
        
        # Linear Acceleration (Rows 3, 4, 5)
        B[3:6, i] = force_vec / mass
        
        # Angular Acceleration (Rows 9, 10, 11)
        B[9, i]  = torque_vec[0] / Ix
        B[10, i] = torque_vec[1] / Iy
        B[11, i] = torque_vec[2] / Iz

    C = np.eye(12)
    D = np.zeros((12, 8))
    
    return A, B, C, D