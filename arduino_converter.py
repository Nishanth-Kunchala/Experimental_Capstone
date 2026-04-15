import numpy as np
import os

# --- 1. SYSTEM CONSTANTS & CAD VALUES ---
MASS = 2.784        # kg
MAX_THRUST = 0.025     # N
DT = 0.01              # 100 Hz discrete time step

# Exact Principal Moments of Inertia from CAD (kg*m^2)
CAD_INERTIA = {
    "T1": {"Ix": 0.0487909933, "Iy": 0.0533788243, "Iz": 0.0148340253},
    "T2": {"Ix": 0.0488292954, "Iy": 0.0148342631, "Iz": 0.0533919391},
    "T3": {"Ix": 0.0145439089, "Iy": 0.0491252827, "Iz": 0.0533975722}
}

def get_cubesat_matrices(mass, Ix, Iy, Iz, max_thrust):
    """
    Generates State-Space matrices using HARDCODED thruster positions and CAD inertia.
    """
    # --- Construct A Matrix (System Dynamics) ---
    A = np.zeros((12, 12))
    
    # Linear Kinematics (Position -> Velocity)
    A[0, 3] = 1
    A[1, 4] = 1
    A[2, 5] = 1
    
    # Angular Kinematics (Angle -> Rate)
    A[6, 9] = 1
    A[7, 10] = 1
    A[8, 11] = 1

    # --- Construct B Matrix (Control Inputs) ---
    # Using your exact thruster configuration
    thrusters = [
        {'label': 'A', 'pos': [0.0550, -0.0400, 0.0451],  'dir': [0, 0, -1]},
        {'label': 'B', 'pos': [0.0550, 0.0460, 0.0331],   'dir': [0, -1, 0]},
        {'label': 'C', 'pos': [0.0550, 0.0400, -0.0451],  'dir': [0, 0, 1]},
        {'label': 'D', 'pos': [0.0550, -0.0460, -0.0331], 'dir': [0, 1, 0]},
        {'label': 'E', 'pos': [0.0680, 0.0000, 0.0331],   'dir': [-1, 0, 0]},
        {'label': 'F', 'pos': [0.0680, 0.0000, -0.0331],  'dir': [-1, 0, 0]},
        {'label': 'G', 'pos': [-0.0550, 0.0400, 0.0413],  'dir': [0, 0, -1]},
        {'label': 'H', 'pos': [-0.0532, -0.0500, 0.0331], 'dir': [0, 1, 0]},
        {'label': 'I', 'pos': [-0.0532, -0.0400, -0.0413], 'dir': [0, 0, 1]},
        {'label': 'J', 'pos': [-0.0532, 0.0500, -0.0331], 'dir': [0, -1, 0]},
        {'label': 'K', 'pos': [-0.0582, 0.0000, 0.0331],  'dir': [1, 0, 0]},
        {'label': 'L', 'pos': [-0.0582, 0.0000, -0.0331], 'dir': [1, 0, 0]},
    ]
    
    B = np.zeros((12, 12))
    
    for i, t in enumerate(thrusters):
        r_vec = np.array(t['pos'])       # Position Vector
        f_dir = np.array(t['dir'])       # Direction Unit Vector
        
        # Linear Force Vector (Newtons)
        force_vec = f_dir * max_thrust
        
        # Torque Vector (N*m) = r x F
        torque_vec = np.cross(r_vec, force_vec)
        
        # Linear Acceleration contributions (a = F/m)
        B[3:6, i] = force_vec / mass
        
        # Angular Acceleration contributions (alpha = Torque/I)
        B[9, i]  = torque_vec[0] / Ix
        B[10, i] = torque_vec[1] / Iy
        B[11, i] = torque_vec[2] / Iz

    return A, B

def print_arduino_matrix(name, matrix):
    """Formats and prints a NumPy array as a C++ 2D array."""
    rows, cols = matrix.shape
    print(f"const float {name}[{rows}][{cols}] = {{")
    for row in matrix:
        # Format to 6 decimal places, padded to 14 characters for neat alignment
        formatted_row = ", ".join([f"{val:14.6f}" for val in row])
        print(f"  {{{formatted_row}}},")
    print("};\n")

def main():
    print("// " + "="*60)
    print("// 1. LQR GAIN MATRICES (K)")
    print("// " + "="*60 + "\n")

    # Load and print K matrices from CSVs
    csv_files = {
        "K_T1": "Gain_Matrix_Test1.csv",
        "K_T2": "Gain_Matrix_Test2.csv",
        "K_T3": "Gain_Matrix_Test3.csv"
    }

    for name, filename in csv_files.items():
        if os.path.exists(filename):
            matrix = np.loadtxt(filename, delimiter=',')
            print_arduino_matrix(name, matrix)
        else:
            print(f"// ERROR: '{filename}' not found in current directory.\n")

    print("// " + "="*60)
    print("// 2. DISCRETE INPUT MATRICES (B_d)")
    print("// " + "="*60 + "\n")

    # Generate and print B_d matrices using HARDCODED inertias and exact thruster pos
    for test_name in ["T1", "T2", "T3"]:
        Ix = CAD_INERTIA[test_name]["Ix"]
        Iy = CAD_INERTIA[test_name]["Iy"]
        Iz = CAD_INERTIA[test_name]["Iz"]
        
        # Pass only mass, the 3 exact moments, and max_thrust
        _, B_continuous = get_cubesat_matrices(MASS, Ix, Iy, Iz, MAX_THRUST)
        
        # Discretize the B matrix (B_d = B * DT)
        B_d = B_continuous * DT
        
        print_arduino_matrix(f"B_d_{test_name}", B_d)

if __name__ == "__main__":
    main()