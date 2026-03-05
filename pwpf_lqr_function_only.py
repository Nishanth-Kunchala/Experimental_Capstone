import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from statespace import get_cubesat_matrices

class LQRPWPFController:
    def __init__(self, mass=1.35, dim=0.10, max_thrust=0.50, dt=0.01, 
                 Km=4.0, Tm=0.1, Uon=0.7, Uoff=0.4, Q_weight=100.0, R_weight=1.0):
        """
        Initializes the LQR controller and PWPF modulator states.
        """
        self.dt = dt
        self.max_thrust = max_thrust
        
        # PWPF Parameters
        self.Km = Km
        self.Tm = Tm
        self.Uon = Uon
        self.Uoff = Uoff
        self.Um = max_thrust
        
        # 1. MODEL & LQR INITIALIZATION
        A_c, B_c, _, _ = get_cubesat_matrices(mass, dim, dim, dim, max_thrust)
        self.A_d, self.B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12, B_c.shape[1]))), dt)

        Q = np.eye(12) * Q_weight
        R = np.eye(B_c.shape[1]) * R_weight
        
        # Calculate LQR Gain Matrix K
        P = solve_discrete_are(self.A_d, self.B_d, Q, R)
        self.K = np.linalg.inv(R + self.B_d.T @ P @ self.B_d) @ (self.B_d.T @ P @ self.A_d)
        
        # 2. PWPF STATE INITIALIZATION
        self.num_thrusters = self.B_d.shape[1]
        self.f_states = np.zeros(self.num_thrusters)  # Internal filter values
        self.u_pwpf = np.zeros(self.num_thrusters)    # Current on/off state of thrusters

    def compute_control(self, current_state):
        """
        Computes the discrete thruster commands for a single time step.
        
        Args:
            current_state (np.ndarray): Current state vector (1D array of length 12)
            
        Returns:
            np.ndarray: Thruster firing commands for this time step
        """
        # A. Calculate desired continuous control (u = -Kx)
        u_desired = -self.K @ current_state
        
        # B. Apply PWPF modulation to EVERY thruster
        u_next_step = np.zeros(self.num_thrusters)
        
        for i in range(self.num_thrusters):
            # 1. Error signal (Desired - Previous Modulated Output)
            e = u_desired[i] - self.u_pwpf[i]
            
            # 2. Update Filter State (Discrete integration)
            self.f_states[i] += (self.dt / self.Tm) * (self.Km * e - self.f_states[i])
            
            # 3. Schmidt Trigger (Hysteresis Logic)
            if np.abs(self.f_states[i]) >= self.Uon:
                u_next_step[i] = self.Um * np.sign(self.f_states[i])
            elif np.abs(self.f_states[i]) <= self.Uoff:
                u_next_step[i] = 0
            else:
                # Inside the deadband: keep the thruster in its previous state
                u_next_step[i] = self.u_pwpf[i]
                
        # Update internal state for the next loop iteration
        self.u_pwpf = u_next_step
        
        return self.u_pwpf
    
'''
EXAMPLE USAGE!!!!!!!!!!
THIS IS LIKE HALF PSEUDO CODE BUT IT GIVES YOU AN IDEA OF HOW I SET UP THE CLASS 
AND HOW TO USE IT IN A SIMULATION LOOP. YOU'LL NEED TO FILL IN THE SIMULATION INTERFACING PARTS.
'''

# 1. Initialize the controller BEFORE the simulation loop begins
controller = LQRPWPFController(mass=1.35, dim=0.10, max_thrust=0.50, dt=0.01)

# Dummy initial state (replace with reading from your sim)
current_state = np.array([1.0, 0.5, -0.5, 0,0,0, 0.75, -0.75, 0.75, 0,0,0])

sim_running = True
while sim_running:
    # 2. Get the current state from the simulation environment
    # current_state = get_state_from_sim() 
    
    # 3. Compute control action
    thruster_commands = controller.compute_control(current_state)
    
    # 4. Apply thruster commands to the simulation
    # apply_commands_to_sim(thruster_commands)
    
    # 5. Step simulation
    # sim.step()