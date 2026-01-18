import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

'''
=========================================
1. CONFIGURATION & PARAMETERS
=========================================
'''
# Physics Parameters (Matched to sim.py visuals)
# Note: sim.py used half-extents of 0.0582, so full width is ~0.1164m
MASS = 1.35
SIDE_LENGTH = 0.0582 * 2.0 
MAX_THRUST = 0.50
DT = 1.0 / 60.0  # 60 Hz Simulation Step

# LQR Costs
Q_WEIGHT_POS = 100.0
Q_WEIGHT_ANG = 10.0
R_WEIGHT = 1.0

# Initial State for the Simulation [x, y, z] (Meters), [r, p, y] (Radians)
# We start slightly off-center to force the LQR to work.
INITIAL_POS = [1.0, 0.5, -0.5]
INITIAL_ORN = p.getQuaternionFromEuler([0.2, -0.2, 0.1]) 

'''
=========================================
2. PYBULLET SETUP (Visuals & Physics)
=========================================
'''
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0) # Zero G
p.setTimeStep(DT)

# Solver settings for stability
p.setPhysicsEngineParameter(numSolverIterations=200)
p.setPhysicsEngineParameter(useSplitImpulse=1, splitImpulsePenetrationThreshold=-0.004, enableConeFriction=0)

# Visual Tweaks
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

# Create CubeSat
HALF = SIDE_LENGTH / 2.0
cubesat_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[HALF, HALF, HALF])
cubesat_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[HALF, HALF, HALF], rgbaColor=[0.8, 0.8, 0.8, 1])
cubesat_body = p.createMultiBody(baseMass=MASS, baseCollisionShapeIndex=cubesat_col, baseVisualShapeIndex=cubesat_visual, basePosition=INITIAL_POS, baseOrientation=INITIAL_ORN)

# Disable damping to test control authority purely
p.changeDynamics(cubesat_body, -1, linearDamping=0.0, angularDamping=0.0)

# Camera zoom
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0.25,0,0])

# Thruster Configuration (Copied from sim.py)
thruster_config = [
    ({'pos': [0.0532, -0.0400, 0.0413], 'dir': [0, 0, -1]}), # A
    ({'pos': [0.0532, 0.0500, 0.0331], 'dir': [0, -1, 0]}),  # B
    ({'pos': [0.0532, 0.0400, -0.0413], 'dir': [0, 0, 1]}),  # C
    ({'pos': [0.0532, -0.0500, -0.0331], 'dir': [0, 1, 0]}), # D
    ({'pos': [0.0582, 0.0000, 0.0331], 'dir': [-1, 0, 0]}),  # E
    ({'pos': [0.0582, 0.0000, -0.0331], 'dir': [-1, 0, 0]}), # F
    ({'pos': [-0.0532, 0.040, 0.0413], 'dir': [0, 0, -1]}),  # G
    ({'pos': [-0.0532, -0.0500, 0.0331], 'dir': [0, 1, 0]}), # H
    ({'pos': [-0.0532, -0.0400, -0.0413], 'dir': [0, 0, 1]}), # I
    ({'pos': [-0.0532, 0.0500, -0.0331], 'dir': [0, -1, 0]}), # J
    ({'pos': [-0.0582, 0.0000, 0.0331], 'dir': [1, 0, 0]}),  # K
    ({'pos': [-0.0582, 0.000, -0.0331], 'dir': [1, 0, 0]}),  # L
]

thrust_line_ids = [None] * len(thruster_config)

'''
=========================================
3. CONTROLLER MATH (LQR Setup)
=========================================
'''
def compute_model_matrices(mass, side_length, config):
    """
    Generates the Continuous A and B matrices based on the 
    actual physical configuration of the thrusters.
    State Vector x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] (12x1)
    """
    # Inertia Tensor for a cube (Approximate)
    # I = m * s^2 / 6
    I_val = mass * (side_length**2) / 6.0
    I = np.eye(3) * I_val
    I_inv = np.linalg.inv(I)

    # --- A Matrix (Continuous) ---
    # x_dot = v
    # v_dot = 0 (no natural dynamics in space)
    # ang_dot = rate
    # rate_dot = 0
    A = np.zeros((12, 12))
    # Position derivative is velocity
    A[0, 3] = 1; A[1, 4] = 1; A[2, 5] = 1
    # Angle derivative is angular rate (Small angle approximation)
    A[6, 9] = 1; A[7, 10] = 1; A[8, 11] = 1

    # --- B Matrix (Continuous) ---
    # Maps control inputs u (12 thrusters) to state derivatives
    num_thrusters = len(config)
    B = np.zeros((12, num_thrusters))

    for i, thrust in enumerate(config):
        pos = np.array(thrust['pos'])
        direction = np.array(thrust['dir'])

        # Force contribution (F = u * dir) -> Accel = F/m
        accel_linear = direction / mass
        
        # Torque contribution (Tau = r x F) -> Accel_ang = I_inv * Tau
        torque = np.cross(pos, direction)
        accel_angular = I_inv @ torque

        # Fill B matrix
        # Linear Velocity terms (rows 3,4,5)
        B[3:6, i] = accel_linear
        # Angular Velocity terms (rows 9,10,11)
        B[9:12, i] = accel_angular

    return A, B

print("Computing LQR Gain...")
# 1. Get Model
A_c, B_c = compute_model_matrices(MASS, SIDE_LENGTH, thruster_config)

# 2. Discretize
A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, np.eye(12), np.zeros((12,12))), DT)

# 3. Define Costs
# Penalize position error heavily, angle error moderately
Q = np.eye(12)
Q[0:3, 0:3] *= Q_WEIGHT_POS  # Position x,y,z
Q[6:9, 6:9] *= Q_WEIGHT_ANG  # Angles
R = np.eye(12) * R_WEIGHT    # Cost of using fuel

# 4. Solve Riccati Equation
P = solve_discrete_are(A_d, B_d, Q, R)

# 5. Compute K
K = np.linalg.inv(R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
print("LQR Gain Computed.")

'''
=========================================
4. HELPER FUNCTIONS
=========================================
'''
label_ids = []

def draw_world_axes():
    # Draw static world axes at (0,0,0)
    # X = Red, Y = Green, Z = Blue
    origin = [0, 0, 0]
    line_len = 0.5
    
    p.addUserDebugLine(origin, [line_len, 0, 0], [1, 0, 0], lineWidth=4, lifeTime=0)
    p.addUserDebugLine(origin, [0, line_len, 0], [0, 1, 0], lineWidth=4, lifeTime=0)
    p.addUserDebugLine(origin, [0, 0, line_len], [0, 0, 1], lineWidth=4, lifeTime=0)
    
    p.addUserDebugText("World X", [line_len, 0, 0], [1, 0, 0], textSize=1.0, lifeTime=0)
    p.addUserDebugText("World Y", [0, line_len, 0], [0, 1, 0], textSize=1.0, lifeTime=0)
    p.addUserDebugText("World Z", [0, 0, line_len], [0, 0, 1], textSize=1.0, lifeTime=0)

def draw_labels_and_axes(body_id):
    # (Simplified version of sim.py's debug drawing)
    global label_ids
    pos_world, orn_world = p.getBasePositionAndOrientation(body_id)

    # Draw Thruster Labels
    if not label_ids:
        # Initialize labels once
        for i, config in enumerate(thruster_config):
            label_ids.append(p.addUserDebugText(chr(ord('A') + i), [0,0,0], [0,1,0], textSize=1.0))
    
    for i, config in enumerate(thruster_config):
        local_pos = np.array(config['pos'])
        norm = np.linalg.norm(local_pos)
        offset = local_pos / norm * 0.02 if norm > 0 else np.array([0,0,0])
        world_pos = p.multiplyTransforms(pos_world, orn_world, (local_pos + offset).tolist(), [0,0,0,1])[0]
        p.addUserDebugText(chr(ord('A') + i), world_pos, [0,1,0], textSize=1.0, replaceItemUniqueId=label_ids[i])

    # Draw Body Local Axes (Smaller than world axes)
    # axis_len = 0.15
    # axes = [[axis_len,0,0], [0,axis_len,0], [0,0,axis_len]]
    # colors = [[1,0,0], [0,1,0], [0,0,1]]
    
    # start = pos_world
    # for i in range(3):
    #     end = p.multiplyTransforms(pos_world, orn_world, axes[i], [0,0,0,1])[0]
    #     p.addUserDebugLine(start, end, colors[i], lineWidth=2, lifeTime=DT*2)

# def apply_control(body_id, u_vec):
#     pos_world, orn_world = p.getBasePositionAndOrientation(body_id)
    
#     for i, thrust_mag in enumerate(u_vec):
#         if thrust_mag <= 1e-4: continue
        
#         config = thruster_config[i]
#         local_pos = config['pos']
#         local_dir = config['dir']
        
#         # Physics Force
#         force = [d * thrust_mag for d in local_dir]
#         p.applyExternalForce(body_id, -1, force, local_pos, p.LINK_FRAME)
        
#         # Visual Force (Red Line)
#         start_pt = p.multiplyTransforms(pos_world, orn_world, local_pos, [0,0,0,1])[0]
#         force_world = p.multiplyTransforms([0,0,0], orn_world, force, [0,0,0,1])[0]
#         end_pt = [start_pt[k] - force_world[k]*0.1 for k in range(3)] # Scale for vis
#         p.addUserDebugLine(start_pt, end_pt, [1,0.35,0], lineWidth=5, lifeTime=DT)

def apply_control(body_id, u_vec):
    global thrust_line_ids

    pos_world, orn_world = p.getBasePositionAndOrientation(body_id)

    for i, thrust_mag in enumerate(u_vec):
        config = thruster_config[i]
        local_pos = config['pos']
        local_dir = config['dir']

        # Physics force
        if thrust_mag > 1e-4:
            force = [d * thrust_mag for d in local_dir]
            p.applyExternalForce(body_id, -1, force, local_pos, p.LINK_FRAME)
        else:
            force = [0,0,0]

        # Compute line endpoints
        start_pt = p.multiplyTransforms(pos_world, orn_world, local_pos, [0,0,0,1])[0]
        force_world = p.multiplyTransforms([0,0,0], orn_world, force, [0,0,0,1])[0]
        end_pt = [start_pt[k] - force_world[k]*0.125 for k in range(3)]

        # --- PERSISTENT DEBUG LINE ---
        if thrust_line_ids[i] is None:
            thrust_line_ids[i] = p.addUserDebugLine(
                start_pt, end_pt,
                [1,0.35,0],   # neon-ish orange
                lineWidth=4,
                lifeTime=0       # <-- persists forever
            )
        else:
            p.addUserDebugLine(
                start_pt, end_pt,
                [1,0.35,0],
                lineWidth=4,
                replaceItemUniqueId=thrust_line_ids[i],
                lifeTime=0
            )


'''
=========================================
5. MAIN LOOP
=========================================
'''
print("Starting Simulation Loop...")
target_state = np.zeros(12) # Goal: Origin, Zero velocity, Zero Angle

# Draw static world axes once
draw_world_axes()

start=True
while True:
    # 1. READ STATE from PyBullet
    pos, quat = p.getBasePositionAndOrientation(cubesat_body)
    lin_vel, ang_vel = p.getBaseVelocity(cubesat_body)
    euler = p.getEulerFromQuaternion(quat)

    # Construct State Vector [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    # Note: This assumes small angles for direct mapping of Euler rates to p,q,r
    x_curr = np.array([
        pos[0], pos[1], pos[2],
        lin_vel[0], lin_vel[1], lin_vel[2],
        euler[0], euler[1], euler[2],
        ang_vel[0], ang_vel[1], ang_vel[2]
    ])

    # 2. CALCULATE CONTROL (u = -Kx)
    # Error = Current - Target
    error = x_curr - target_state
    
    u_optimal = -K @ error

    # 3. CONSTRAINT (0 to Max Thrust)
    # Thrusters can only push (0 to MAX), they cannot pull.
    u_applied = np.clip(u_optimal, 0, MAX_THRUST)

    if start:
        start = False
        time.sleep(2)

    # 4. APPLY TO PHYSICS
    apply_control(cubesat_body, u_applied)
    draw_labels_and_axes(cubesat_body)


    # 5. STEP
    p.stepSimulation()
    time.sleep(DT)