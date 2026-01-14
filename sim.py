import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# --- SETUP ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0) # Zero G

# CRITICAL: Set the timestep to match your simulation loop
dt = 1.0 / 60.0  # 60 Hz
p.setTimeStep(dt)

# Increase solver iterations for better accuracy
# Default is 50, but we need more for precise force balance
p.setPhysicsEngineParameter(numSolverIterations=200)

# Disable damping that can accumulate over time
p.setPhysicsEngineParameter(
    useSplitImpulse=1,
    splitImpulsePenetrationThreshold=-0.004,
    enableConeFriction=0
)

# Hide the default GUI items
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

# Black background
p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0,0,0])

# Create CubeSat (Visuals + Physics)
HALF = 0.0582  # 58.2 mm = proper half-side in meters

cubesat_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[HALF, HALF, HALF])
cubesat_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[HALF, HALF, HALF], rgbaColor=[0.8, 0.8, 0.8, 1])
cubesat_body = p.createMultiBody(baseMass=1.33, baseCollisionShapeIndex=cubesat_col, baseVisualShapeIndex=cubesat_visual, basePosition=[0, 0, 0])
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,   # zoom level (bigger = farther away)
    cameraYaw=45,         # left/right rotation
    cameraPitch=-30,      # up/down tilt
    cameraTargetPosition=[0, 0, 0]  # what the camera looks at
)

# Disable sleep so physics always runs
p.changeDynamics(cubesat_body, -1, 
                activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING,
                linearDamping=0.0,   # Remove linear damping
                angularDamping=0.0)  # Remove angular damping

# --- THRUSTER CONFIGURATION (12 Thrusters) ---
# Mapping Index -> (Local Position [x,y,z], Force Direction [x,y,z])
# This is a hypothetical 6-DOF layout (corners firing tangentially)
thruster_config = [
    # Thruster A
    ({'pos': [0.0532, -0.0400, 0.0413], 'dir': [0, 0, -1]}), 
    # Thruster B
    ({'pos': [0.0532, 0.0500, 0.0331], 'dir': [0, -1, 0]}),
    # Thruster C
    ({'pos': [0.0532, 0.0400, -0.0413], 'dir': [0, 0, 1]}),
    # Thruster D
    ({'pos': [0.0532, -0.0500, -0.0331], 'dir': [0, 1, 0]}),
    # Thruster E
    ({'pos': [0.0582, 0.0000, 0.0331], 'dir': [-1, 0, 0]}),
    # Thruster F
    ({'pos': [0.0582, 0.0000, -0.0331], 'dir': [-1, 0, 0]}),
    # Thruster G
    ({'pos': [-0.0532, 0.040, 0.0413], 'dir': [0, 0, -1]}),
    # Thruster H
    ({'pos': [-0.0532, -0.0500, 0.0331], 'dir': [0, 1, 0]}),
    # Thruster I
    ({'pos': [-0.0532, -0.0400, -0.0413], 'dir': [0, 0, 1]}),
    # Thruster J
    ({'pos': [-0.0532, 0.0500, -0.0331], 'dir': [0, -1, 0]}),
    # Thruster K
    ({'pos': [-0.0582, 0.0000, 0.0331], 'dir': [1, 0, 0]}),
    # Thruster L
    ({'pos': [-0.0582, 0.000, -0.0331], 'dir': [1, 0, 0]}),
] 
label_ids = []
for i, config in enumerate(thruster_config):
    label = chr(ord('A') + i)  # 'A', 'B', ... 'L'

    pos = config['pos']
    dir = config['dir']

    # Offset text slightly outward so it doesn't clip into the cube surface
    text_offset = [pos[j] + dir[j] * 0.01 for j in range(3)]

    label_id = p.addUserDebugText(
        text=label,
        textPosition=text_offset,
        textColorRGB=[0, 1, 0],  # green labels
        textSize=1.4,
        lifeTime=0  # 0 = persistent
    )
    label_ids.append(label_id)

axis_length = 0.08  # 8 cm arrows
axis_labels_ids = []  # store debug text IDs so we can update each frame

def draw_local_axes(body_id):
    global axis_labels_ids
    pos, orn = p.getBasePositionAndOrientation(body_id)

    axes = {
        'X': ([0,0,0], [axis_length,0,0], [1,0,0]),  # red
        'Y': ([0,0,0], [0,axis_length,0], [0,1,0]),  # green
        'Z': ([0,0,0], [0,0,axis_length], [0,0,1]),  # blue
    }

    # If this is the first frame, create text IDs
    first_frame = len(axis_labels_ids) == 0

    for i, (label, (start_local, end_local, color)) in enumerate(axes.items()):
        start_world = p.multiplyTransforms(pos, orn, start_local, [0,0,0,1])[0]
        end_world = p.multiplyTransforms(pos, orn, end_local, [0,0,0,1])[0]

        # Draw line
        p.addUserDebugLine(start_world, end_world, color, lineWidth=3, lifeTime=0.1)

        # Draw label at the end of the axis
        if first_frame:
            text_id = p.addUserDebugText(
                label,
                end_world,
                textColorRGB=color,
                textSize=1.2,
                lifeTime=0  # persistent, will overwrite each frame
            )
            axis_labels_ids.append(text_id)
        else:
            p.addUserDebugText(
                label,
                end_world,
                textColorRGB=color,
                textSize=1.2,
                replaceItemUniqueId=axis_labels_ids[i]
            )

def update_labels(body_id):
    pos_world, orn_world = p.getBasePositionAndOrientation(body_id)

    for i, config in enumerate(thruster_config):
        local_pos = np.array(config['pos'])

        # Compute outward normal (from center of cube)
        norm = np.linalg.norm(local_pos)
        if norm == 0:
            offset_dir = np.array([1,0,0])  # fallback, shouldn't happen
        else:
            offset_dir = local_pos / norm

        # Offset 2 cm outward
        local_pos_offset = local_pos + offset_dir * 0.02

        # Convert to world space
        world_pos = p.multiplyTransforms(
            pos_world, orn_world,
            local_pos_offset.tolist(), [0,0,0,1]
        )[0]

        p.addUserDebugText(
            chr(ord('A') + i),
            world_pos,
            [0,1,0],
            textSize=1.4,
            replaceItemUniqueId=label_ids[i]
        )
        
# --- THE CORE FUNCTION ---
def apply_thruster_commands(body_id, command_array):
    """
    body_id: The PyBullet ID of the satellite
    command_array: A list of 12 floats (Newtons). E.g., [0.5, 0.0, 10.0, ...]
    
    Applies forces in LINK_FRAME (body frame) which is correct for thrusters
    that are physically attached to the satellite.
    """
    
    # Get current orientation for transforming local coordinates to world
    pos_world, orn_world = p.getBasePositionAndOrientation(body_id)

    for i, thrust_value in enumerate(command_array):
        # Safety: Don't process if we don't have a config for this index
        if i >= len(thruster_config): break
        
        # Efficiency: Don't compute physics for 0 thrust
        if abs(thrust_value) < 0.001: continue

        config = thruster_config[i]
        local_pos = config['pos']
        local_dir = config['dir']

        # 1. Apply the Force (Physics)
        # force_vector = direction * magnitude
        force_vec_local = [d * thrust_value for d in local_dir]
        
        # Use LINK_FRAME: both position and force are in body-relative coordinates
        # This is correct because thrusters are attached to the body and rotate with it
        p.applyExternalForce(body_id, -1, force_vec_local, local_pos, p.LINK_FRAME)

        # 2. Visual Debug (Optional) - Draw line showing the thrust
        # Transform to world coordinates for visualization
        start_point_world = p.multiplyTransforms(pos_world, orn_world, local_pos, [0,0,0,1])[0]
        
        # Rotate the force vector to world space for visualization
        force_vec_world = p.multiplyTransforms([0,0,0], orn_world,
                                       force_vec_local, [0,0,0,1])[0]
        
        # Calculate end point of the red line (scale for visibility)
        end_point_world = [start_point_world[j] + -1*force_vec_world[j] * 0.1 for j in range(3)]
        
        # Draw red line
        p.addUserDebugLine(start_point_world, end_point_world, [1, 0, 0], lineWidth=2, lifeTime=0.1)

def draw_3d_grid(grid_size=0.2, step=0.05, lifeTime=0):
    """
    Draw a 3D reference grid centered at the origin.
    grid_size : half-size of the grid along each axis (meters)
    step : spacing between grid lines
    lifeTime : 0 for persistent
    """
    # Draw lines parallel to X and Y axes in Z planes
    z_levels = np.arange(-grid_size, grid_size + step, step)
    for z in z_levels:
        for x in np.arange(-grid_size, grid_size + step, step):
            p.addUserDebugLine([x, -grid_size, z], [x, grid_size, z], [0.5, 0.5, 0.5], 1, lifeTime)
        for y in np.arange(-grid_size, grid_size + step, step):
            p.addUserDebugLine([-grid_size, y, z], [grid_size, y, z], [0.5, 0.5, 0.5], 1, lifeTime)

    # Optionally draw lines along Z for a cube reference
    xy_levels = np.arange(-grid_size, grid_size + step, step)
    for x in xy_levels:
        for y in xy_levels:
            p.addUserDebugLine([x, y, -grid_size], [x, y, grid_size], [0.5, 0.5, 0.5], 1, lifeTime)

# --- SIMULATION LOOP (Deterministic Timing) ---
print("Starting Sim... ")

dt = 1.0 / 60.0  # Must match p.setTimeStep() above!
t = 0.0

# Diagnostics
cumulative_impulse = np.zeros(3)

while True:
    # Increment time FIRST so conditions match the physics state
    t += dt

    # ---------------------------------------------------
    # FLIGHT CONTROLLER  (Thruster ON durations accurate)
    # ---------------------------------------------------
    # Use >= for start times to avoid off-by-one errors
    if t < 1.0:
        commands = [0.0]*12
    elif 1.0 <= t < 2.0:
        commands = [0.0]*12
        commands[10] = 1  # K
        commands[11] = 1  # L
    elif 2.0 <= t < 3.0:
        commands = [0.0]*12
        commands[4] = 1   # E
        commands[5] = 1   # F
    elif 3.0 <= t < 3.5:
        commands = [0.0]*12
    elif 3.5 <= t < 4.5:
        commands = [0.0]*12
        commands[2] = 1   # C
        commands[8] = 1   # I
    elif 4.5 <= t < 5.5:
        commands = [0.0]*12
        commands[0] = 1   # A
        commands[6] = 1   # G
    else:
        commands = [0.0]*12

    # ------------------------------------
    # APPLY FORCES AND VISUAL OVERLAYS
    # ------------------------------------
    apply_thruster_commands(cubesat_body, commands)

    # Visual debugging
    update_labels(cubesat_body)
    draw_local_axes(cubesat_body)
    draw_3d_grid(grid_size=1.0, step=1.0)

    # Velocity diagnostics
    lin_vel, ang_vel = p.getBaseVelocity(cubesat_body)
    print(f"t={t:.4f}  lin_vel={lin_vel}  ang_vel={ang_vel}")

    # Accumulate expected impulse this step
    for i, f in enumerate(commands):
        if abs(f) < 1e-9: continue
        cfg = thruster_config[i]
        dir_world = p.multiplyTransforms([0,0,0], p.getBasePositionAndOrientation(cubesat_body)[1],
                                        cfg['dir'], [0,0,0,1])[0]
        cumulative_impulse += np.array(dir_world) * f * dt

    # Print cumulative impulse every second
    if abs((t % 1.0) - 0.0) < 1e-6:
        print("cumulative_impulse (approx) =", cumulative_impulse)

    # Physics step
    p.stepSimulation()

    # Time increments EXACTLY with each physics step
    t += dt

    # Optional rendering speed control
    time.sleep(dt)