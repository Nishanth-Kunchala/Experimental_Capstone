import omni
import carb
import numpy as np

from pxr import Gf, UsdPhysics, UsdGeom, PhysicsSchemaTools
from omni.isaac.core.utils.rotations import quat_to_euler_angles

#from omni.physx.scripts import physicsUtils
# Cubesat class
class CubeSatSetup:
	
	def __init__(self):
		# Creating general vars
		self.Cube_path = "/World/Cube"
		
		self.thruster_count = 12;
		self.thruster_path = []
		self.thruster_cmd = np.zeros(self.thruster_count)
		self.thrust = 25/1000
		
		self.stage = omni.usd.get_context().get_stage()
		self.stage_id = omni.usd.get_context().get_stage_id()
		
		self.CubeSat = self.stage.GetPrimAtPath(self.Cube_path)
		self.xform = UsdGeom.Xformable(self.CubeSat)
		
		self.CubeSat_Properties()
		
	# Assign Cube Properties
	def CubeSat_Properties(self):
		
		self.m = 2.652333  # CubeSat mass in kg
		self.Cube_Dim = 0.1 # 10cm in m
		#self.Inertia =(self.m/6)*((self.Cube_Dim**2) - (0.02**2))
		self.Inertia1 = 0.0487909933
		self.Inertia2 = 0.0533788243 
		self.Inertia3 = 0.0148340253
		
		self.Inertia_vec = Gf.Vec3d(self.Inertia1,self.Inertia2,self.Inertia3)
		
		CubeSat_dynamics = UsdPhysics.RigidBodyAPI.Apply(self.CubeSat)
		UsdPhysics.CollisionAPI.Apply(self.CubeSat)
		
		self.CubeSat.GetAttribute("physics:mass").Set(self.m)
		self.CubeSat.GetAttribute("physics:diagonalInertia").Set(self.Inertia_vec)
		
		#World/Cube.physics:diagonalInertia
		self.lx = 0.0550
		self.ly = 0.0460
		self.lz = 0.0451
		self.l = 0.04 
		self.l2 = 0.0331
		self.Dc = 0.05/2 # m (0.5 U) the distance between center thrusters
		
		# Setting up Thruster List
		# Name, Location, Rotation Axis, Rotation Magnitude
		self.Thruster = [
# Mapped for Test Tuning (T10 and T8 swapped)
		    ("T1", (self.lx, -self.l, self.lz), (0,0,0)),
		    ("T2", (self.lx, self.ly, self.l2),  (-90,0,0)),
		    ("T3", (self.lx, self.l, -self.lz),  (-180,0,0)),
		    ("T4", (self.lx, -self.ly, -self.l2),  (90,0,0)),

		    ("T5", (0.0680, 0, self.l2), (0,90,0)),
		    ("T6", (0.0680, 0, -self.l2), (0,90,0)),

		    ("T7", (-self.lx, self.l, self.lz), (0,0,0)),
		    ("T8", (-self.lx, -self.ly, self.l2), (90,0,0)),
		    ("T9", (-self.lx, -self.l, -self.lz), (-180,0,0)),
		    ("T10", (-self.lx, self.ly, -self.l2), (-90,0,0)),

		    ("T11", (-0.0680, 0, self.l2), (0,-90,0)),
		    ("T12", (-0.00680, 0, -self.l2), (0,-90,0)),
		    
		]
		
		# Setting Thrusters as Xforms
		for name, pos, axis in self.Thruster:
			
			# Setting Current Thruster
			self.thruster_path.append(self.Cube_path + "/" + name)
			Current_thruster =  self.stage.GetPrimAtPath(self.Cube_path + "/" + name)
			
			# Setting Position and Orientation
			Current_thruster.GetAttribute("xformOp:translate").Set(pos)
			Current_thruster.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(axis))
		
		# Note that if the xforms do not rotate as expected, select one and toggle "Current Transformation Space:Local" by selecting the
		# Earth Icon
		
		# Applies forces at the given thruster locations (1 == on)
	def apply_thrust(self, cmd_vector):
		
		#sim_physx = omni.physx.bindings._physx.IPhysxSimulation
		sim_physx = omni.physx.get_physx_simulation_interface()
		local_cord = Gf.Vec3d(0,0,-1)
		
		stage_id = PhysicsSchemaTools.sdfPathToInt(self.Cube_path)
		
		# Looping through Thrusters
		for i, path in enumerate(self.thruster_path):
			
			# Only apply a force when prompted by the controller
			if cmd_vector[i] == 0:
				
				continue
			
			# Calculate location of current thruster 
			# Selecting the current thruster
			Thruster_prim = self.stage.GetPrimAtPath(path)
			Thruster_xform = UsdGeom.Xformable(Thruster_prim)
			
			# Calculating the position and rotational transform matrix
			transform_matrix = Thruster_xform.ComputeLocalToWorldTransform(0.0)
			
			# Extracting the orientation in the inertial coordinate system
			world_cord = transform_matrix.TransformDir(local_cord)
			world_cord.Normalize()
			
			pos = transform_matrix.ExtractTranslation()
			
			# Computing the force vector
			Thruster_force = world_cord*cmd_vector[i]
			
			sim_physx.apply_force_at_pos(
			self.stage_id,
			PhysicsSchemaTools.sdfPathToInt(self.Cube_path),
			Thruster_force,
			pos)
			

# Simulation Class
class CubeSatController:
	
	def __init__(self):
		
		 #Generating Cube
		self.sim = CubeSatSetup()
		
		# Setting stepping parameters
		self.controller_step = 0.01
		self.actuator_step = 0.0
		self.t_step = 0
		
		# Setting convergence criteria
		#x_ac = 5e-3;
		#v_ac = 1e-3;
		#theta_ac = 5*(np.pi/180)
		#w_ac = 1*(np.pi/180)
		
		#self.conv = np.array([x_ac, x_ac, x_ac,
		#v_ac, v_ac, v_ac,
		#theta_ac, theta_ac, theta_ac,
		#w_ac, w_ac, w_ac])
		
		self.cmd = np.zeros(self.sim.thruster_count)
		
		# Setting variables to read the API
		self.x = self.sim.CubeSat.GetAttribute("xformOp:translate")
		self.vel = self.sim.CubeSat.GetAttribute("physics:velocity")
		self.w = self.sim.CubeSat.GetAttribute("physics:angularVelocity")
		
		# Setting up the physxs sim
		self.physx = omni.physx.acquire_physx_interface()
		self.sub = None
		
		# Setting LQR vals
		self.K = np.loadtxt(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\G_M_Test1.csv", delimiter=',')
		self.f_states = np.zeros(self.sim.thruster_count)
		
	
	def sim_state(self):
		
		# Finding the States from IsaacSim
		pos = self.x.Get()
		linear_vel = self.vel.Get()
		
		# Calculating the Euler Angles using a rotation matrix
		transform = self.sim.xform.ComputeLocalToWorldTransform(0.0)
		rotation = transform.ExtractRotation()
		
		orient = (rotation.Decompose(Gf.Vec3d(1,0,0),
		Gf.Vec3d(0,1,0),
		Gf.Vec3d(0,0,1)
		))*(np.pi/180)
		
		angular_vel =(self.w.Get())*(np.pi/180)
		
		# Build the state vector
		state = np.array([pos[0], pos[1], pos[2],
		linear_vel[0], linear_vel[1], linear_vel[2],
		orient[0], orient[1], orient[2],
		angular_vel[0], angular_vel[1], angular_vel[2]
		])
		
		return state
	
	# Function that performs PWPF LQR control allocation
	def compute_control(self,state, Km=4.0, Tm=0.1, Uon=0.7, Uoff=0.4):
		
		# Calculating LQR Thrust
		u_lqr = -self.K @ state
		u_lqr[u_lqr < 0] = 0
		
		# Setting pwpwd
		u_pwpf = np.zeros(self.cmd.shape[0])
		
		for i in range(self.sim.thruster_count):
			# Error between desired continuous thrust and actual applied discrete thrust
			e = u_lqr[i] - self.cmd[i]*self.sim.thrust
			
			# Discrete integration for the filter state
			self.f_states[i] += (self.controller_step / Tm) * (Km * e - self.f_states[i])
			
			 # Schmidt Trigger (Hysteresis logic)
			if np.abs(self.f_states[i]) >= Uon:
				
				u_pwpf[i] = self.sim.thrust
				
			elif np.abs(self.f_states[i]) <= Uoff:
				
				u_pwpf[i] = 0
				
			
		return u_pwpf
	
	
	# Function that is called each physics time step, add controller here
	def sim_step(self,dt, sim_type="data"):
		
		self.actuator_step += dt
		state = self.sim_state()
		
		# re-calculate the thrust each controller step
		if self.actuator_step >= self.controller_step:
			
			self.actuator_step = 0.0
			self.cmd = self.compute_control(state)
			
		# Logging Data
		self.thrust_log.append(self.cmd)
		self.t_log.append(self.t_step)
		self.state_log.append(state)
		
		self.t_step += dt
		
		self.sim.apply_thrust(self.cmd)
		
	
	# Starts the controller simulation
	def start_sim(self):
		
		if self.sub is None:
			
			state = self.sim_state()
			
			self.thrust_log = []
			self.t_log = []
			self.state_log = []
			self.sub = self.physx.subscribe_physics_step_events(self.sim_step)
			
	
	# Stops the controller simulation
	def stop_sim(self):
		
		if self.sub is not None:
			self.sub.unsubscribe()
			self.sub = None
			self.cmd = np.zeros(self.sim.thruster_count)
		
		self.log_sim()
	
	# Writes variables
	def log_sim(self):
		
		thrust_log = np.array(self.thrust_log)
		t_log = np.array(self.t_log)
		state_log = np.array(self.state_log)
		
		path = "\\Users\\anton\\OneDrive\\Documents\\GitHub\\Experimental_Capstone\\sim_T1Z_TX.npz"
		
		np.savez(path, thrust_log=thrust_log, t_log=t_log, state_log = state_log)
		
	
# Check whether there is already a Cube_main Object
if "Cube_main" not in globals():
	
	print("Generating Cube_main Object")
	Cube_main = CubeSatController()
	
elif Cube_main.sub is not None:
	
	Cube_main.sub.unsubscribe()
	Cube_main.sub = None

# sim

#Cube_main.start_sim()
#Cube_main.stop_sim()












