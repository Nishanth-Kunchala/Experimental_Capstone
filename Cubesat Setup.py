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
		
		self.Thruster_path = []
		self.thruster_cmd = np.zeros(12)
		self.Thrust = 55/1000
		
		self.stage = omni.usd.get_context().get_stage()
		self.stage_id = omni.usd.get_context().get_stage_id()
		
		self.CubeSat = self.stage.GetPrimAtPath(self.Cube_path)
		self.xform = UsdGeom.Xformable(self.CubeSat)
		
		self.CubeSat_Properties()
		
	# Assign Cube Properties
	def CubeSat_Properties(self):
		
		self.m = 1.35 # CubeSat mass in kg
		self.Cube_Dim = 0.1 # 10cm in m
		self.Inertia =(self.m/6)*((self.Cube_Dim**2) - (0.02**2))
		self.Inertia_vec = Gf.Vec3d(self.Inertia,self.Inertia,self.Inertia)
		
		CubeSat_dynamics = UsdPhysics.RigidBodyAPI.Apply(self.CubeSat)
		UsdPhysics.CollisionAPI.Apply(self.CubeSat)
		
		#self.CubeSat_mass = UsdPhysics.MassAPI.Apply(self.CubeSat)
		#self.CubeSat_mass.CreateMassAttr(self.m) 
		self.CubeSat.GetAttribute("physics:mass").Set(self.m)
		
		self.CubeSat.GetAttribute("physics:diagonalInertia").Set(self.Inertia_vec)
		#World/Cube.physics:diagonalInertia
		self.lx =self.Cube_Dim/2
		self.ly =self.Cube_Dim/2
		self.lz = self.Cube_Dim/2
		self.l = 0.08/2 # m (0.8 U) the side distances (not full U)
		self.Dc = 0.05/2 # m (0.5 U) the distance between center thrusters
		
		self.T_force = 25/1000 # Thrust Magnitude in newtons
		
		# Setting up Thruster List
		# Name, Location, Rotation Axis, Rotation Magnitude
		self.Thruster = [

		    ("T1", (self.lx, -self.l, self.lz), (0,0,0)),
		    ("T2", (self.lx, self.ly, self.l),  (-90,0,0)),
		    ("T3", (self.lx, self.l, -self.lz),  (-180,0,0)),
		    ("T4", (self.lx, -self.ly, -self.l),  (90,0,0)),

		    ("T5", (self.lx, 0, self.Dc), (0,90,0)),
		    ("T6", (self.lx, 0, -self.Dc), (0,90,0)),

		    ("T7", (-self.lx, self.l, self.lz), (0,0,0)),
		    ("T8", (-self.lx, self.ly, -self.l), (-90,0,0)),
		    ("T9", (-self.lx, -self.l, -self.lz), (-180,0,0)),
		    ("T10", (-self.lx, -self.ly, self.l), (90,0,0)),

		    ("T11", (-self.lx, 0, self.Dc), (0,-90,0)),
		    ("T12", (-self.lx, 0, -self.Dc), (0,-90,0)),
		    
		]
		
		# Setting Thrusters as Xforms
		for name, pos, axis in self.Thruster:
			
			# Setting Current Thruster
			self.Thruster_path.append(self.Cube_path + "/" + name)
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
		for i, path in enumerate(self.Thruster_path):
			
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
			Thruster_force = world_cord*self.Thrust
			
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
		self.controller_dt = 0.01
		self.actuator_step = 0.0
		
		self.cmd = np.zeros(12)
		
		# Setting variables to read the API
		self.x = self.sim.CubeSat.GetAttribute("xformOp:translate")
		self.vel = self.sim.CubeSat.GetAttribute("physics:velocity")
		self.w = self.sim.CubeSat.GetAttribute("physics:angularVelocity")
		
		# Setting up the physxs sim
		self.physx = omni.physx.acquire_physx_interface()
		self.sub = None
		
		# Reading the gain matrix
		self.K = np.loadtxt(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\Gain_Matrix.csv", delimiter=',')
		
	
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
		
		angular_vel =( w.Get())*(np.pi/180)
		
		# Build the state vector
		state = np.array([pos[0], pos[1], pos[2],
		linear_vel[0], linear_vel[1], linear_vel[2],
		orient[0], orient[1], orient[2],
		angular_vel[0], angular_vel[1], angular_vel[2]
		])
		
		return state
	
	# Function that is called each physics time step, add controller here
	def sim_step(self,dt):
		
		# Controller logic goes here. Use the controller_dt and actuator_step to
		# reprsent the frequency that the controller updates a
		self.sim.apply_thrust(self.cmd)
	
	# Starts the controller simulation
	def start_sim(self):
		
		if self.sub is None:
			
			self.cmd = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1]
			self.sub = self.physx.subscribe_physics_step_events(self.sim_step)
			
	
	# Stops the controller simulation
	def stop_sim(self):
		
		if self.sub is not None:
			self.sub.unsubscribe()
			self.sub = None
			self.cmd = np.zeros(12)
		
	
# Check whether there is already a Cube_main Object

# del Cube_main

if "Cube_main" not in globals():
	
	print("Generating Cube_main Object")
	Cube_main = CubeSatController()

Cube_main.sim.CubeSat_Properties()
#Cube_main.start_sim()
#Cube_main.stop_sim()

#print(Cube_main.K)

#x = Cube_main.sim.CubeSat.GetAttribute("xformOp:translate")
#vel = Cube_main.sim.CubeSat.GetAttribute("physics:velocity")
#w = Cube_main.sim.CubeSat.GetAttribute("physics:angularVelocity")

#pos = x.Get()
#linear_vel = vel.Get()

#transform = Cube_main.sim.xform.ComputeLocalToWorldTransform(0.0)
#rotation = transform.ExtractRotation()

#orient = (rotation.Decompose(Gf.Vec3d(1,0,0),
#Gf.Vec3d(0,1,0),
#Gf.Vec3d(0,0,1)))*(np.pi/180)

#angular_vel =( w.Get())*(np.pi/180)
#angular_vel =( w.Get())*(np.pi/180)

# Build the state vector
#state = np.array([pos[0], pos[1], pos[2],
#linear_vel[0], linear_vel[1], linear_vel[2],
#orient[0], orient[1], orient[2],angular_vel[0],
#angular_vel[1], angular_vel[2]
#])

#w1 = angle.GetReal()
#x1, y1, z1 = angle.GetImaginary()

#print(-Cube_main.K @ state2)






