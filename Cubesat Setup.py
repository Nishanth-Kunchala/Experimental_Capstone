import omni
import numpy as np
from pxr import Gf, UsdPhysics

class CubeSatController:
	
	def __init__(self):
		# Creating general vars
		self.Cube_Path = "/World/Cube"
		self.Thruster_Path = []
		self.thruster_cmd = np.zeros(12)
		self.stage = omni.usd.get_context().get_stage()
		self.CubeSat = self.stage.GetPrimAtPath(self.Cube_Path)
		self.CubeSat_Properties()

	# Assign Cube Properties
	def CubeSat_Properties(self):
		self.m = 1.35 # CubeSat mass in kg

		UsdPhysics.RigidBodyAPI.Apply(self.CubeSat)
		UsdPhysics.CollisionAPI.Apply(self.CubeSat)
		self.CubeSat_mass = UsdPhysics.MassAPI.Apply(self.CubeSat)
		self.CubeSat_mass.CreateMassAttr(self.m) 

		self.Cube_Dim = 0.1 # 10cm in m

		self.lx =self.Cube_Dim/2
		self.ly =self.Cube_Dim/2
		self.lz = self.Cube_Dim/2
		self.l = 0.08/2 # m (0.8 U) the side distances (not full U)
		self.Dc = 0.05/2 # m (0.5 U) the distance between center thrusters

		self.T_force = 25/1000 # Thrust Magnitude in newtons

		# Setting up Thruster List
		# Name, Location, Rotation Axis, Rotation Magnitude
		self.Thruster = [

		    ("T1", (self.lx, self.ly, self.lz), (0,0,0)),
		    ("T2", (self.lx, -self.l, self.l),  (-90,0,0)),
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
			self.Thruster_Path.append(Cube_Path + "/" + name)
			Current_thruster =  Stage.GetPrimAtPath(Cube_Path + "/" + name)
			
			# Setting Position and Orientation
			Current_thruster.GetAttribute("xformOp:translate").Set(pos)
			Current_thruster.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(axis))
			
		# Note that if the xforms do not rotate as expected, select one and toggle "Current Transformation Space:Local" by selecting the
		# Earth Icon

sim = CubeSatController()

print(sim.Thruster_Path[0])



