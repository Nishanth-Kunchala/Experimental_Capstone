import omni
from pxr import Gf
# This script can be used to modify the position and orientation of the thrusters on the cubesat

# Creating general vars
Cube_Path = "/World/Cube"

Cube_Dim = 0.1 # 10cm in m

lx = Cube_Dim/2
ly = Cube_Dim/2
lz = Cube_Dim/2
l = 0.08/2 # m (0.8 U) the side distances (not full U)
Dc = 0.05/2 # m (0.5 U) the distance between center thrusters

T_force = 10 # Thrust Magnitude in newtons

# Setting up Thruster List
# Name, Location, Rotation Axis, Rotation Magnitude

Thruster = [

    ("T1", (lx, ly, lz), (0,0,0)),
    ("T2", (lx, -l, l),  (-90,0,0)),
    ("T3", (lx, l, -lz),  (-180,0,0)),
    ("T4", (lx, -ly, -l),  (90,0,0)),

    ("T5", (lx, 0, Dc), (0,90,0)),
    ("T6", (lx, 0, -Dc), (0,90,0)),

    ("T7", (-lx, l, lz), (0,0,0)),
    ("T8", (-lx, ly, -l), (-90,0,0)),
    ("T9", (-lx, -l, -lz), (-180,0,0)),
    ("T10", (-lx, -ly, l), (90,0,0)),

    ("T11", (-lx, 0, Dc), (0,-90,0)),
    ("T12", (-lx, 0, -Dc), (0,-90,0)),

]
 
Stage = omni.usd.get_context().get_stage()
CubeSat = Stage.GetPrimAtPath(Cube_Path)

# Setting Thrusters as Xforms

for name, pos, axis in Thruster:
	
	# Setting Current Thruster
	Thruster_Path = Cube_Path + "/" + name
	Current_thruster =  Stage.GetPrimAtPath(Thruster_Path)
	
	# Setting Position and Orientation
	Current_thruster.GetAttribute("xformOp:translate").Set(pos)
	Current_thruster.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(axis))
	
    # Note that if the xforms do not rotate as expected, select one and toggle "Current Transformation Space:Local" by selecting the
	# Earth Icon