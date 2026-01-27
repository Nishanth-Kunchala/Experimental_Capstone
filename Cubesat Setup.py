# from pxr import Usd, UsdGeom, Gf

# Creating general vars
Cube_Path = "/World/Cube"

Cube_Dim = 0.1 # 10cm in m

lx = Cube_Dim/2
ly = Cube_Dim/2
lz = Cube_Dim/2
l = 0.08/2 # m (0.8 U) the side distances (not full U)
Dc = 0.05/2 # m (0.5 U) the distance between center thrusters

f = [[0, 0, -1], [0, -1, 0], [0, 0, 1], [0, 1, 0], [-1, 0, 0], [-1, 0, 0],
     
      [0, 0, -1], [0, -1, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [1, 0, 0,]]

# x positions
rx = [lx, lx, lx, lx, lx, lx, -lx, -lx, -lx, -lx, -lx, -lx]

# y positions
ry = [-l, ly, l, -ly, 0, 0, l, ly, -l, -ly, 0, 0]

# z positions
rz = [lz, l, -lz, -l, Dc, -Dc, lz, -l, -lz, l, Dc, -Dc]

print("Hi" + "/Hi")