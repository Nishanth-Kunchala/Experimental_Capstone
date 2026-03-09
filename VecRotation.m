function [R_zyx] = VecRotation(theta_z,theta_y,theta_x)
%Creates a zyx rotation matrix (R_xyz)for a [x y z]' vector from angles of
%rotation

sinx = sind(theta_x);
cosx = cosd(theta_x);

siny = sind(theta_y);
cosy = cosd(theta_y);

sinz = sind(theta_z);
cosz = cosd(theta_z);


Rx = [1 0 0; 0 cosx -sinx; 0 sinx cosx];
Ry = [cosy 0 siny; 0 1 0; -siny 0 cosy];
Rz = [cosz -sinz 0; sinz cosz 0; 0 0 1];

R_zyx = Rz*Ry*Rx;

end