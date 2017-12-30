function [jGx,jGz] = GetNewFeatureJacs(xVehicle, z)
%Get jacobians

x = xVehicle(1,1);
y = xVehicle(2,1);
theta = xVehicle(3,1);
r = z(1);
range = z(2);

jGx = [ 1   0   -r*sin(theta + range);
    0   1   r*cos(theta + range)];

jGz = [ cos(theta + range) -r*sin(theta + range);
    sin(theta + range) r*cos(theta + range)];
end
