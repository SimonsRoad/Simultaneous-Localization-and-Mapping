function beacons=beacon(pos)

%Generates beacon at given position

%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%color is the color of the beacon

x = pos(1); y = pos(2);
ang = 0:0.01:2*pi; 
r = 7;
xp = r*cos(ang);
yp = r*sin(ang);
color = 'w';
plot(x+xp,y+yp,color);
hold on;
plot(x,y,'*');
end