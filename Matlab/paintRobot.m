function vRobotMap=paintRobot(posRobotXY,thetaRobot,tam,scale),
%
% Get points of robot in order to draw it using the fill fuction 
%
% vRobotMap=paintRobot(posRobotXY,thetaRobot,tam,scale);
%
% posRobotXY:   Position (x,y), center of robot (cm)
%               x are columns, y are rows.
% tethaRobot:   Course of robot (to x) (rad)
% optional params:
%       tam:    size of robot (lengthxwidth), by default (10x10)
%       scale: scale of map, by default (1 pixel = 1 cm.)

if nargin<2,
    disp('Error: number of params is wrong, use help');
    return;
end;
    
if nargin<3,
    tam=[10,10];
    scale=1;
end;

if nargin<4,
    scale=1;
end;

thetaRobot=thetaRobot+pi/2;

ROT =[ cos(thetaRobot)  sin(thetaRobot) 0 posRobotXY(1)
      -sin(thetaRobot)  cos(thetaRobot) 0 posRobotXY(2)
       0                  0               1     0 
       0                  0               0     1];
  
VBody=       [-tam(1)/2*scale  tam(2)/2*scale 0 1
               tam(1)/2*scale  tam(2)/2*scale 0 1
               tam(1)/2*scale -tam(2)/2*scale 0 1
              -tam(1)/2*scale -tam(2)/2*scale 0 1]';
          
vRobotMap=ROT*VBody;
