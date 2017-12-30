function [rangeObj,arc]=sonar(Map, sensor, rob, beta),

global DEBUG

posX=rob.x(end);
posY=rob.y(end);
orientRobot=rob.theta(end);

valtheta=(beta+orientRobot)-sensor.rangTetha/2:0.05:(beta+orientRobot)+sensor.rangTetha/2;
valrange=0:0.1:sensor.rangR;

if (DEBUG),
    ptoArcI=[posX+sensor.rangR*cos(valtheta(1));posY-sensor.rangR*sin(valtheta(1))]';
    ptoArcF=[posX+sensor.rangR*cos(valtheta(end));posY-sensor.rangR*sin(valtheta(end))]';
    figure(2),          
    line([posX, ptoArcI(1)], [posY, ptoArcI(2)])
    line([posX, ptoArcF(1)], [posY, ptoArcF(2)])
end;

obj=0;
rangeObj=-1; arc=[];
Mr=length(valrange);   k=1;
[f,c]=size(Map);
while ((obj==0)&(k<=Mr)),
    
    ptoArc1=[posX+valrange(k)*cos(valtheta);posY-valrange(k)*sin(valtheta)]';
    indMap=ceil(ptoArc1);
    r=Map(indMap(:,1)*c+indMap(:,2));
    
    if (sum(r)>0), 
        obj=1; 
        rangeObj=valrange(k);
        arc=ptoArc1;
        
        if (DEBUG), 
            figure(2),
            plot(ptoArc1(:,1),ptoArc1(:,2),'.r'); 
            
            line([posX, ptoArc1(1,1)], [posY, ptoArc1(1,2)])
            line([posX, ptoArc1(end,1)], [posY, ptoArc1(end,2)])
        end;
        
    else
        k=k+1;
    end;
    
end;

