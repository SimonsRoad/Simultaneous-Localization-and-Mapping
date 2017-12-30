clear all; close all;
%-------------------------- INPUTS AND VARIABLES --------------------------
% Read a map: imagen BMP
M=imread('maps/map2.bmp');
M=~M; % Objets = 1, vacio 0
bplot=[0 0]; b =[]; bmeasure = 0;   %saving newly found beacons
scannings=[];   %scanings of map, not beacons
srange=[];      %saving sonar object range
sangle=[];      %saving sonar angle
saved_true=[];           %saving true robot movement
saved_estimate=[];       %saving predicted robot movement
P = diag([1 1 0.01]); %initial covariance, we know where we begin
saved_position_error=[0];          %saved error over time
iFeature=[];
Condition=0;
time_measure=0;
%--------------------------------------------------------------------------

%--------------------------- SETUP BEACON DATA ----------------------------
global xVehicleTrue;
global LandFeatures;
global UTrue;

c=[];
%--------------------------- SETUP BEACON DATA ----------------------------
c = [c; 83,320]; c = [c; 168,320]; c = [c; 225,217];
c = [c; 290,320]; c = [c; 381,218]; c = [c; 296,150];
%--------------------------------------------------------------------------

%------------------------ CREATE BEACON VARIABLES -------------------------
LandFeatures = zeros(2,1,size(c,1)); MappedLandFeatures = NaN*zeros(size(c,1),2);
for nb=1:size(c,1)
    LandFeatures(:,:,nb)=c(nb,:);           %create landmark measure
end
for nd=1:6
  eval(sprintf('d%d = [];', nd));            %create distance variables
end
for nb2=1:size(c,1)
  eval(sprintf('c%d = [c(nb2,:)];', nb2));   %create beacon co. variables
end
pnt=[];
%--------------------------------------------------------------------------

%------------------------------ MODEL ROBOT -------------------------------
rob.Nc=100; % Encoder: total pulses 
rob.Rb=5;   % wheel radius (cm)
rob.b=10;   % Distance between wheels (cm)
rob.x(1)=130; rob.y(1)=400;  % Position of robot
rob.theta(1)=pi/2;% Orient of robot
saved_true = [rob.x(1) rob.y(1) rob.theta(1)]; saved_estimate=[rob.x(1) rob.y(1) rob.theta(1)];
current_estimate = [rob.x(1); rob.y(1); rob.theta(1)];
%--------------------------------------------------------------------------

%--------------------------- MODEL SENSOR DATA ----------------------------
sensor.rangTetha=5*pi/180;    % Beam: Width (rad)
sensor.rangR=100;               % Beam: Range (cm)
sensor.epsil=10;               % error (cm)
% -------------------------------------------------------------------------

%----------------------------- ROBOT MOTION -------------------------------
% Example. The robot get encoders readings when it moves 
% Readings of encoder sensors (Nl, left encoder, Nr right encoder):
Nr=[20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20  0  0  0 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 16 17 17 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 16 17 17 16 17 17 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20  0  0  0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20 16 17 17 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20 20 20 20];
Nl=[20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 16 17 17 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20  0  0  0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20  0  0  0  0  0  0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 16 17 17 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20  0  0  0 20 20 20 20 20 20 20 20 20 ...
    20 20 20 20 20 20 20 20 20 20 20 20 20 20 20];

%--------------------------------------------------------------------------

%------------------------------ INITIATE MAP ------------------------------
figure(1)
image(M*100);
hold on;
for nb3=1:size(c)
    beacon(c(nb3,:));      %draw beacons
end
%--------------------------------------------------------------------------

%------------------------------ PLOT ROBOTS -------------------------------
%Plot initial position of robot:
figure(1)
V=paintRobot([rob.x(1),rob.y(1)],rob.theta(1),[10,40]);
r=fill(V(1,:),V(2,:),[0.100 0.100 0.100]);

%Plot initial position of predicted robot
Q=paintRobot([rob.x(1),rob.y(1)],rob.theta(1),[10,40]);
q=fill(Q(1,:),Q(2,:),'r');
%--------------------------------------------------------------------------

%--------------------------------- MAIN LOOP ------------------------------
pause;
for i=1:length(Nl)
% ----- Move robot (motion model) ----------    
Dr=(2*pi*rob.Rb)*(Nr(i)/rob.Nc);
Dl=(2*pi*rob.Rb)*(Nl(i)/rob.Nc);
D=(Dr+Dl)/2;

rob.x(i+1)=rob.x(i)+D*cos(rob.theta(i));
rob.y(i+1)=rob.y(i)-D*sin(rob.theta(i));  %We use "-" because y axis is reverse en the plot

Dtheta=(Dr-Dl)/rob.b;
rob.theta(i+1)=rob.theta(i)+Dtheta;
% --------------------------------------

% ------ Scan round robot --------------
thetaScan=0:30:360; % sonar moves around the robot 
%thetaScan=0;       % sonar fixed to front robot 

slam = 0;
for k=thetaScan,
    [rangeObj,pts]=sonar(M, sensor, rob,k);
    if (rangeObj>0),
        scannings=[scannings;pts(end,:)];  pnt=[pnt;pts(2,:)]; ppp = pnt(end,:);
        %------------------------------ FOUND BEACON CONDITION -----------------------------
        for dmeas = 1:size(c,1) %for all landmarks
            variable = eval(sprintf('d%d',dmeas));
            c_i = eval(sprintf('c%d',dmeas));
            current_distance = sqrt((ppp(1)-c_i(1))^2+(ppp(2)-c_i(2))^2);
            eval(sprintf('d%d = [variable; current_distance];',dmeas));
        end
        %-----------------------------------------------------------------------------------

        %------------------------------------- PLOT SONAR ----------------------------------
        sangle=[sangle;k]; srange=[srange;rangeObj];
        a=plot(pts(:,1),pts(:,2),'.r');
        pause(0.01)
        delete(a)
        %-----------------------------------------------------------------------------------
    end
end;
%--------------------------- CHECK IF BEACON FOUND & RUN SLAM ---------------------------
%add points found to saved vector b and add index to feature (beacon)
for ii=1:length(d1),
    if d1(ii)<=9 || d2(ii)<=9 || d3(ii)<=9 || d4(ii)<=9 || d5(ii)<=9 || d6(ii)<=9
         if d1(ii)<=9
            iFeature = [iFeature; 1];
            fprintf('Found beacon 1 at time: %i.\n',time_measure)
         end
         if d2(ii)<=9
            iFeature = [iFeature; 2];
            fprintf('Found beacon 2 at time: %i.\n',time_measure)
         end
         if d3(ii)<=9
            iFeature = [iFeature; 3];
            fprintf('Found beacon 3 at time: %i.\n',time_measure)
         end
         if d4(ii)<=9
            iFeature = [iFeature; 4];
            fprintf('Found beacon 4 at time: %i.\n',time_measure)
         end
         if d5(ii)<=9
            iFeature = [iFeature; 5];
            fprintf('Found beacon 5 at time: %i.\n',time_measure)
         end
         if d6(ii)<=9
            iFeature = [iFeature; 6];
            fprintf('Found beacon 6 at time: %i.\n',time_measure)
         end
        b = [b; pnt(ii,:)]; bplot=[bplot; pnt(ii,:)];
        bmeasure = bmeasure+1;
    end
end

%------------------------------------- SLAM ---------------------------------------
time_measure = time_measure+0.1;
[X2, P2, MappedLandFeatures2] = SLAM([rob.x(end) rob.y(end) rob.theta(end)],current_estimate,[D Dr Dl],b,P, Dtheta,iFeature,MappedLandFeatures);
saved_estimate = [saved_estimate; X2(1) X2(2) X2(3)]; saved_true=[saved_true; rob.x(end) rob.y(end) rob.theta(end)];
current_estimate = [X2];
P=P2;
MappedLandFeatures = MappedLandFeatures2;
%----------------------------------------------------------------------------------------

%-------------------------------- UPDATING VARIABLES ------------------------------------
for nqq=1:size(c,1)
  eval(sprintf('d%d = [];', nqq));            %update distance variables
end
pnt = []; b = []; iFeature=[]; Condition=0;
%----------------------------------------------------------------------------------------

%-------------------------- PLOT UPDATED ROBOT POSITION ---------------------------------
delete(r); delete(q) % Clear last real and predicted robot

% Draw new real robot 
V=paintRobot([rob.x(i+1),rob.y(i+1)],rob.theta(i+1),[10,40]);
r=fill(V(1,:),V(2,:),[0.100 0.100 0.100]);

%Draw new predicted robot
Q=paintRobot([X2(1),X2(2)],X2(3),[10,40]);
q=fill(Q(1,:),Q(2,:),'r');    
% ----------------------------------------------------------------------------------------
end;
% ------------------------------------ PLOT RESULTS ---------------------------------------
figure;
subplot(2,2,1);
set(gca,'YDir','Reverse')
sonarplot(bplot,'r','*'); hold on
sonarplot(scannings,'b');
axis([0 500 0 500])
title('Map and Beacons'); 
legend('Beacons','Other Scannings', 'Location', 'northwest'); legend('boxoff');

xlabel('cm'); ylabel('cm');

subplot(2,2,2)
plot(saved_true(:,1), saved_true(:,2),'b'); hold on
plot(saved_estimate(:,1), saved_estimate(:,2),'r');
axis([0 500 0 500])
set(gca,'YDir','Reverse')
title('True and Predicted Path');
legend('True Position','Estimate Position', 'Location', 'northwest'); legend('boxoff');
xlabel('cm'); ylabel('cm');

subplot(2,2,3)
error = saved_true-saved_estimate;
plot(0.1*colon(1,size(error,1))',error(:,1)); grid on;
title('X-Position Error Over Time');
legend('Error', 'Location', 'northwest'); legend('boxoff');
xlabel('Time [T]'); ylabel('cm'); grid on;

subplot(2,2,4)
plot(0.1*colon(1,size(error,1))',error(:,2)); grid on;
title('Y-Position Error Over Time');
legend('Error', 'Location', 'northwest'); legend('boxoff');
xlabel('Time [T]'); ylabel('cm'); grid on;

%total position error
figure
err_x = error(:,1);
err_y = error(:,2);
err_tot = sqrt(err_x.^2+err_y.^2);
plot(0.1*colon(1,size(error,1))',err_tot)
title('Total Position Error Over Time');
legend('Error', 'Location', 'northwest'); legend('boxoff');
xlabel('Time [T]'); ylabel('cm'); grid on;
%----------------------------------------------------------------------------------------

