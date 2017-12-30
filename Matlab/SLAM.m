function [xEst,PEst,MappedLandFeatures] = SLAM(rob,pos,D,b,P, Dtheta,iFeature,MappedLandFeatures)
global xVehicleTrue;
global LandFeatures;
global UTrue;

% -------------------------- INPUTS AND VARIABLES -------------------------
V = D(1); %Velocity used for control vector
xVehicleTrue = rob'; %Start position (x, y, theta)
z = [];

xEst = pos; %Estimated position
PEst = P;   %Estimated covariance
ObsNoise =0.5*randn(2,1); %White noise

UTrue = diag([1,1,1.5*pi/180]).^2; %Standard deviation
RTrue = diag([1.1,5*pi/180]).^2;         %Standard deviation error

UEst = 0.5*UTrue;
REst = 0.5*RTrue;
   
u = [V;V;Dtheta]; %Control vector

xVehicle = xEst(1:3); %Position estimate only
xMap = xEst(4:end);   %Feature Estimate only
%--------------------------------------------------------------------------

%------------------------------ UPDATING STEP -----------------------------
xVehiclePred = Estimate(xVehicle,u); %Start update position

%Start update covariance
PPvv = J1(xVehicle,u)* PEst(1:3,1:3) *J1(xVehicle,u)' + J2(xVehicle,u)* UEst * J2(xVehicle,u)';
PPvm = J1(xVehicle,u)*PEst(1:3,4:end);
PPmm = PEst(4:end,4:end);

%Finalize Updating
xPred = [xVehiclePred;xMap]; 
PPred = [PPvv PPvm; 
        PPvm' PPmm];
%--------------------------------------------------------------------------

%--------------- CORRECTION STEP & LANDMARK INITIALIZATION ----------------
if size(iFeature) == 0
    z =[];
else
    z = DoObservationModel(xVehicleTrue,b(end,:)')+sqrt(RTrue)*ObsNoise; %Get z from measurement
end
if(~isempty(z)) %If beacon found:
    if( ~isnan(MappedLandFeatures(iFeature(end),1)))%If beacon already found:
        %----------------------- CORRECTION STEP --------------------------
        FeatureIndex = MappedLandFeatures(iFeature(end),1);
        xFeature = xPred(FeatureIndex:FeatureIndex+1);

        zPred = DoObservationModel(xVehicle,xFeature);

        [jHv,jHf] = GetObsJacs(xVehicle,xFeature);

        H = zeros(2,length(xEst));
        H(:,FeatureIndex:FeatureIndex+1) = jHf;
        H(:,1:3) = jHv;

        Innov = z-zPred;
        Innov(2) = AngleWrapping(Innov(2));

        S = H*PPred*H'+REst;
        W = PPred*H'*inv(S); 
        xEst = xPred+ W*Innov;

        PEst = PPred-W*S*W';
        PEst = 0.5*(PEst+PEst');  %Make sure that P remains symmetric
        %------------------------------------------------------------------
    else  %If found new beacon
        %-------------------- LANDMARK INITIALIZATION ---------------------
        nStates = length(xEst); 
        xFeature = xVehicle(1:2)+ [z(1)*cos(z(2)+xVehicle(3));z(1)*sin(z(2)+xVehicle(3))];
        xEst = [xEst;xFeature]; %add to state vector estimated
                
        % compute jacobians regarding feature and observation 
        [jGx, jGz] = GetNewFeatureJacs(xVehicle,z); 

        M = [eye(nStates), zeros(nStates,2);
        jGx zeros(2,nStates-3)  , jGz];

        PEst = M*blkdiag(PEst,REst)*M';

        MappedLandFeatures(iFeature(end),:) = [length(xEst)-1, length(xEst)];
        %------------------------------------------------------------------
    end
%--------------------------------------------------------------------------

%---------------------------- NO BEACON FOUND -----------------------------
else  %No beacon found
xEst = xPred;
PESt = PPred;
end
%--------------------------------------------------------------------------
end