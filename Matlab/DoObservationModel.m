function [z] = DoObservationModel(xVehicle, xFeature)
%Get the distance to the beacons

DeltaObs = xFeature-xVehicle(1:2);
z = [norm(DeltaObs); %Normalize (x,y)
    atan2(DeltaObs(2),DeltaObs(1))-xVehicle(3)]; 
z(2) = AngleWrapping(z(2));
end