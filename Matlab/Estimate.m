function Est=Estimate(pos,u)
T = 1.0; %time sampling

result = pos(3)+u(3);  %Final angle

if result > pi || result <= -pi
   result = AngleWrapping(result) ;
end

v = rand(2,1); %noise

s = sin(pos(3))/2;
c = cos(pos(3))/2;

%Actual value added with the new control vector
Est = [pos(1:2)+ T*[c c; -s -s]*u(1:2)+v;
       T*result];