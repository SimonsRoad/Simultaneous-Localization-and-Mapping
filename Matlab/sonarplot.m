function plot = sonarplot(b,color,sign)
%Plots readings

if nargin==1 || nargin==2,
b(~any(b,2),:)=[]; %removes empty rows
scatter(b(:,1),b(:,2),color)
set(gca,'YDir','Reverse');
end
if nargin==3,
   b(~any(b,2),:)=[]; %removes empty rows
scatter(b(:,1),b(:,2),color,sign)
set(gca,'YDir','Reverse'); 
end
end

