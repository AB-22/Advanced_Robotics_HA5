%calculates joint variable
function [theta] = jointParam(pos_,k,param)
rot_Mat = [cosd(k), -sind(k), 0; sind(k), cosd(k), 0; 0, 0, 1];
%coord_in = [x0,y0,z0];
coord_param = rot_Mat*(pos_)';
e = param(1); %end effector 
f = param(2); %base
rf = param(4); %base arm
re = param(3); %parallelogram
%Coordinates of the end effector
y = coord_param(2) - (e/(2*sqrt(3))); %shift center to edge
z = coord_param(3);
x = coord_param(1);
%projected point on yz plane
y1 = -0.5 * f / sqrt(3);  % - f/2 * tan(30)

a = (x^2 + y^2 + z^2 + rf^2 - re^2 - y1^2)/(2*z);
b = (y1-y)/(z);

%discriminant
d = -(a+b*y1)^2 + rf*(rf*b^2 + rf);
if d < 0
    
    
    theta = nan;
else
    
    y_joint = (y1 - a*b - sqrt(d))/(b^2 + 1); % choosing outer point
    z_joint = a + b*y_joint;
    theta = 180*atan(-z_joint/(y1 - y_joint))/pi;
    
    if y_joint>y1
        theta = theta + 180;
    end
end

end
