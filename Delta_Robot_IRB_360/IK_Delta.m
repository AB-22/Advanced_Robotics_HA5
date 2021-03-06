function [q, flag] = IK_Delta(pos_,param)


%coord_in = [x0,y0,z0];
e = param(1); %end effector 
f = param(2); %base
rf = param(4); %base arm
re = param(3); %parallelogram
%Coordinates of the end effector
y = pos_(2) - (e/(2*sqrt(3))); %shift center to edge
z = pos_(3);
x = pos_(1);

T = zeros(1,3);
A = zeros(1,3);
B = zeros(1,3);
k = [0,120,-120];
for i = 1:3
    T(i) = jointParam(pos_,k(i),param);
end
if (isnan(T(1)) || isnan(T(2)) || isnan(T(3)))
    flag = 1;
    q = zeros(1,9);
else
    flag = 0;
    %q = [T(1), T(2), T(3)];
    
    B(1)=asind(y/re);
    B(2)=asind((y*cosd(120)-x*sind(120))/re);
    B(3)=asind((y*cosd(120)+x*sind(120))/re);
    
    A(1)=acosd( ( -(rf*cosd(T(1))-x)*cos(T(1))+sqrt(-rf^2*cosd(T(1))^2+ ...
        2*rf*x*cosd(T(1))+re^2*cosd(B(1))^2-x^2)*sind(T(1)))/(re*cosd(B(1))));
    
    A(2)=acosd((-(rf*cosd(T(2))-(x*cosd(120)+y*sind(120)))*cosd(T(2))+sqrt(-rf^2*cosd(T(2))^2+...
        2*rf*(x*cosd(120)+y*sind(120))*cosd(T(2))+re^2*cosd(B(2))^2-(x*cosd(120)+y*sind(120))^2)*sind(T(2)))/...
        (re*cosd(B(2))));
    
    A(3)=acosd((-(rf*cosd(T(3))-(x*cosd(120)-y*sind(120)))*cosd(T(3))+sqrt(-rf^2*cosd(T(3))^2+...
        2*rf*(x*cosd(120)-y*sind(120))*cosd(T(3))+re^2*cosd(B(3))^2-(x*cosd(120)-y*sind(120))^2)*sind(T(3)))/...
        (re*cosd(B(3))));
    
    q=[T,A,B];
end

