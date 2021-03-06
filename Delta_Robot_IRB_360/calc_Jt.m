function Jt = calc_Jt(q , param)

e = param(1); %end effector frame
f = param(2); %base frame
rf = param(4); %base arm
re = param(3); %parallelogram

%[theta1,theta2,theta3,alpha1,alpha2,alpha3, beta1,beta2,beta3]=q;
theta1=q(1);
theta2=q(2);
theta3=q(3);
alpha1=q(4);
alpha2=q(5);
alpha3=q(6);
beta1=q(7);
beta2=q(8);
beta3=q(9);

J1 = -rf*(sind(theta1)*cosd(theta1+alpha1)-cosd(theta1)*sind(theta1+alpha1));
J2 = -rf*(sind(theta2)*cosd(theta2+alpha2)-cosd(theta2)*sind(theta2+alpha2));
J3 = -rf*(sind(theta3)*cosd(theta3+alpha3)-cosd(theta3)*sind(theta3+alpha3));

Jt=diag( [J1 , J2 , J3]);

end