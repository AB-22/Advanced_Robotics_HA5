function Jp = calc_Jp(q , param)

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

J1=[ cosd(theta1+alpha1) , -tand(beta1) , sind(theta1+alpha1)]';
J2=[ cosd(theta2+alpha2) , -tand(beta2) , sind(theta2+alpha2)]';
J3=[ cosd(theta3+alpha3) , -tand(beta3) , sind(theta3+alpha3)]';

Jp=[J1 , J2 , J3];

end