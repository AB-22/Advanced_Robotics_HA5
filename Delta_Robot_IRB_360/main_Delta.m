clc;
clear all;
close all;

%% Parameters
%link lengths are in m:
rf = 0.3;  %Base to Joint
re = 0.8;  %Joint to endeffector

%triangular side length in m:
f =0.567;  %base
e = 0.076; %endeffector
param=[e,f,re,rf];
%% Robot visualization
r0=[0, 0, -1.1]; %start Pose
[q, flag] = IK_Delta(r0,param);
t=q(1:3);
plot_Delta_Robot( r0',t, param ) % Plot Robot at start Pose
%% Test IK_Delta Function
T=[45 , 20 , 60];
[r,flag]=FK_Delta(T,param);
[angls, flag]=IK_Delta(r,param);
disp(angls(1:3))
disp(' Error is')
disp(abs(((T-angls(1:3)))))
%% Test FK_Delta Function
rr=[0.2 , 0.15 , -0.85];
[angles, flag1]=IK_Delta(rr,param);
[posi,flag2]=FK_Delta(angles(1:3),param);
disp(' Error is')
disp(abs(((rr-posi))))

%%  Drawing Singularities Map
Map_Singularities(param);