function [] = Delta_workspace(param)
%Finding Workspace
wkspaceA = zeros((140/5)^3,3);
Singularity = zeros((140/5)^3,1);
n = 1;
for t1 = -38.84:5:94.65
    for t2 = -46.18:5:95.87
        for t3 = -46.18:5:95.87
            T = [t1,t2,t3];
            [pos_out,f] = FK_Delta(T,param);
            
            if f == 0
                    wkspaceA(n,1) = pos_out(1);
                    wkspaceA(n,2) = pos_out(2);
                    wkspaceA(n,3) = pos_out(3);
                    [q, flag]=IK_Delta(pos_out,param);
                    Jt = calc_Jt(q , param);
                    Jp = calc_Jp(q , param);
                    Jacob=pinv(Jt)*Jp;
                    m=sqrt(abs(det(Jacob*Jacob')));
                    Singularity(n)=m;
                    n = n+1;
            end
            
            
            
            
            
            
            
        end
    end
end

X = wkspaceA(:,1);
Y = wkspaceA(:,2);
Z = wkspaceA(:,3);
disp(wkspaceA);
hold on
%Initiate time sequence
t=0:pi/180:2*pi;
x=param(1)*cos(t); 
y=param(1)*sin(t); 
plot(x,y,'Linewidth',2);
%Indicate plotting the point rotated about z-axis (plotting a circle)
%plot3(X, Y, Z,'.','color', [0 0.75 0.75], 'MarkerSize',4)
scatter3(X, Y, Z,100,Singularity,'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
colormap jet
rotate3d on
axis equal 
title('Singularity Map')
%title('Workspace')
hold off    

min(X)
max(Y)
min(Z)
end