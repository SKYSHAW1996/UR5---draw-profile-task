g1=[0 -1 0 0.47;...
0 0 1 0.55;...
-1 0 0 0.12;...
0 0 0 1];
g2=[0 -1 0 -0.30;...
0 0 1 0.39;...
-1 0 0 0.12;...
0 0 0 1];
offset1 = [-pi/2 -pi/2 0 -pi/2 0 0]';

q1 = ur5InvKin(g1);
g1_index = ur5FwdKin(q1(:,1)-offset1)
g1_index = ur5FwdKin(q1(:,4)-offset1)
translation_index1 = g1_index(1:2,4) + [step;k1*step+b1]
g1_index(1:2,4) = translation_index1

translation_index1 = g1(1:2,4);threshold = 0.005;
g1_index = g1;step = 0.002;i=1;
while norm(translation_index1 - point1) > threshold
    q1 = ur5InvKin(g1_index);
%     ur5.move_joints(q1(:,1), 5);pause(5.5);%+offset
    disp(['drawing the ',num2str(i),'th dot on the first line']);
    i = i+1;
    g1_index = ur5FwdKin(q1(:,1)-offset);
    x_index1=g1_index(1,4)+step;
    y_index1=k1*x_index1+b1;
    translation_index1 = [x_index1;y_index1];
    plot(x_index1,y_index1,'*');hold on
    pause(1.5)
    g1_index(1:2,4) = translation_index1;
    disp(['position Z is ',num2str(g1_index(3,4))]);
end
disp('Finish drawing the first line');