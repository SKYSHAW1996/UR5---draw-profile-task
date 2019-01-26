% function InverseKin(ur5, home, gstart1, gtarget1, gstart2, gtarget2)

%test
ur5 =  ur5_interface();

gstart1=[0 -1 0 0.5;
0 0 1 0.55;
-1 0 0 0.12;
0 0 0 1];

gtarget1=[0 -1 0 0.45;
0 0 1 0.55;
-1 0 0 0.12;
0 0 0 1];

gstart2=[0 -1 0 0.5;
0 0 1 0.45;
-1 0 0 0.12;
0 0 0 1];

gtarget2=[0 -1 0 0.45;
0 0 1 0.45;
-1 0 0 0.12;
0 0 0 1];

offset = [-pi/2, -pi/2, 0, -pi/2, 0 0]';

%rotate about y-axis for 90
r_x = [1  0 0;
       0 -1 0;
       0  0 -1];
r_y = [0 0 1;
        0 1 0;
        -1 0 0];
    
r_z = [-1 0 0;
        0 -1 0;
        0  0 1];
  
home = [pi/4, -pi/3, pi/2, -3*pi/4, -pi/2, pi/2]';

%Compute the other two end points on the parallel lines
%point1 is on the first line including point_start, 
%point2 is on the second line including point_target.
%k1,b1 the slope & intercept between point_start and point1
%k2,b2 the slope & intercept between point_target and point2
[point1,point2,k1,b1,k2,b2] = ComputePoints(gstart1,gtarget1,gstart2,gtarget2);
disp('Get the point1,point2,k1,b1,k2,b2');
disp('Please press any button to start the task');
waitforbuttonpress;

% Drawing the first line from the gstart
% define the first line start position and other parameters
gstart1(1:3,1:3)= gstart1(1:3,1:3) * r_y;
translation_index1 = gstart1(1:2,4);
g1_index = gstart1;
threshold = 0.006;step = 0.01;i=1;dist=1;
q1 = ur5InvKin(g1_index);
ur5.move_joints(home, 5);
pause(5)

%% Move to home position and attach a pen

pause(5.5);
disp('Please attach a pen on the end_effector before pressing any button');
waitforbuttonpress;

% Move the end-effector to 4cm above the first-line start position
g1_above = gstart1;
g1_above(3,4) = gstart1(3,4) + 0.04;
q1_above = ur5InvKin(g1_above);
ur5.move_joints(q1_above(:,1), 5);
pause(5.5);

while dist-threshold > 0
    x_index1=g1_index(1,4) - step
    y_index1=k1*x_index1+b1
    translation_index1 = [x_index1;y_index1];
    g1_index(1:2,4) = translation_index1;
    dist = norm(translation_index1 - point1)
    
    q1 = ur5InvKin(g1_index);
    ur5.move_joints(q1(:,1), 5);
    pause(2.5);
    g1_index = ur5FwdKin(q1(:,1)-offset);
    i = i+1;     
    plot(x_index1,y_index1,'*');
    hold on
%     %for debug
%     disp(['drawing the ',num2str(i),'th dot on the first line']);    
     disp(['position Z is ',num2str(g1_index(3,4))]);
%     disp(dist);
end
disp('Finish drawing the first line');
ur5.move_joints(q1_above(:,1), 5);
pause(5);
ur5.move_joints(home, 5);

% define the second line start position and other parameters
gstart2(1:3,1:3)= gstart2(1:3,1:3) * r_y;
translation_index2 = point2;i=1;dist2=1;
g2_index = gstart2;

% Move the end-effector to 4cm above the first-line start position
g2_above = gstart2;
g2_above(3,4) = gstart2(3,4) + 0.04;
q2_above = ur5InvKin(g2_above);
ur5.move_joints(q2_above(:,1), 5);
pause(5.5);

disp('Please press any button to start drawing the second line');
waitforbuttonpress;

%Start drawing
while dist2 - threshold > 0
    x_index2=g2_index(1,4)-step;
    y_index2=k2*x_index2+b2;
    translation_index2 = [x_index2;y_index2];
    g2_index(1:2,4) = translation_index2;
    dist2=norm(translation_index2 - point2);
    
    q2 = ur5InvKin(g2_index);
    ur5.move_joints(q2(:,1), 5);
    pause(2.5);%+offset
    g2_index = ur5FwdKin(q2(:,1)-offset);
%     %for debug
%     i = i+1;
%     disp(['drawing the ',num2str(i),'th dot on the first line']);
%     disp(['position Z is ',num2str(g2_index(3,4))]);
     plot(x_index2,y_index2,'^');hold on
%     disp(dist2);
end
disp('Finish drawing the second line');
pause(5);
ur5.move_joints(q2_above(:,1), 5);
ur5.move_joints(home, 5); 
pause(5.5)