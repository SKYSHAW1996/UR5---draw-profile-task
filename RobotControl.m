%Teach the manipulator the g_start and g_target
offset = [-pi/2, -pi/2, 0, -pi/2, 0 0]';
% [start_joints,target_joints] = Initialization();
% gstart  = ur5FwdKin(start_joints - offset);
% gtarget  = ur5FwdKin(target_joints - offset);
% disp('Get the gstart and gtarget');
ur5=ur5_interface();

% % Use these for test
gstart=[0 -1 0 0.47;
0 0 1 0.55;
-1 0 0 0.12;
0 0 0 1];
gtarget=[0 -1 0 -0.30;
0 0 1 0.39;
-1 0 0 0.12;
0 0 0 1];
ur5.move_joints(offset, 5); %gst0 configuration
pause(5.5)

%Compute the other two end points on the parallel lines
%point1 is on the first line including point_start, 
%point2 is on the second line including point_target.
%k1,b1 the slope & intercept between point_start and point1
%k2,b2 the slope & intercept between point_target and point2
[point1,point2,k1,b1,k2,b2] = ComputePoints(gstart,gtarget);
disp('Get the point1,point2,k1,b1,k2,b2');
disp('Please press any button to start the task');
waitforbuttonpress;

% Drawing the first line from the gstart
%define the start point of the first line
translation_index1 = gstart(1:2,4);g1_index = gstart;
%set the parameters
threshold = 0.006;step = 0.01;i=1;dist=1;
%Set the robot to the start position on the first line
q1 = ur5InvKin(g1_index);
ur5.move_joints(q1(:,1), 5);pause(5.5);
plot(g1_index(1,4),g1_index(2,4),'*');hold on
disp('Please attach a pen on the end_effector before pressing any button');
waitforbuttonpress;
%Start drawing 
while dist > threshold
    x_index1=g1_index(1,4)-step;
    y_index1=k1*x_index1+b1;
    translation_index1 = [x_index1;y_index1];
    g1_index(1:2,4) = translation_index1;
    dist=norm(translation_index1 - point1);
    
    q1 = ur5InvKin(g1_index);
    ur5.move_joints(q1(:,1), 2);pause(2.5);
    g1_index = ur5FwdKin(q1(:,1)-offset);
    i = i+1;
    %for debug
    disp(['drawing the ',num2str(i),'th dot on the first line']);    
    disp(['position Z is ',num2str(g1_index(3,4))]);
    plot(x_index1,y_index1,'*');hold on
    disp(dist);
end
disp('Finish drawing the first line');

%Move the end_effector to leave the workbench
g1_index(3,4) = g1_index(3,4) + 0.04;
q = ur5InvKin(g1_index);
ur5.move_joints(q(:,1), 2.5);pause(3);
disp('Please press any button to start drawing the second line');
waitforbuttonpress;

%define the start point of the second line
g2=gtarget;
gtarget(1:2,4)=point2;
g2_index = gtarget;gtarget = g2;
% Drawing the second line to the gtarget
%Set robot to the position above start position 0.04m on the second line
g2_index(3,4) = g2_index(3,4) + 0.04;
q2 = ur5InvKin(g2_index);
ur5.move_joints(q2(:,1), 4);pause(4.5);
%Set robot to the start position on the second line
g2_index(3,4) = g2_index(3,4) - 0.04;
q2 = ur5InvKin(g2_index);
ur5.move_joints(q2(:,1), 4);pause(4.5);

plot(g2_index(1,4),g2_index(2,4),'^');hold on
translation_index2 = point2;i=1;dist2=1;
%Start drawing
while dist2 > threshold
    x_index2=g2_index(1,4)-step;
    y_index2=k2*x_index2+b2;
    translation_index2 = [x_index2;y_index2];
    g2_index(1:2,4) = translation_index2;
    
    q2 = ur5InvKin(g2_index);
    ur5.move_joints(q2(:,1), 2);pause(2.5);%+offset
    i = i+1;
    g2_index = ur5FwdKin(q2(:,1)-offset);
    %for debug
    disp(['drawing the ',num2str(i),'th dot on the first line']);
    dist2=norm(translation_index2 - gtarget(1:2,4));
    disp(['position Z is ',num2str(g2_index(3,4))]);
    plot(x_index2,y_index2,'^');hold on
    disp(dist2);
end
disp('Finish drawing the second line');

ur5.move_joints(offset, 5); %gst0 configuration
pause(5.5)