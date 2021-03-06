function MAIN ()
%access to the image file
m = '0.jpg';
image = profile(m);
[row,col] = size(image);

%Initialize the ur5 manipulator
ur5=ur5_interface();
ur5.move_joints(offset, 5); %gst0 configuration
pause(5.5)
disp('Please press any button to start the drawing task');
waitforbuttonpress;

%Define the index
translation_index = gstart(1:2,4);g_index = gstart;
%set the parameter:  Step defines the distance between pixels
step = 0.003; time_interval = 2;
%Set the robot to the start position
g_index(3,4) = g_index(3,4) + 0.04;
q = ur5InvKin(g_index);
ur5.move_joints(q(:,1), 5);pause(5.5);
g_index = ur5FwdKin(q(:,1)-offset);
disp(g_index(3,4));
plot(g_index(1,4),g_index(2,4),'*');hold on
disp('Please attach a pen on the end_effector before pressing any button');
waitforbuttonpress;

%Start drawing 
x_origin = g_index(1,4);
y_origin = g_index(2,4);
for n = 1:col
    %set the value of x
    x_index=x_origin+step*(n-1);
    for m = 1:row
        %set the value of y
        y_index=y_origin-step*(m-1);
        %get the translation part
        if image(m,n) == 0
            %set the drawing dot
            translation_index = [x_index; y_index];
            g_index(1:2,4) = translation_index;
            %move the robot to the dot position & above the workbench 0.04m
            q = ur5InvKin(g_index);
            ur5.move_joints(q(:,1), time_interval);pause(time_interval);
            
%             g_index = ur5FwdKin(q(:,1)-offset);
%             disp(g_index(3,4));
            %draw the dot and then leave the workbench
            g_index(3,4) = g_index(3,4) - 0.04;
            q = ur5InvKin(g_index);
            ur5.move_joints(q(:,1), time_interval);pause(time_interval);
            
%             g_index = ur5FwdKin(q(:,1)-offset);
%             disp(g_index(3,4));
            
            g_index(3,4) = g_index(3,4) + 0.04;
            q = ur5InvKin(g_index);
            ur5.move_joints(q(:,1), time_interval);pause(time_interval);
            %update the g_index
            g_index = ur5FwdKin(q(:,1)-offset);
            
            %for debug
            disp(['drawing the ',num2str(m),' ',num2str(n),'th dot on the first line']);    
            disp(['position Z is ',num2str(g_index(3,4))]);
            plot(x_index,y_index,'*');hold on
        end
    end
end
disp('Finish drawing the profile');
end
