ur5=ur5_interface();

% step 0
% define home
init = [pi/4, -pi/3, pi/2, -3*pi/4, -pi/2, pi/2]';

% step 1: 
disp('Move the robot to the first line start position');
waitforbuttonpress;
gs1 = ur5.get_current_transformation('base_link', 'ee_link');
pause();

% step 2:
disp('Move the robot to the first line final position')
waitforbuttonpress;
gt1 = ur5.get_current_transformation('base_link', 'ee_link');
pause();

% step 3: 
disp('Move the robot to the second line start position');
waitforbuttonpress;
gs2 = ur5.get_current_transformation('base_link', 'ee_link');
pause();

% step 4:
disp('Move the robot to the second line final position')
waitforbuttonpress;
gt2 = ur5.get_current_transformation('base_link', 'ee_link');
pause();
 condition = 1;

while (condition)
prompt = 'Enter 1 for Kinematic; 2 for Rate Control; 3 for Gradient Based Control, 4 for Extra Credit: ';
x = input(prompt);

if (x == 1)
    % in kin
    disp 'You chose Inverse Kinematics'
    InverseKin(ur5, init, gs1, gt1, gs2, gt2);
    condition = 0;

elseif(x == 2)
    % rr control
    disp 'You chose RR Control'
    RRControl(ur5, init, gs1, gt1, gs2, gt2);
    condition = 0;
    
elseif (x == 3)
    % gradient contorl
    disp 'You chose Gradient Based Control'
    GBControl(ur5, init, gs1, gt1, gs2, gt2);
    condition = 0;
    
elseif (x == 4)
    % extra credit
    disp 'You chose Extra Credit'
    ExtraCredit();
    condition = 0;
else
    disp 'Invalid Input, please try again.'
end
end

