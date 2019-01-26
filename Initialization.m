%Initialization
function [start_joints,target_joints] = Initialization()

ur5=ur5_interface();
disp('Please press any button to set the start configuration')
w = waitforbuttonpress;
start_joints=ur5.get_current_joints();
disp('Get the start configuration')
disp('Please press any button to set the end configuration')
w = waitforbuttonpress;
target_joints=ur5.get_current_joints();
disp('Get the end configuration')

end


