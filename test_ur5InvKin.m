% test_ur5InvKin allows students to test the accuracy of their ur5FwdKin.m functions
% Author: Jakub Piwowarczyk, 08/27/2023

% initialize an array to store the errors
errors = zeros(6,1);

% select the number of tests to ensure many configurations work
n_tests = 1000;

for i = 1:n_tests
    % generate a random set of joint angles between -pi and pi
    q = (rand(6,1) * 2*pi) - pi;
    
    % calculate the tool0 pose using your forward kinematics
    gBaseTool = ur5FwdKin(q);
    
    % calculate the set of solution using ur5InvKin
    q_sol = ur5InvKin(gBaseTool);
    
    % find the closest matching kinematic configuration
    [min_error, min_error_i] = min(vecnorm(q - q_sol,1));
    
    % calculate the errors
    errors = errors + abs(q_sol(:,min_error_i) - q);
end

% print the errors
fprintf("The Average Joint Errors are: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",errors/n_tests)
