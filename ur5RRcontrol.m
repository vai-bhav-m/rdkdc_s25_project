function finalerr = ur5RRcontrol(gdesired, K, ur5)
% This function implements resolved-rate control for a UR5 robot to reach a desired pose.
% Inputs:
% - gdesired: 4x4 SE(3) homogeneous transform representing the desired end-effector pose
% - K: gain scalar
% - ur5: robot object with methods to read and move joints
% Output:
% - finalerr: final positional error (in cm) if converged, or -1 if aborted due to singularity

Tstep = 0.5/K;              % Time step for control update
thresh_v = 0.1;          % Threshold for convergence of linear component of Xi  (m)
thresh_w = 1;             % Threshold for convergence of angular component of Xi (rad)

while true
    q = ur5.get_current_joints();   
    Jb = ur5BodyJacobian(q);       

    % Jb Singularity check using inverse condition number (low means near singularity)
    mu = manipulability(Jb, 'invcond');
    if  mu < 0.01
        mu
        disp("ABORTING: configuration near a singularity")
        finalerr = -1;
        break
    end

    gst = compute_FK_DH(q);              
    Xi = getXi(pinv(gdesired) * gst);  % error twist between desired and current pose

    % Check for convergence: both rotational and translational components below threshold
    if (norm(Xi(1:3)) < thresh_v) && (norm(Xi(4:6)) < thresh_w)
        % disp("REACHED: converged to the desired configuration")
        finalerr = norm(gst(1:3,4) - gdesired(1:3,4)) * 100;  % Final position error in cm
        break
    end

    % Resolved-rate control law (euler integration): q_next = -K * pinv(Jb) * Xi * Tstep 
    q_next = q - K * Tstep * pinv(Jb) * Xi;
    ur5.move_joints(q_next, 5);      % Move robot to q_next
    pause(10);               
end