function H = forward(joint, myrobot)
    % Calculates forward kinematics given joint angles and robot model.
    H = eye(4);
    
    % Loop to create individiual matrices from ith to (i-1)th reference
    % frames, and multiply them to get the forward kinematics matrix.
    for i = 1:6
        theta_i = joint(i);
        alpha_i = myrobot.alpha(i);
        a_i = myrobot.a(i);
        d_i = myrobot.d(i);
        H_i = [cos(theta_i) -sin(theta_i)*cos(alpha_i) ...
               sin(theta_i)*sin(alpha_i) a_i*cos(theta_i); ...
               sin(theta_i) cos(theta_i)*cos(alpha_i) ...
               -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i); ...
               0 sin(alpha_i) cos(alpha_i) d_i; ...
               0 0 0 1];
        H = H * H_i;
end

