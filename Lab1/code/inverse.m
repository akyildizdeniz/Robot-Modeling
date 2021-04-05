function q = inverse(H,myrobot)
    % Define end effector position and orientation.
    ee_position = H(1:3, 4);
    ee_orientation = H(1:3, 1:3);
    
    % Wrist centre position oc:
    oc = ee_position - (ee_orientation * ...
        [0; 0; myrobot.d(6)]);
    
    % Using the equations from preparation, 
    % find theta_1, theta_2, and theta_3
    gamma = asin(- myrobot.d(2) / sqrt(oc(1)^2 + oc(2)^2));
    theta_1 = atan2(oc(2), oc(1)) - gamma;
    D = ((oc(1)^2 + oc(2)^2)*cos(gamma)*cos(gamma) ...
        + ((oc(3) - myrobot.d(1))^2) ...
        - (myrobot.a(2))^2 - (myrobot.d(4))^2) / (2*myrobot.a(2)*myrobot.d(4));
    theta_3 = atan2(D, real(sqrt(1 - D^2)));
    psi_1 = atan2(myrobot.d(4)*sin(theta_3 - pi/2), ...
        myrobot.a(2) + myrobot.d(4)*cos(theta_3 - pi/2));
    psi_2 = atan2(oc(3) - myrobot.d(1), ...
        sqrt(oc(1)^2 + oc(2)^2)*cos(gamma));
    theta_2 = psi_2 - psi_1;
    
    % Assign q entries to be returned from this function.
    q(1) = theta_1;
    q(2) = theta_2;
    q(3) = theta_3;
    
    % After finding theta_1, theta_2, and theta_3, use equations from prep
    % and section 2.2 to first find homogenous transformation from frame
    % 0 to frame 3.
    H_3_0 = eye(4);
    for i = 1:3
        theta_i = q(i);
        alpha_i = myrobot.alpha(i);
        a_i = myrobot.a(i);
        d_i = myrobot.d(i);
        H_i = [cos(theta_i) -sin(theta_i)*cos(alpha_i) ...
               sin(theta_i)*sin(alpha_i) a_i*cos(theta_i); ...
               sin(theta_i) cos(theta_i)*cos(alpha_i) ...
               -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i); ...
               0 sin(alpha_i) cos(alpha_i) d_i; ...
               0 0 0 1];
        H_3_0 = H_3_0 * H_i;
    end
    
    % Extract rotation matrix R_3_0 from homogenous transformation matrix
    % from frame 0 to frame 3.
    R_3_0 = H_3_0 (1:3, 1:3);
    
    % Find the rotation matrix from frame 3 to frame 6 as suggested in 
    % section 2.2
    R_6_3 = R_3_0' * ee_orientation;
    
    % ZYZ Euler angle formulas from book page 55 to extract the rest of
    % the theta values.
    fi = atan2(R_6_3(2,3), R_6_3(1,3));
    theta = atan2(sqrt(1- R_6_3(3,3)^2), R_6_3(3,3));
    psi = atan2(R_6_3(3,2), -R_6_3(3,1));
    q(4) = fi;
    q(5) = theta;
    q(6) = psi;
end