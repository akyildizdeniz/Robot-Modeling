function Ja = ajacobian(joint, myrobot)
    % Function to compute the analytic jacobian 
    % given the robot structure and joint variables
    H = forward(joint, myrobot);
    R = H(1:3, 1:3); % could not use inv. kin. here since euler angles are different
    fi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(1- R(3,3)^2), R(3,3));
    Jg = jacobian(joint, myrobot);
    B = [0, -sin(fi), cos(fi) * sin(theta); ...
         0, cos(fi), sin(fi) * sin(theta); ...
         1, 0, cos(theta)];
    Ja = ([eye(3) zeros(3,3); zeros(3,3) inv(B)] * Jg);
end

