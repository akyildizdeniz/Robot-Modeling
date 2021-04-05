function J = jacobian(joint, myrobot)
    % Function to compute the geometric jacobian 
    % given the robot structure and joint variables
    H_all = forward_jacobian(joint, myrobot);
    o_0_6 = H_all(1:3,4,6);
    J_v = zeros(3,6);
    J_w = zeros(3,6);
    for i = 1:6
        if i == 1
            o = zeros(3,1);
            z = [0;0;1];
        else
            o = H_all(1:3,4,i-1);
            z = H_all(1:3,3,i-1);
        end
        Jv_i = cross(z, (o_0_6 - o));
        Jw_i = z;
        J_v(:,i) = Jv_i;
        J_w(:,i) = Jw_i;
    end
    J = [J_v ; J_w];
end

