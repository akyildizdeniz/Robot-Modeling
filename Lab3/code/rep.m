function tau = rep(q,myrobot,obs)
    % Repulsive torques calculation
    H = forward_jacobian(q, myrobot);
    eta = 0.01;
    tau = zeros(1,6);
    % Using the equations from preparation and textbook (5.6 and 5.7):
    for i = 1:6
        o_i(:,i) = H(1:3,4,i);
        if obs.type == 'cyl'
            oi_b_x = (o_i(1,i) - obs.c(1)) * (1 - (obs.R / norm(o_i(1:2,i) - obs.c)));
            oi_b_y = (o_i(2,i) - obs.c(2)) * (1 - (obs.R / norm(o_i(1:2,i) - obs.c)));
            oi_b = [oi_b_x; oi_b_y; 0];
            n = norm(o_i(1:2,i) - obs.c) - obs.R;
        end
        if obs.type == 'sph'
            oi_b = (o_i(:,i) - obs.c) * (1 - (obs.R / norm(o_i(:,i) - obs.c)));
            n = norm(o_i(:,i) - obs.c) - obs.R;
        end
        J = jacobian_i(q, myrobot,i);
        J_o_i = horzcat(J(1:3, 1:i),zeros(3,6-i));
        if n <= obs.rho0
            F_rep(:,i) = eta * ((1/n) - (1/obs.rho0)) * (1/(n*n)) * (oi_b/n);
        else
            F_rep(:,i) = zeros(3,1);
        end 
        tau = tau + (J_o_i' * F_rep(:,i))';
    end
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end
end

