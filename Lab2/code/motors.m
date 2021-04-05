function dx = motors(t,x,myrobot,splineqref, ...
                          splineqdref,splineqddref)
    % Computation of reference signal and its first two time derivatives
    qr=ppval(splineqref,t);
    qdr=ppval(splineqdref,t);
    qddr=ppval(splineqddref,t);

    % state vectors
    q = x(1:6);
    qd = x(7:12);
    
    for i = 1:6    
        link = myrobot.links(i);
        J(i,1) = link.Jm;
        B(i,1) = link.B;
    end

    % Controller parameters
    % Enter your gain matrices Kp and Kd here. Kp and Kd should be
    % diagonal matrices 6x6.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Kp = [8*10^-4 0 0 0 0 0; 0 8*10^-4 0 0 0 0; 0 0 8*10^-4 0 0 0; ...
        0 0 0 1.32*10^-4 0 0; 0 0 0 0 1.32*10^-4 0; 0 0 0 0 0 1.32*10^-4];
    Kd = [-6.8*10^-4 0 0 0 0 0; 0 -1.7*10^-5 0 0 0 0; 0 0 -5.8*10^-4 0 0 0; ...
        0 0 0 6.08*10^-5 0 0; 0 0 0 0 4.94*10^-5 0; 0 0 0 0 0 9.53*10^-5];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ft = J.*qddr + B.*qdr; % feedforward term
    Vt = ft + Kd*(qdr-qd) + Kp*(qr-q);   % feedback controller
    qdd = (Vt-B.*qd)./J;   % acceleration
    
    dx = [qd; qdd;];