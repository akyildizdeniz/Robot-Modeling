%% Definition of robot structure
% DH Table 
% All units in cm.
DH = [0, 76, 0, pi/2; ...
      0, -23.65, 43.24, 0; ...
      0, 0, 0, pi/2; ...
      0, 43.18, 0, -pi/2; ...
      0, 0, 0, pi/2; ...
      0, 20, 0, 0];

% Create the robot model.
myrobot = mypuma560(DH);

%% Preparation Part 3
% Geometric Jacobian

% Create q matrix that consists of joint angles.
q = [pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6];

% Verify jacobian
disp('Verify geometric jacobian:')
jacobian(q, myrobot)

%% Preparation Part 4
% Analytic Jacobian

% Create q matrix that consists of joint angles.
q = [pi/4 pi/3 -pi/2 pi/4 pi/6 -pi/6];

% Verify analytic jacobian
disp('Verify analytic jacobian:')
ajacobian(q, myrobot)

%% Experiment Part 4.1
% Computation of joint reference signals

% Part 1: Compute qref
load('Href.mat')
qref_tmp = zeros(100,6);
for i = 1:100
    H = Href(:,:,i);
    qref_tmp(i,:) = inverse(H, myrobot);
end
qref = qref_tmp';

% Part 2.a: Compute odot and omega
R_ref = Href(1:3,1:3,:);
R_ref_dot = Hrefdot(1:3,1:3,:);
omega = zeros(3,100);
odot = zeros(3,100);
for i = 1:100
    odot_i = Hrefdot(1:3,4,i);
    odot(:,i) = odot_i;
    R_ref_i = R_ref(:,:,i);
    R_ref_dot_i = R_ref_dot(:,:,i);
    S_w_i = R_ref_dot_i * (R_ref_i');
    omega_i = [S_w_i(3,2); S_w_i(1,3); S_w_i(2,1)];
    omega(:,i) = omega_i;
end

% Part 2.b: Compute the derivative of qref
qdref = zeros(6,100);
for i = 1:100
    J_i = jacobian(qref(:,i), myrobot);
    twist_i = [odot(:,i); omega(:,i)];
    qdref(:,i) = J_i\twist_i;
end


% Part 2.c: Find euler angles
fi = zeros(1,100);
theta = zeros(1,100);
psi = zeros(1,100);
for i = 1:100
    eul = tr2eul(Href(:,:,i));
    fi(i) = eul(1);
    theta(i) = eul(2);
    psi(i) = eul(3);
end

% Experiment 2.d: Compute the derivative of euler angles (alphadot)
alphadot = zeros(3,100);
for i = 1:100
    B_i = [0, -sin(fi(i)), cos(fi(i)) * sin(theta(i)); ...
           0, cos(fi(i)), sin(fi(i)) * sin(theta(i)); ...
           1, 0, cos(theta(i))];
    % Using A\b instead of inv(A)*b as suggested by Matlab interpreter.
    alphadot(:,i) = B_i\omega(:,i);
end

% Part 2.e: Compute qdref1, to compare with qdref
qdref1 = zeros(6,100);
for i = 1:100
    if sin(theta(i)) > 1E-5
        J_i = ajacobian(qref(:,i), myrobot);
        twist_i = [odot(:,i); alphadot(:,i)];
        qdref1(:,i) = J_i\twist_i;
    end
end
    

% Part 2.f: Compare qdref1 and qdref to see if there is a problem
for i = 1:100
    if sin(theta(i)) > 1E-5
        norm_dif = norm(qdref1(:,i) - qdref(:,i));
        if norm_dif > 1E-5
            disp('Something is wrong') % This never gets printed (no issues)
        end
    end
end

% Part 3: Create cubic splines for qref and qdref
splineqref=spline(t,qref);
splineqdref=spline(t,qdref);
    
% Part 4: Create cubic splines for qddref (second derivative of qref)
d=length(splineqdref.coefs);
splineqddref=splineqdref;
splineqddref.coefs=splineqdref.coefs(:,1:3).*(ones(d,1)*[3 2 1]);
splineqddref.order=3;

%% Experiment Part 4.2
% Testing Independent Joint Controller

% Initialize the Jm and B values from the handout
% Otherwise motor.m would not work
myrobot.links(1).Jm = 2*10^-4;
myrobot.links(2).Jm = 2*10^-4;
myrobot.links(3).Jm = 2*10^-4;
myrobot.links(4).Jm = 3.3*10^-5;
myrobot.links(5).Jm = 3.3*10^-5;
myrobot.links(6).Jm = 3.3*10^-5;
myrobot.links(1).B = 1.48*10^-3;
myrobot.links(2).B = 8.17*10^-4;
myrobot.links(3).B = 1.38*10^-3;
myrobot.links(4).B = 7.12*10^-5;
myrobot.links(5).B = 8.26*10^-5;
myrobot.links(6).B = 3.67*10^-5;

% Part 2: Simulate the motors
sys=@(t,x)motors(t,x,myrobot,splineqref,splineqdref,splineqddref);
Ts=0.02;
q0=[3*pi/2;zeros(11,1)];
[t,q]=ode45(sys,([0:Ts:6*pi])',q0);

% Part 3: Plot the required figures
qref=ppval(splineqref,t)';
for i = 1:6
    figure('Name',['Joint Variable ',num2str(i)]);
    plot(t,qref(:,i), t,q(:,i))
    legend('qref', 'q')
    xlabel('t');
    ylabel(['q ',num2str(i)]);
end

% Part 4: Plot the robot movement and the reference trajectory
figure('Name','Robot Movemement and Ref. Trajectory');
hold on
oref=squeeze(Href(1:3,4,:));
plot3(oref(1,:),oref(2,:),oref(3,:),'r')
view(-125,40);
plot(myrobot,q(:,1:6))
hold off
