%% 4.1 Definition of robot structure

% DH Table from prep. report. 
% All units in cm.
DH = [0, 76, 0, pi/2; ...
      0, -23.65, 43.24, 0; ...
      0, 0, 0, pi/2; ...
      0, 43.18, 0, -pi/2; ...
      0, 0, 0, pi/2; ...
      0, 20, 0, 0];
  
% Create the robot model.
myrobot = mypuma560(DH);

%% 4.2 Plot a sample joint space trajectory

% Initialize the given joint angles.
theta_1 = linspace(0, pi, 200);
theta_2 = linspace(0, pi/2, 200);
theta_3 = linspace(0, pi, 200);
theta_4 = linspace(pi/4, 3*pi/4, 200);
theta_5 = linspace(-pi/3, pi/3, 200);
theta_6 = linspace(0, 2*pi, 200);

% Create q matrix that consists of joint angles.
q = [theta_1.' theta_2.' theta_3.' ...
     theta_4.' theta_5.' theta_6.'];

% Plot the trajectory.
plot(myrobot, q);

%% 4.3 Forward Kinematics

% Define the end effector trajectory to be followed.
o = zeros(200, 3);
for i = 1:200
    H = forward(q(i,:), myrobot);
    o(i,:) = H(1:3,4);
end

% Plot and verify that the end effector trajectory matches the given one.
plot3(o(:,1),o(:,2),o(:,3),'r');
hold on
plot(myrobot,q);
hold off

%% 4.4 Inverse Kinematics

% Verify inverse kinematics equations, compare results with the q values 
% given in section 4.4.
H = [cos(pi/4) -sin(pi/4) 0 20; ...
    sin(pi/4) cos(pi/4) 0 23; ...
    0 0 1 15; 0 0 0 1];
disp('Verify inverse kinematics:')
q = inverse(H,myrobot)

% Create the coordinates to be followed with constant orientation.
d = [linspace(10, 30, 100)' linspace(23, 30, 100)' linspace(15, 100, 100)'];
% Define the constant orientation
R_z_pi4 = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1];

% Now compute the trajectory using IK for given orientation and coordinates.
q = zeros(100,6);
for i = 1:100
    H = [R_z_pi4(1,:) d(i,1); R_z_pi4(2,:) d(i,2);...
         R_z_pi4(3,:) d(i,3); 0 0 0 1];
    q(i,:) = inverse(H, myrobot);
end

% Plot and verify that the robot follows a straight line.
plot3(d(:,1),d(:,2),d(:,3),'r');
hold on
plot(myrobot,q);
