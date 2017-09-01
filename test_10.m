
% The function used with the program: Quadrotor control

function xdot = test_10(t,x,omega)

global Jtp Ixx Iyy Izz b d l m g 
Ixx = 0.01;  % Quadrotor moment of inertia around X axis
Iyy = 0.01;  % Quadrotor moment of inertia around Y axis
Izz = 0.02;  % Quadrotor moment of inertia around Z axis
Jtp = 0.016*9.9865*10^(-6);  % Total rotational moment of inertia around the propeller axis
b = 9.9865*10^(-6);  % Thrust factor
d = 8.06428*10^(-5);  % Drag factor
l = 0.21;  % Distance to the center of the Quadrotor
m = 1.0;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

stepsize = 0.01;
% Initial conditions for the Quadrotor

torques(1) = Jtp*x(11)*[1,-1,1,-1]*omega;  %roll

torques(2) = Jtp*x(9)*[-1,1,-1,1]*omega;  %pitch
omega = omega.^2; % squared omega

thrust = b * sum(omega);
forces=[0,0,thrust];

torques(1)= torques(1) + b*l*(omega(2)-omega(4));
torques(2)= torques(2) + b*l*(omega(3)-omega(1));
        torques(3)= d * [-1,1,-1,1] * omega;


% Evaluation of the State space wrt H-frame
xdot(1) = x(2); % Xdot
xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7)))*(forces(3)/m);    % Xdotdot
xdot(3) = x(4); % Ydot
xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7)))*(forces(3)/m);	% Ydotdot
xdot(5) = x(6); % Zdot
xdot(6) = -g + (cos(x(9))*cos(x(7)))*(forces(3)/m);    % Zdotdot

xdot(7) = x(8) + sin(x(7))*tan(x(9))*x(10) + cos(x(7))*tan(x(9))*x(12); % phydot
xdot(8) = ((Iyy - Izz)/Ixx)*x(10)*x(12)  + torques(1)/Ixx; % pdot = phydotdot
xdot(9) = cos(x(7))*x(8) - sin(x(7))*x(10);    % thetadot
xdot(10) = ((Izz - Ixx)/Iyy)*x(8)*x(12)  + torques(2)/Iyy;	% qdot = thetadotdot
xdot(11) = sin(x(7))/cos(x(9))*x(10) + cos(x(7))/cos(x(9))*x(12);   % thetadot
xdot(12) = ((Ixx - Iyy)/Izz)*x(8)*x(10) + torques(3)/Izz;	% rdot = psidotdot

xdot = xdot';
