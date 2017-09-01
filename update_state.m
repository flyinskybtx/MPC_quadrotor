function xdot = update_state(t,x,omega)
%UNTITLED8 此处显示有关此函数的摘要
%   此处显示详细说明

global Jtp Ixx Iyy Izz b d l m g;
global noise_rate;

%% bound input

for j = 1:4
    if omega(j) > 523
        omega(j) = 523;
    end

    if omega(j) < 125
        omega(j) = 125;
    end
end

%%
%% calculate forces and torques

torques(1) = Jtp*x(11)*[1,-1,1,-1]*omega;  %roll
torques(2) = Jtp*x(9)*[-1,1,-1,1]*omega;  %pitch
omega = omega.^2; % squared omega

thrust = b * sum(omega);
forces=[0,0,thrust];

torques(1)= torques(1) + b*l*(omega(2)-omega(4));
torques(2)= torques(2) + b*l*(omega(3)-omega(1));
torques(3)= d * [-1,1,-1,1] * omega;


forces = (1 + noise_rate * rand) .* forces;
torques = (1+ noise_rate * rand) .* torques;


%% calculate acceleration to generate state function

% state function
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

%% regulate output
xdot = xdot';


