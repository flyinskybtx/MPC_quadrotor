% Quadrotor Dynamics

% x1 x3 x5 --> X,Y,Z
% x7 x9 x11 --> phi,theta,psi
% U1 --> total thrust
% U2,U3,U4 --> Roll Pitch Yaw input

    xdot(1) = x(2); % Xdot
    xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7)))*(U(1)/m);    % Xdotdot
    xdot(3) = x(4); % Ydot
    xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7)))*(U(1)/m);	% Ydotdot
    xdot(5) = x(6); % Zdot           
    xdot(6) = - g + (cos(x(9))*cos(x(7)))*(U(1)/m);    % Zdotdot
    xdot(7) = x(8); % phydot
    xdot(8) = ((Iyy - Izz)/Ixx)*x(10)*x(12) - (Jtp/Ixx)*x(10)*Omega + (U(2)/Ixx); % pdot = phydotdot
    xdot(9) = x(10);    % thetadot
    xdot(10) = ((Izz - Ixx)/Iyy)*x(8)*x(12) + (Jtp/Iyy)*x(8)*Omega + (U(3)/Iyy);	% qdot = thetadotdot
    xdot(11) = x(12);   % thetadot
    xdot(12) = ((Ixx - Iyy)/Izz)*x(8)*x(10) + (U(4)/Izz);	% rdot = psidotdot 
    
 % Pendulum Quadrotor Dynamics
 
    xdot(1) = x(2); % Xdot
    xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7)))*(U(1)/m);    % Xdotdot
    xdot(3) = x(4); % Ydot
    xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7)))*(U(1)/m);	% Ydotdot
    xdot(5) = x(6); % Zdot           
    xdot(6) = - g + (cos(x(9))*cos(x(7)))*(U(1)/m);    % Zdotdot
    xdot(7) = x(8); % phydot
    xdot(8) = ((Iyy - Izz)/Ixx)*x(10)*x(12) - (Jtp/Ixx)*x(10)*Omega + (U(2)/Ixx); % pdot = phydotdot
    xdot(9) = x(10);    % thetadot
    xdot(10) = ((Izz - Ixx)/Iyy)*x(8)*x(12) + (Jtp/Iyy)*x(8)*Omega + (U(3)/Iyy);	% qdot = thetadotdot
    xdot(11) = x(12);   % thetadot
    xdot(12) = ((Ixx - Iyy)/Izz)*x(8)*x(10) + (U(4)/Izz);	% rdot = psidotdot 