function  xdot  = statespace_function(x, u)
%statespace_function 此处显示有关此函数的摘要
%   U1 = total thrust
%   U2 = roll input
%   U3 = pitch input
%   U4 = yaw input
global Jtp Ixx Iyy Izz m g b d l;

u(1) = u(1) + m*g; % complement for Gravity force


% Calculation of rotors angular velocities
omegasqr(1) = (1/4*b)*u(1) + (1/2*b*l)*u(3) - (1/4*d)*u(4);
omegasqr(2) = (1/4*b)*u(1) - (1/2*b*l)*u(2) + (1/4*d)*u(4);
omegasqr(3) = (1/4*b)*u(1) - (1/2*b*l)*u(3) - (1/4*d)*u(4);
omegasqr(4) = (1/4*b)*u(1) + (1/2*b*l)*u(2) + (1/4*d)*u(4);
omegasqr = real(omegasqr);

omega(1) = sqrt(omegasqr(1));
omega(2) = sqrt(omegasqr(2));
omega(3) = sqrt(omegasqr(3));
omega(4) = sqrt(omegasqr(4));
% Bounding the angular velocities
for j = 1:4
    if omega(j) > 523
        omega(j) = 523;
    end
    
    if omega(j) < 125
        omega(j) = 125;
    end
end
omegasqr(1) = (omegasqr(1))^2;
omegasqr(2) = (omegasqr(2))^2;
omegasqr(3) = (omegasqr(3))^2;
omegasqr(4) = (omegasqr(4))^2;
% Disturbance
Omega = d*(- sqrt(omegasqr(1)) + sqrt(omegasqr(2)) - sqrt(omegasqr(3)) + sqrt(omegasqr(4)));

% state function DEPRECATED
xdot(1) = x(2); % Xdot
xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7)))*(u(1)/m);    % Xdotdot
xdot(3) = x(4); % Ydot
xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7)))*(u(1)/m);	% Ydotdot
xdot(5) = x(6); % Zdot
xdot(6) = - g + (cos(x(9))*cos(x(7)))*(u(1)/m);    % Zdotdot
xdot(7) = x(8); % phydot
xdot(8) = ((Iyy - Izz)/Ixx)*x(10)*x(12) - (Jtp/Ixx)*x(10)*Omega + (u(2)/Ixx); % pdot = phydotdot
xdot(9) = x(10);    % thetadot
xdot(10) = ((Izz - Ixx)/Iyy)*x(8)*x(12) + (Jtp/Iyy)*x(8)*Omega + (u(3)/Iyy);	% qdot = thetadotdot
xdot(11) = x(12);   % thetadot
xdot(12) = ((Ixx - Iyy)/Izz)*x(8)*x(10) + (u(4)/Izz);	% rdot = psidotdot
% %

xdot = xdot';

%


end


% % state function DEPRECATED
% xdot(1) = x(2); % Xdot
% xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7)))*(u(1)/m);    % Xdotdot
% xdot(3) = x(4); % Ydot
% xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7)))*(u(1)/m);	% Ydotdot
% xdot(5) = x(6); % Zdot
% xdot(6) = - g + (cos(x(9))*cos(x(7)))*(u(1)/m);    % Zdotdot
% xdot(7) = x(8); % phydot
% xdot(8) = ((Iyy - Izz)/Ixx)*x(10)*x(12) - (Jtp/Ixx)*x(10)*Omega + (u(2)/Ixx); % pdot = phydotdot
% xdot(9) = x(10);    % thetadot
% xdot(10) = ((Izz - Ixx)/Iyy)*x(8)*x(12) + (Jtp/Iyy)*x(8)*Omega + (u(3)/Iyy);	% qdot = thetadotdot
% xdot(11) = x(12);   % thetadot
% xdot(12) = ((Ixx - Iyy)/Izz)*x(8)*x(10) + (u(4)/Izz);	% rdot = psidotdot
% %

