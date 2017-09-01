function [] = initialize_params( )
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明

global Jtp Ixx Iyy Izz m g b d l
global x0 xd
global R Q H 
global stepsize t0 tf
global lb ub 
global noise_rate
global ode_options

%%
noise_rate = 0;
ode_options = odeset('RelTol',1e-4,'AbsTol',[1e-5*ones(1,6) 1e-4*ones(1,6)]);

%%
% initialize timer
stepsize =  0.1;
t0=0.00;
tf=5.00;

%%
% Initial values
x0=zeros(12,1);

% Desired values
% xd = rand(12,1).*[10,5,10,5,10,5,1,1,1,1,1,1]';
xd = zeros(12,1);xd(5)=10;

%%
R = 100 * ones(4) / 250000;% control input cost
Q = zeros(12,12);
Q(2,2) = 1; Q(4,4) = 1; Q(6,6)=1; % translational velocity cost
Q(7,7) = 5; Q(9,9) = 5; Q(11,11)=5; % vabration cost
H = zeros(12,12); % final state cost

%%
% initialize parameters
Ixx = 0.01;  % Quadrotor moment of inertia around X axis
Iyy = 0.01;  % Quadrotor moment of inertia around Y axis
Izz = 0.02;  % Quadrotor moment of inertia around Z axis
Jtp = 0.016*9.9865*10^(-6);  % Total rotational moment of inertia around the propeller axis
b = 9.9865*10^(-6);  % Thrust factor
d = 8.06428*10^(-5);  % Drag factor
l = 0.21;  % Distance to the center of the Quadrotor
m = 1.0;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

%%
% control limits
lb = 125*ones(4,1);
ub = 523*ones(4,1);

% Objective function
% J=sum(0.5*uT*R*u + 0.5*xT*Q*x) + xfT*H*xf



end

