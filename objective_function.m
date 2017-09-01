function [ J] = objective_function(t0,tf,x0,xd,u )
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
global stepsize ode_options;
global R Q H;

J= 0.; % initialize performance index

x=x0;

  timer2 = tic;
    
%% update x
for t= t0:stepsize:tf-stepsize
  
    step = round(t/stepsize + 1);
    [~,x] = ode45(@statespace_function, [t,t+stepsize],x(:,end),ode_options,u(:,step));
    x=x';
    
    J = J + 0.5 * stepsize * u(:,step)' * R * u(:,step); %% update control performance, which may be unnessessary in UAV control
    J = J + 0.5 * stepsize * x(:,end)' * Q * x(:,end); % update state performance
    
end

J = J + 0.5 * (x(:,end)-xd)' * H * (x(:,end)-xd); % final state performance d
% 
% global timer1
% toc(timer1)

end

