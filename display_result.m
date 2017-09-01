function [ x ] = display_result(t0,tf,x0,xd,u )
%UNTITLED7 此处显示有关此函数的摘要
%   此处显示详细说明

global ode_options
global stepsize

x=x0;

%% update x
for t= t0:stepsize:tf-stepsize 
    [~,xtemp] = ode45(@statespace_function, [t,t+stepsize],x(:,end),ode_options,u(:,round(t/stepsize + 1))); 
    x= [x xtemp(end,:)'];
end

%%
% i=0;
% x(:,1)=x0;
% for ti=t(2:end)
%     i=i+1;
%     x(:,i+1) = x(:,i) +statespace_function(x(:,i),u(:,i))*stepsize;  % update x
%
% end


%%
% plot figures
xsize = size(x);
xd=xd*ones(1,xsize(2));

figure(1);
subplot(2,2,1)
plot(t0:stepsize:tf,x(1,:),'r');hold on;
plot(t0:stepsize:tf,x(3,:),'g');hold on;
plot(t0:stepsize:tf,x(5,:),'b');hold on;
plot(t0:stepsize:tf,xd(1,:),'y--');hold on;
plot(t0:stepsize:tf,xd(3,:),'y--');hold on;
plot(t0:stepsize:tf,xd(5,:),'y--');hold on;


subplot(2,2,2)
plot(t0:stepsize:tf,x(2,:),'r');hold on;
plot(t0:stepsize:tf,x(4,:),'g');hold on;
plot(t0:stepsize:tf,x(6,:),'b');hold on;


subplot(2,2,3)
plot(t0:stepsize:tf,x(7,:),'g');hold on;
plot(t0:stepsize:tf,x(9,:),'r');hold on;
plot(t0:stepsize:tf,x(11,:),'b');hold on;
plot(t0:stepsize:tf,xd(7,:),'y--');hold on;
plot(t0:stepsize:tf,xd(9,:),'y--');hold on;
plot(t0:stepsize:tf,xd(11,:),'y--');hold on;


subplot(2,2,4)
plot(t0:stepsize:tf,x(8,:),'g');hold on;
plot(t0:stepsize:tf,x(10,:),'r');hold on;
plot(t0:stepsize:tf,x(12,:),'b');hold on;
hold off;

figure(2);
plot(t0:stepsize:tf,[zeros(4,1) u]);
return;

end

