clc
clear all
close all

% declare parameters
global Jtp Ixx Iyy Izz m g b d l;
global  x0 xd;
global R Q H;
global t0 tf stepsize
global ub lb
global noise_rate

initialize_params();
num_steps = (tf-t0)/stepsize;

display('beginning')

%% U test


% u=500*ones(4,num_steps);
% u(1,1:2) = 512*ones(1,2);
% u(3,1:2) =  490*ones(1,2);
% u(1,5:6) = 490*ones(1,2);
% u(3,5:6) =  512*ones(1,2);
% x0=zeros(12,1);
% xd=zeros(12,1);
% xd(5)=20;
% 
% x = display_result( @statespace_function,x0,xd,u);


%%  solve full optimal control problem
display('task 1')

global timer1
timer1 = tic;

step_range = num_steps;
u0 = 495 * ones(4,step_range);
Q(2,2) = 0; Q(4,4) = 0; Q(6,6)=0; % translational velocity cost
Q(7,7) = 0; Q(9,9) = 0; Q(11,11)=0; % vabration cost
Q(8,8) = 0; Q(10,10) = 0; Q(12,12)=0; % vabration cost

%solve optimal control
options = optimoptions('fmincon','Display', 'iter', 'Algorithm','sqp');
[u2,J] = fmincon(@(u)objective_function(t0,tf,x0,xd,u),u0,[],[],[],[],lb*ones(1,step_range),ub*ones(1,step_range),[],options);

% display result
x = display_result( t0,tf,x0,xd,u2);

toc(timer1)

%% solve mpc control problem
display('task 2')

global timer2
timer2 = tic;

timer3 = tic;

step_range = 20; % seriel optimation length
u0 = 495*ones(4,step_range);

%solve sqp control
options = optimset('Display','iter','Algorithm','sqp', 'MaxIter', 1000, 'MaxFunEvals', 1000);

x = x0;
t = t0;

xdtemp = x0;
for step = 1:num_steps-step_range+1
    
    tstart = tic;%s et timer
    xdtemp = xdtemp + xd/num_steps; % set interval xd

    [u3,J] = fmincon(@(u)objective_function(t,t+stepsize*step_range,x(:,end),xdtemp,u),u0,[],[],[],[],lb*ones(1,step_range),ub*ones(1,step_range),[],options); % calc U at each step
%         [u3(:,step:step+step_range-1),J] = fmincon(@(u)objective_function(t,t+stepsize*step_range,x(:,end),xdtemp,u),u0,[],[],[],[],lb*ones(1,step_range),ub*ones(1,step_range),[],options); % calc U at each step

    %update x
    for ttemp= t:stepsize:t+step_range*stepsize
        [~,xtemp] = ode45(@statespace_function, [t,t+stepsize],x(:,end),options,u3(:,round(ttemp/stepsize + 1)));
        x= [x xtemp(end,:)'];
    end
     
     toc(timer3)
end

toc(timer2)

% display result
x = display_result(t,t+stepsize*step_range,x0,xd,u3);



