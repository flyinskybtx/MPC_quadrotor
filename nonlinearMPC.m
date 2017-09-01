function nonlinearMPC


h=1000;q=0;r=1;
A=[0 1;0 0];B=[0;1];
tf=2;dt=0.1;
H=[h 0;0 h];Q=[q 0;0 q];R=r;

h=4;q=1;r=3;
A=[0 1;0 1];B=[0 1]';tf=10;dt=.1;
H=[0 0;0 h];Q=[q 0;0 0];R=r;
xx0=[1;1];
time=0:dt:tf;

options = optimoptions('fmincon','Display', 'off', 'Algorithm','sqp');

    function J=obj(q,x0,n)
        J=0.;
        x=zeros(2,n);
        x(:,1)=x0;
        for i=1:n
            xdot=A*x(:,i)+B*q(i);
            x(:,i+1)=x(:,i)+xdot*dt;
            J = J + 0.5 * dt * q(i)' * R * q(i);
            J = J + 0.5 * dt * x(:,i)' * Q * x(:,i);
        end
        J = J + 0.5 * x(:,n+1)' * H * x(:,n+1);
    end
tic;
NN=tf/dt;
q0=zeros(1,NN);
lb = -1.5*ones(1,NN);ub = 1.5*ones(1,NN);

u2 = fmincon(@(q)obj(q,xx0,length(q0)),q0,[],[],[],[],lb,ub,[],options);
x2=zeros(2,NN);
x2(:,1)=xx0;
for kk=1:NN
    xdot2=A*x2(:,kk)+B*u2(kk);
    x2(:,kk+1)=x2(:,kk)+xdot2*dt;
end
figure(1);
plot(0:dt:tf,x2);
ylabel('x(t)')
xlabel('Time')
title('states, N=100, open-loop optimal control (4,1,3)')
print -dpng -r300 nmpc-N100-state.png;
figure(2);
plot(0:dt:tf-dt,u2);
axis([0 10 -1.8 1.2]);
ylabel('u(t)')
xlabel('Time')
title('control, N=100, open-loop optimal control (4,1,3)')
print -dpng -r300 nmpc-N100-control.png;
toc;


N=10;
q0=zeros(1,N);
lb = -1.5*ones(1,N);ub = 1.5*ones(1,N);
x3=zeros(2,NN);
x3(:,1)=xx0;
for kk=1:length(time)-N
    tic;

    q0 = fmincon(@(q)obj(q,x3(:,kk),N),q0,[],[],[],[],lb,ub,[],options);
    u3(kk:kk+N-1)=q0;
    xdot3=A*x3(:,kk)+B*u3(kk);
    x3(:,kk+1)=x3(:,kk)+xdot3*dt;
    toc;
end
for kk=length(time)-N+1:length(time)-1
    xdot3=A*x3(:,kk)+B*u3(kk);
    x3(:,kk+1)=x3(:,kk)+xdot3*dt;
end
figure(3);
plot(0:dt:tf,x3);
ylabel('x(t)')
xlabel('Time')
title('states, N=10, NMPC, (4,1,3)')
print -dpng -r300 nmpc-N10-state.png;

figure(4);
plot(0:dt:tf-dt,u3);
axis([0 10 -1.8 1.2]);
ylabel('u(t)')
xlabel('Time')
title('control, N=10, NMPC, (4,1,3)')
print -dpng -r300 nmpc-N10-control.png;

N=10;
q0=zeros(1,N);
lb = -1.5*ones(1,N);ub = 1.5*ones(1,N);
x4=zeros(2,NN);
x4(:,1)=xx0;
for kk=1:length(time)-N
    tic;

    q0 = fmincon(@(q)obj1(q,x4(:,kk),N),q0,[],[],[],[],lb,ub,[],options);
    u4(kk:kk+N-1)=q0;
    xdot4=A*x4(:,kk)+B*u4(kk);
    x4(:,kk+1)=x4(:,kk)+xdot4*dt;
    toc;
end
for kk=length(time)-N+1:length(time)-1
    xdot4=A*x4(:,kk)+B*u4(kk);
    x4(:,kk+1)=x4(:,kk)+xdot4*dt;
end

figure(5);
plot(0:dt:tf,x4);
ylabel('x(t)')
xlabel('Time')
title('states, N=10, NMPC, (4,1,0.3)')
print -dpng -r300 nmpc-N10-state1.png;

figure(6);
plot(0:dt:tf-dt,u4);
axis([0 10 -1.8 1.2]);
ylabel('u(t)')
xlabel('Time')
title('control, N=10, NMPC, (4,1,0.3)')
print -dpng -r300 nmpc-N10-control1.png;

    function J=obj1(q,x0,n)
        J=0.;
        x=zeros(2,n);
        x(:,1)=x0;
        for i=1:n
            xdot=A*x(:,i)+B*q(i);
            x(:,i+1)=x(:,i)+xdot*dt;
            J = J + 0.1 * 0.5 * dt * q(i)' * R * q(i);
            J = J + 0.5 * dt * x(:,i)' * Q * x(:,i);
        end
        J = J + 0.5 * x(:,n+1)' * H * x(:,n+1);
    end
end