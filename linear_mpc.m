function linear_mpc

A=[1.1 2;0 0.95];
B=[0;0.0787];
Q=[1 -1;-1 1];H=Q;R=0.01;
kkk = dlqr(A,B,Q,R);
tf=12;dt=1.;

N=4;
xcurr1=[.5;-.5];

BB=[B 0*B 0*B 0*B;A*B B 0*B 0*B;A^2*B A*B B 0*B;A^3*B A^2*B A*B B]
CC=[A;A^2;A^3;A^4]
QQ=[Q 0*Q 0*Q 0*Q;0*Q Q 0*Q 0*Q;0*Q 0*Q Q 0*Q;0*Q 0*Q 0*Q H];
RR=[R 0*R 0*R 0*R;0*R R 0*R 0*R;0*R 0*R R 0*R;0*R 0*R 0*R R];



Q1=BB'*QQ*BB+RR
B1=-BB'*QQ*CC
Q2=CC'*QQ*CC+Q

time=0:dt:tf;

x=zeros(2,length(time));
for kk=1:length(time)-1
    x(:,kk)=xcurr1;
    u(kk)=optu(x(:,kk));
    xcurr1=A*x(:,kk)+B*u(kk);
end

x(:,length(time))=xcurr1;
figure(1);clf
plot(time,x);
legend('x_1','x_2');
xlabel('time');ylabel('states');title('LinearMPC');
print -dpng -r300 linearmpc.png

figure(2);clf
plot(u);
inv(Q1)*B1
kkk
    function result=optu(x)
        result = [1 0 0 0]*inv(Q1)*B1*x;
    end

end
