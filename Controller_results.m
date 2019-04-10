%MAE 672 Project Optimal control of artificial kidney patient system
% Swapneel Mehta, Person# 50213670

clc;
close all;
%System parameters which are obtained from the data of existing
%dialyzer used for artificial kidney treatment provided in report
v1=15; 
v2=25;
v3=0.135;
k1=56.7;
k2=0.085;
Qb=12;
Ess=0.6;
C1ss=0.0433;
L=0.00197;
P0=84;
%Weight matrix that penalizes the states of system dynamics
Q=0.1*eye(3);
%Weight matrix that penalizes the control 
R=25*eye(3);
A=[((-k1/v1)-(k2/v2)-(Qb/v1)*Ess) k1/v1 ((L/v1)+(k2/v1));
    k1/v2                       -k1/v2 0;
    k2/v3                        0     -(k2/v3)-(L/v3)];
B=[(-(Qb/v1)*C1ss) 0 0;
    0 0 0;
    0 0 0];
M=[1  0  0;
   0  1  0;
   -P0 0 P0];
D=zeros(3);

[K,S,E]=dlqr(A,B,Q,R);
sys_closedloop=ss(A-B*K,B,M,zeros(3,3));
t=0:0.01:100;
x_n=initial(sys_closedloop,[1;0;0],t);
x_1=[1 0 0]*x_n';
x_2=[0 1 0]*x_n';
x_3=[0 0 1]*x_n';
u_closedloop=-(K*x_n')';
figure(1)
subplot(2,2,1); 
plot(t,x_1); grid
title('State x1')
subplot(2,2,2); 
plot(t,x_2); grid
title('State x2')
subplot(2,2,3); 
plot(t,x_3); grid
title('State y3')
subplot(2,2,4); 
plot(t,u_closedloop);
title('Control')
grid
