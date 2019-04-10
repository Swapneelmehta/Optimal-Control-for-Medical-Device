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

%State space model matrices A, B and M for system Dynamics 
% A and B are for the state equation while 
% M comes in output equation of constraint
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
flag=0;
if flag==0

% Open Loop simulation for system without any control action
for k=1:10
rhok=k;
R=rhok*eye(3);
for j=1:5
    rhoq=j;
    Q=rhoq*eye(3);
[K,S,E]=dlqr(A,B,Q,R);
[K2,S2,E2] = dlqry(A,B,M,D,Q,R);
sys_openloop=ss(A,B,M,zeros(3,3));
t=0:0.01:10;
x00=initial(sys_openloop,[1;0;0],t); %system is initiated here
% And states of the system are defined as below with initial  values of
%x1=1, x2=0, x3=0, By calculation we get y3=-84 at initial shown in report
x____1=[1 0 0]*x00';
x____2=[0 1 0]*x00';
x____3=[0 0 1]*x00';
u_openloop=-(K2*x00')'; % Control of open loop system
subplot(2,2,1); 
plot(t,x____1); grid
title('State x1')
subplot(2,2,2); 
plot(t,x____2); grid
title('State x2')
subplot(2,2,3); 
plot(t,x____3); grid
title('State y3')
subplot(2,2,4); 
plot(t,u_openloop);
title('Control effort u')
grid
pause(0.8)
end
end    
 
%After designing controller A-BK is used to compare stability
%For the next two simulations, output weights are not considered to test 
% the system, and results are good enough to implement the actual system
for k=1:30
rhok=k;
R=rhok*eye(3);
[K,S,E]=dlqr(A,B,Q,R);
[K2,S2,E2] = dlqry(A,B,M,D,Q,R);
sys_closedloop=ss(A-B*K,B,M,zeros(3,3));
t=0:0.01:100;
xn=initial(sys_closedloop,[1;0;0],t);
x__1=[1 0 0]*xn';
x__2=[0 1 0]*xn';
x__3=[0 0 1]*xn';
u_closedloop=-(K2*xn')';
subplot(2,2,1); 
plot(t,x__1); grid
title('State x1')
subplot(2,2,2); 
plot(t,x__2); grid
title('State x2')
subplot(2,2,3); 
plot(t,x__3); grid
title('State y3')
subplot(2,2,4); 
plot(t,u_closedloop);
title('Control effort u')
grid
pause(0.8)
end

%Simulation of control system of designed controller without including 
%measurement matrix into system, by using identity matrix as C
for k=1:10
rhok=k;
R=rhok*eye(3);
[K,S,E]=dlqr(A,B,Q,R);
[K2,S2,E2] = dlqry(A,B,M,D,Q,R);
sys=ss(A-B*K,eye(3),eye(3),zeros(3));
t=0:0.01:100;
x=initial(sys,[1;0;0],t);
x1=[1 0 0]*x';
x2=[0 1 0]*x';
x3=[0 0 1]*x';
u=-(K*x')';
subplot(2,2,1); 
plot(t,x1); grid 
title('State variable x1')
subplot(2,2,2); 
plot(t,x2); grid
title('State variable x2')
subplot(2,2,3); 
plot(t,x3); grid
title('State variable y3')
subplot(2,2,4); 
plot(t,u); grid
title('Control effort u')
pause(0.8)
end 

%Controller design with output weights consideration 
%in state feedback controller simulation
for k=1:25
rhok=k;
R=rhok*eye(3);
[K,S,E]=dlqr(A,B,Q,R);
[K2,S2,E2] = dlqry(A,B,M,D,Q,R);
sys_weights=ss(A-B*K2,eye(3),M,zeros(3,3));
t=0:0.01:100;
xN=initial(sys_weights,[1;0;0],t);
x_1=[0.2 0 0]*xN';
x_2=[0 0.2 0]*xN';
x_3=[0 0 0.2]*xN';
u_closedloopweights=-(K2*xN')';
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
plot(t,u_closedloopweights);
title('Control effort u')
grid
pause(0.8)
end
end
flag=1;