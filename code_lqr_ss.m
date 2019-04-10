clear all;
clc;
close all;

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

Q=eye(3);

rho=1;
R=rho*eye(3);
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

[K,P,E]=dlqr(A,B,Q,R);
[K2,S2,E2] = dlqry(A,B,M,D,Q,R);

sys=ss(A-B*K,eye(3),eye(3),zeros(3,3));
t=0:0.01:100;
x=initial(sys,[1;0;0],t);
x1=[1 0 0]*x';
x2=[0 1 0]*x';
x3=[0 0 1]*x';
u=-(K*x')';
figure(1)
subplot(2,2,1); 
plot(t,x1);
grid
subplot(2,2,2); 
plot(t,x2);
grid
subplot(2,2,3); 
plot(t,x3);
grid
subplot(2,2,4); 
plot(t,u);
grid

sys2=ss(A-B*K2,eye(3),eye(3),zeros(3,3));
t=0:0.01:100;
xN=initial(sys2,[1;0;0],t);
x_1=[1 0 0]*xN';
x_2=[0 1 0]*xN';
x_3=[0 0 1]*xN';
u2=-(K2*xN')';
figure(2)
subplot(2,2,1); 
plot(t,x_1);
grid
subplot(2,2,2); 
plot(t,x_2);
grid
subplot(2,2,3); 
plot(t,x_3);
grid
subplot(2,2,4); 
plot(t,u2);
grid

sys3=ss(A-B*K2,B,M,zeros(3,3));
t=0:0.01:100;
xn=initial(sys3,[1;0;0],t);
x__1=[1 0 0]*xn';
x__2=[0 1 0]*xn';
x__3=[0 0 1]*xn';
u3=-(K2*xn')';
figure(3)
subplot(2,2,1); 
plot(t,x__1);
grid
subplot(2,2,2); 
plot(t,x__2);
grid
subplot(2,2,3); 
plot(t,x__3);
grid
subplot(2,2,4); 
plot(t,u3);
grid

sys4=ss(A,B,M,zeros(3,3));
t=0:0.01:100;
x0=initial(sys3,[1;0;0],t);
x___1=[1 0 0]*x0';
x___2=[0 1 0]*x0';
x___3=[0 0 1]*x0';
u4=-(K2*x0')';
figure(4)
subplot(2,2,1); 
plot(t,x___1);
grid
subplot(2,2,2); 
plot(t,x___2);
grid
subplot(2,2,3); 
plot(t,x___3);
grid
subplot(2,2,4); 
plot(t,u4);
grid