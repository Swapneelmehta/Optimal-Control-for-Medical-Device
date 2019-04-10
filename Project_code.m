%Discrete LQR for solving the problem of optimal control of artificial
%kidney system
clear all;
clc;
close;

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

for k=10:20:50
rho=k;
R=rho*eye(3);
A=[((-k1/v1)-(k2/v2)-(Qb/v1)*Ess) k1/v1 ((L/v1)+(k2/v1));
    k1/v2                       -k1/v2 0;
    k2/v3                        0     -(k2/v3)-(L/v3)]
B=[(-(Qb/v1)*C1ss) 0 0;
    0 0 0;
    0 0 0]

M=[1  0  0;
   0  1  0;
   -P0 0 P0];

D=zeros(3);

[K1,S1,E1] = dlqr(A,B,Q,R,0)
[K2,S2,E2] = dlqry(A,B,M,D,Q,R)
% hold on
figure(1)
plot(R,K1)
% hold on
figure(2)
plot(R,S1)
% hold on
figure(3)
plot(R,E1)
% hold on
figure(4)
plot(R,K2)
% hold on
figure(5)
plot(R,S2)
% hold on
figure(6)
plot(R,imag(E2))
% hold on 
plot(R,real(E2))
%hold on
end



