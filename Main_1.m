%% INTELLIGENT AND ADAPTIVE CONTROL SYSTEMS
% Assignment 2 - Winter Semester 2021/2022
% Kavelidis Frantzis Dimitrios - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH

%% Description:
% Simulation of an aircraft control system an adaptive controller
% implementation
% i)   Part 1 : Control of a Linear system without uncertainties
% ii)  Part 2 : Destabilization of the system in the existence of
%               linear state-dependent uncertainties
% iii) Part 3 : Control of system with uncertainties

%% Clearing
clear
close all
clc
%% Initialization
% xp = [a,q]'
global Ap Bp Cp D A B C Kx Am Bm Cm ka kq Gammax Pm
Ap = [-0.8060 1; -9.148 -4.59];
Bp = [-0.04; -4.59];
Cp = [1; 0];
% Augmented system
A = [0 Cp';zeros(2,1) Ap];
B = [0; Bp];
C = [0;Cp];
% Checking controllability
fprintf('Is it Controllable? 1 for yes, 0 for no : %d \n', rank(ctrb(A,B))==size(A,1))

%% Part 1 - Linear System
% Gains / LQR
Q = [10 0 0;zeros(2,3)];
% Q = eye(3)
% Q = [10 0 0;[0;0] eye(2)]
R = 1;
[Kx,S,eggs] = lqr(A,B,Q,R)
% Kx = [-3.1623 1.1016 0.2152];
% Reference Model
Bm = [-1; zeros(2,1)];
Am = A-B*Kx;
Cm = C;
% Time Span
tStart = 0;
tStep = 0.1;
tEnd = 100;
tspan = tStart:tStep:tEnd;
% Initial Value
initCond = [0 0 0 0 0 0];

% % Step Response for an overview
% sys = ss(Am,B,C',0);
% step(sys)

% [y,t,x] = initial(sys,initCond,tspan)

% Solving diferential equations
[t,xx] = ode45(@(t,xx) IdealSys(t,xx), tspan, initCond);

% Extracting solutions
x = xx(:,1:3);
y = C'.*x;
xm = xx(:,4:6);
ym = Cm'.*xm;
a = x(:,2);
q = x(:,3);

for i = 1:length(t)
    r(i) = RefSig(t(i));
end
r = r';

% Plots
figure
subplot(2,1,1)
plot(t,xm(:,2),'g')
grid on
title('Angle of attack vs time - Output / Performance Evaluation')
xlabel('Time (s)')
ylabel('alpha (rad)')
hold on
plot(t,a,'r-.')
plot(t,r,'blue')
legend('Reference','Actual','Command')

u = -Kx.*x;
subplot(2,1,2)
plot(t,u(:,2))
grid on
title('Elevator Deflection vs time - Controller / Actuator Effort Evaluation')
xlabel('Time (s)')
ylabel('delta_c (rad)')

%% Part 2 - Destabilize system with State dependent uncertainties
D = 0.5;
% Mdelta          %   Control Effectiveness
Ma = Ap(2,1);     %   Static Stability
Mq = Ap(2,2);     %   Pitch Damping 
% Our goal is to destabilize the LQR.
ka = 1.5*Ma;
kq = 0.5*Mq;
Aunst = Am + B*D*(-Kx+1/D*Kx+[0 1.5*Ma 0.5*Mq])

% System stability with chosen parameters for the uncertainties
% The system will be: xdot = Aunst*x + Bm*r
sys = ss(Aunst,Bm,C',0);
if (isstable(sys)==0)
    fprintf('For the choice of ka = %d, kq = %d, the system is unstable \n',ka,kq)
else
    fprintf('For the choice of ka = %d, kq = %d, the system is stable \n',ka,kq)
end

% Nyquist
figure
nyquist(sys)

% Ideal gains
Kxideal = (1/D*Kx+[0 ka kq])

% Simulating system with linear uncertainties
% Solving diferential equations
[t,xx] = ode45(@(t,xx) LinUnSys(t,xx), tspan, initCond);
% Extracting results
x = xx(:,1:3);
y = C'.*x;
xm = xx(:,4:6);
ym = Cm'.*xm;
a = x(:,2);
q = x(:,3);

% Plots
figure
subplot(2,1,1)
plot(t,xm(:,2),'g')
grid on
title('Angle of attack vs time - Output / Performance Evaluation')
xlabel('Time (s)')
ylabel('alpha (rad)')
hold on
plot(t,a,'r-.')
plot(t,r,'blue')
legend('Reference','Actual','Command')

u = -Kx.*x;
subplot(2,1,2)
plot(t,u(:,2))
grid on
title('Elevator Deflection vs time - Controller / Actuator Effort Evaluation')
xlabel('Time (s)')
ylabel('delta_c (rad)')


%% Part 3a: Control the system with uncertainties 
Q = [100 0 0; 0 100 0; 0 0 100];
Pm = lyap(Am',Q);
Gammax = [200 0 0; 0 200 0; 0 0 200];
% Initial conditions
initCond3 = zeros(1,9);
[t,xx] = ode45(@(t,xx) mrac_sys(t,xx), tspan, initCond3);
% Extracting results
x = xx(:,1:3);
y = C'.*x;
xm = xx(:,4:6);
ym = Cm'.*xm;
a = x(:,2);
q = x(:,3);
Kxest = xx(:,7:9);

% Plots - Output & Controller
% Output
figure
subplot(2,1,1)
plot(t,xm(:,2),'g')
grid on
title('Angle of attack vs time - Output - Presence of Uncertainties')
xlabel('Time (s)')
ylabel('alpha (rad)')
hold on
plot(t,a,'r-.')
plot(t,r,'blue')
legend('Reference','Actual','Command')

% Controller
u = -Kxest.*x;
% u = -Kxest.*x ;
subplot(2,1,2)
plot(t,u(:,2))
grid on
title('Elevator Deflection vs time - Controller - Presence of Uncertainties')
xlabel('Time (s)')
ylabel('delta_c (rad)')


% Plots - Errors
% x - xm
ex = x-xm;
figure

subplot(3,1,1)
plot(t,ex(:,1))
xlabel('Time (s)')
ylabel('e_y_I (rad)')
title('Error e_y_I - e_y_I_m vs Time')
grid on

subplot(3,1,2)
plot(t,ex(:,2))
xlabel('Time (s)')
ylabel('Error of alpha (rad)')
title('Error a - a_m vs Time')
grid on

subplot(3,1,3)
plot(t,ex(:,3))
xlabel('Time (s)')
ylabel('Error of q (rad/s)')
title('Error q - q_m vs Time')
grid on

figure
% y - ym / Tracking output reference error
e = a - ym(:,2);
subplot(2,1,1)
plot(t,e)
xlabel('Time (s)')
ylabel('e (rad)')
title('Tracking output reference error e = y - y_m vs Time')
grid on

% ey = y - r / Tracking reference signal error
ey = a - r;
subplot(2,1,2)
plot(t,ey)
xlabel('Time (s)')
ylabel('e_y (rad)')
title('Tracking output reference error e_y = y - r vs Time')
grid on

% Kx estimations
figure
subplot(3,1,1)
plot(t,Kxest(:,1))
hold on
plot([tStart tEnd],[Kxideal(1), Kxideal(1)])
xlabel('Time (s)')
ylabel('K_x_1')
legend('K_x_1','K_x_1_i_d_e_a_l')
title('Gains Estimation & Ideal Kx vs Time')
grid on

subplot(3,1,2)
plot(t,Kxest(:,2))
hold on
plot([tStart tEnd],[Kxideal(2), Kxideal(2)])
xlabel('Time (s)')
ylabel('K_x_2')
legend('K_x_2','K_x_2_i_d_e_a_l')
grid on

subplot(3,1,3)
plot(t,Kxest(:,3))
hold on
plot([tStart tEnd],[Kxideal(3), Kxideal(3)])
xlabel('Time (s)')
ylabel('K_x_3')
legend('K_x_3','K_x_3_i_d_e_a_l')
grid on


% x, xm
figure
subplot(3,1,1)
plot(t,x(:,1))
hold on
plot(t,xm(:,1))
xlabel('Time (s)')
ylabel('e_y_I, e_y_I_m (rad)')
legend('e_y_I','e_y_I_m')
title('x and x_m vs Time')
grid on

subplot(3,1,2)
plot(t,x(:,2))
hold on
plot(t,xm(:,2))
xlabel('Time (s)')
ylabel('a, a_m (rad)')
legend('a_I','a_m')
grid on

subplot(3,1,3)
plot(t,x(:,3))
hold on
plot(t,xm(:,3))
xlabel('Time (s)')
ylabel('q, q_m (rad)')
legend('q_I','q_m')
grid on