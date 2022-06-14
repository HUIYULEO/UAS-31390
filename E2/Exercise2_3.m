%% Exercise 2.1
clc; clear; close all

%Defining constants:
m = 0.5;
L = 0.225;
k = 0.01;
b = 0.001;

Dx = 0.01;
Dy = 0.01;
Dz = 0.01;

Ixx = 3e-06;
Iyy = 3e-06;
Izz = 1e-05;
g = 9.81;

% omega1 = 0; %1
% omega2 = 0;
% omega3 = 0;
% omega4 = 0;

% omega1 = 10000; %2
% omega2 = 0;
% omega3 = 10000;
% omega4 = 0;
% 
omega1 = 0;   %3
omega2 = 10000;
omega3 = 0;
omega4 = 10000;
tau_phi=L*k*(omega1^2 - omega3^2);
tau_theta=L*k*(omega2^2 - omega4^2);
tau_psi=b*(omega1^2 - omega2^2 + omega3^2 - omega4^2);

%Starting simulation
t_stop=10;
out = sim('Quadcopter_sim1') %2.1
%out = sim('Quadcopter_simLINEAR'); %2.3


figure(1)
plot(out.t, out.z)
xlabel('x')
grid
title('Position of UAV')

figure(2)
subplot(1, 3, 1)
plot(out.t, out.phi)
xlabel('time [s]')
ylabel('Roll phi [rad]')
grid

subplot(1, 3, 2)
plot(out.t, out.theta)
xlabel('time [s]')
ylabel('Pitch theta [rad]')
grid

subplot(1, 3, 3)
plot(out.t, out.psi)
xlabel('time [s]')
ylabel('Yaw psi [rad]')
grid

%sgtitle('Omega = [0, 0, 0, 0]^T') %1
%sgtitle('Omega = [10000, 0, 10000, 0]^T')  %2
sgtitle('Omega = [0, 10000, 0, 10000]^T')  %3


