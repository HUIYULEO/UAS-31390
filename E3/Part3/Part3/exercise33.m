clear
clc
close all
%% DEFINE
r2d = 180/pi;
d2r = pi/180;

%addpath("./lib");
addpath("E:\Exercise_data\2022-spring\UAS\E3\Part3\Part3\lib");


%% CONSTANTS
% k_f = 0.01; % [?]
% b = 0.001; % [?]
b = 0.016;
k_f = 8.54858e-06;
m = 0.5; % [kg]
g = 9.81; % [m/s²]
L = 0.225; % [m]

D = diag([0.01, 0.01, 0.01]); % drag of UAV in air [Ns/m]
I = diag([3E-6, 3E-6, 1E-5]); % inertia [Nms²/rad]

T = 0; % elapsed time [s]

%% INITIAL CONDITIONS
p = [0,0,0]'; % position
dp = [0,0,0]'; % velocity
Th = [0,0,0]'; % orientation Euler
dTh = [0,0,0]'; % angular velocity

%% Altitude controller
% P-controller
K_pz = 4.5;
% D-controller
K_dz = 2.5;
% I-controller

%% Attitude controller
% P-controller attitude
K_p = 3;
% D-controller attitude
K_d = 3;

%% Position controller
% P-controller
K_ppos = 0.021;%0.021;
% D-controller 
K_dpos = 0.075; %0.075;
% I-controller 
K_ipos = 0;

%% Desired attitude/altitude
% Steps
roll = 0;
pitch = 0;
yaw = 0;
altitude = 1;

reference = [roll; pitch; yaw];

% Position desired
position = [10; 0; altitude];

%% SET SIM VALUES
dt = 0.01; % time increment [s]
start_time = 0;
end_time = 25;
time = start_time:dt:end_time;

% reference = ones(4,length(time));
% reference(:,time<1) = reference(:,time<1)*0;
% reference(:, time>=1) = reference(:,time>=1) .* [roll; pitch; yaw; altitude];

p_d = ones(3,length(time));
p_d(:,time<1) = p_d(:,time<1)*0;
p_d(:, time>=1) = p_d(:,time>=1) .* position;

p_d(1:2, time < 3) = p_d(1:2, time < 3)*0;



% filename for storing pictures
filename = sprintf('STEP-LIN-%d-%d-%d-%d', roll, pitch, yaw, altitude);
path = '~/Desktop/';

%% INIT VECTORS
p_vec = zeros(3,length(time));
Th_vec = zeros(3,length(time));
e_p_vec = zeros(3, length(time));
dTh_vec = zeros(3,length(time));

%% Simulation
i = 0;
for t = time
    i = i + 1;
    %% Attitude + altitude controller
    b2i = rot_rpy(Th);
    R = b2i';
    
    % Position error
    e_p = p_d(:,i) - p;
    e_p_vec(:,i) = e_p;
    
    % Desired angles
    angles_desired = [0 -1 0; 1 0 0]*R*[K_ppos*e_p(1) + K_dpos * (-dp(1)) + K_ipos * trapz(dt, e_p_vec(1,:));... 
                                        K_ppos*e_p(2) + K_dpos * (-dp(2)) + K_ipos * trapz(dt, e_p_vec(2,:)); 0];
    angles_desired = atan2(sin(angles_desired), cos(angles_desired));
    reference(1:2) = angles_desired;
    
    % Control inputs
    u = K_d*(zeros(3,1)-dTh) + K_p*(reference-Th);
    u_z = K_pz*e_p(3) + K_dz * (-dp(3));
    alti_part = (m*g+u_z)/(k_f*4*cos(Th(2))*cos(Th(1)));
    
    % propeller speeds squared
    gamma = zeros(1,4);
    gamma(1) = alti_part + (2*b*u(1)*I(1,1)+...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(2) = alti_part + (2*b*u(2)*I(2,2)-...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(3) = alti_part + (-2*b*u(1)*I(1,1)+...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(4) = alti_part + (-2*b*u(2)*I(2,2)-...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    
    %% SYSTEM DYNAMICS
    % input from thrusters
    u1 = k_f*sum(gamma);
    
    % rotational velocity of body fixed frame
    omega = dTh;
    
    % linear acceleration
    ddp = 1/m*[(Th(1)*sin(Th(3))+Th(2)*cos(Th(3)))*u1;
               (sin(Th(3))*Th(2)-Th(1)*cos(Th(3)))*u1;
               u1-m*g] - D*dp;
           
    % angular acceleration
    domega = I \ getTauB(L, k_f, b, gamma);
    
    %% Update variables
    % position
    dp = dp + dt * ddp;
    p = p + dt * dp;
    
    % rotation
    omega = omega + dt * domega;
    dTh = EulerParam(Th(1),Th(2)) \ omega;
    Th = Th + dt * dTh;
    Th = atan2(sin(Th),cos(Th));
    
    %% Append to vectors
    p_vec(:,i) = p;
    Th_vec(:,i) = Th;
    dTh_vec(:,i) = dTh;
    
end

%% PLOT
figure(1)
plot(time, p_vec(1,:), 'LineWidth', 2)
hold on
plot(time, p_vec(2,:), 'LineWidth', 2)
plot(time, p_vec(3,:), 'LineWidth', 2)

plot(time, p_d(3,:), '--k')
plot(time, p_d(1,:), '-.k')
grid on
xlabel('Time [s]')
ylabel('Position [m]')
legend('x position', 'y position', 'z position', 'step input z', 'step input x', 'Location', 'southeast')

% saveas(gcf,strcat(path, 'position_', filename), 'epsc');

% figure(2)
% plot(time, Th_vec(1,:)*r2d, 'LineWidth', 2)
% hold on
% plot(time, Th_vec(2,:)*r2d, 'LineWidth', 2)
% plot(time, Th_vec(3,:)*r2d, 'LineWidth', 2)
% grid on
% xlabel('Time [s]')
% ylabel('Angle [deg]')
% legend('\phi', '\theta', '\psi', 'Location', 'southeast')

% saveas(gcf,strcat(path, 'angle_', filename), 'epsc');

%% FUNCTIONS
function dTh2omega = EulerParam(phi, theta)
    dTh2omega = [1, 0, -sin(theta);
                 0, cos(phi), cos(theta)*sin(phi);
                 0, -sin(phi), cos(theta)*cos(phi)];
end

function tau_B = getTauB(L,k,b,gamma)
    tau_B = [L*k*(gamma(1) - gamma(3));
             L*k*(gamma(2) - gamma(4));
             b*(gamma(1)-gamma(2)+gamma(3)-gamma(4))];
end

function ang = normalizeAng(ang)
    ang = atan2(sin(ang), cos(ang));
end