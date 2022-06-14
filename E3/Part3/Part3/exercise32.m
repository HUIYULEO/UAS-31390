clear
clc
close all
%% DEFINE
r2d = 180/pi;
d2r = pi/180;

%addpath("./lib");
addpath("E:\Exercise_data\2022-spring\UAS\E3\Part3\Part3\lib");


%% CONSTANTS
k_f = 0.01; % [?]
b = 0.001; % [?]
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
K_pz = 0.8;
% D-controller
K_dz = 1.1;
% I-controller

%% Attitude controller
% P-controller attitude
K_p = 0.9;
% D-controller attitude
K_d = 1.5;

%% Desired attitude/altitude
% Steps
roll = 0;
pitch = 10*d2r;
yaw = 0;
altitude = 0;

reference = [roll, pitch, yaw, altitude]';

%% SET SIM VALUES
dt = 0.01; % time increment [s]
start_time = 0;
end_time = 10;
time = start_time:dt:end_time;

% filename for storing pictures
filename = sprintf('STEP-%d-%d-%d-%d', altitude, roll, pitch, yaw);
path = '~/Desktop/';

%% INIT VECTORS
p_vec = zeros(3,length(time));
Th_vec = zeros(3,length(time));

dTh_vec = zeros(3,length(time));


%% Simulation
i = 0;
for t = time
    i = i + 1;
    %% Attitude + altitude controller
    
    % Position error
    ez = reference(4) - p(3);
    
    % integrated orientation
    dTh_int = [trapz(dt, dTh_vec(1,:));
               trapz(dt, dTh_vec(2,:));
               trapz(dt, dTh_vec(3,:))];
           
    u = K_d*dTh + K_p*(dTh_int - reference(1:3));
       
    % propeller speeds squared
    u_z = K_pz*ez + K_dz * (-dp(3));
    alti_part = (m*g+u_z)/(k_f*4*cos(Th(2))*cos(Th(1)));
    
    gamma = zeros(1,4);
    gamma(1) = alti_part - (2*b*u(1)*I(1,1)+...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(2) = alti_part - (2*b*u(2)*I(2,2)-...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(3) = alti_part - (-2*b*u(1)*I(1,1)+...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);
    gamma(4) = alti_part - (-2*b*u(2)*I(2,2)-...
        u(3)*I(3,3)*k_f*L)/(4*b*k_f*L);

    
    %% SYSTEM DYNAMICS
    % input from thrusters
    u1 = k_f*sum(gamma);
    
    % rotation of body fixed frame w.r.t. intertial frame
    b2i = rot_rpy(Th);
    R = b2i';
    
    % rotational velocity of body fixed frame
    omega = EulerParam(Th(1),Th(2))*dTh;
    F_B = [0,0,u1]';
    % Forces
    F_D = -D*dp;
    F_g = -[0,0,m*g]';
    
    % linear acceleration
    ddp = 1/m*(F_g + R*F_B + F_D);
   
    % angular acceleration
    domega = I \ (getTauB(L, k_f, b, gamma) - cross(omega,I*omega));
    
    %% Update variables
    % position
    dp = dp + dt * ddp;
    p = p + dt * dp;
    
    % rotation
    omega = omega + dt * domega;
    dTh = EulerParam(Th(1),Th(2)) \ omega;
    Th = Th + dt * dTh;
    
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
grid on
xlabel('Time [s]')
ylabel('Position [m]')
legend('x position', 'y position', 'z position', 'Location', 'northwest')

saveas(gcf,strcat(path, 'position_', filename), 'epsc');

figure(2)
plot(time, Th_vec(1,:)*r2d, 'LineWidth', 2)
hold on
plot(time, Th_vec(2,:)*r2d, 'LineWidth', 2)
plot(time, Th_vec(3,:)*r2d, 'LineWidth', 2)
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('\phi', '\theta', '\psi', 'Location', 'northwest')

saveas(gcf,strcat(path, 'angle_', filename), 'epsc');

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