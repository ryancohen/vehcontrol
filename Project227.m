%% Part 1
clear all;
verbose = 1;
% setup_niki;
[final_ax, final_ay, final_a, final_vel] = generate_speed_profiles(verbose);
load('project_data.mat');
path.UxDes = final_vel;
path.axDes = final_ax;
save('project_data.mat');

%% Part 2: Speed Controller with Grade
close all;

% append speed profile to path
path.UxDes = final_vel;
path.axDes = final_ax;
path.k_1pm = zeros(length(final_ax),1);

g = 9.81;                   	% gravity acceleration, meters/sec^2

setup_niki;

x=1;

t_final = 100;
dt = 0.01;
t_s = 0:dt:t_final;


% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
ux_des_mps  = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);
Fx_N        = zeros(N,1);
ay_mps2     = zeros(N,1);
ax_mps2     = zeros(N,1);
a_tot       = zeros(N,1);


% set initial conditions
ux_mps(1)       = 0.001;
e_m(1)          = 1;
frr             = 0.015;
CdA             = 0.594; % m^(2)
theta_r         = zeros(N,1);
    theta_r(round(0.1*N):round(0.15*N)) = 0.08;
rho             = 1.225; % kg/m^(3)
mode = 2; %1 = feedback/forward, 2 = PID

for idx = 1:N
   % look up K
    K = interp1(path.s_m, path.k_1pm, s_m(idx));  
    % current states
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);
    ux_des_mps(idx) = interp1(path.s_m, path.UxDes, s);
    
    [ delta, Fx ] = me227_controller(s, e, dpsi, ux, uy, r, mode, path); 
    %Calculate the Dynamics with the Nonlinear Bike Model
    [r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model(r, uy, ux, dpsi, e, delta, Fx, K, veh,...
            tire_f, tire_r,frr,CdA,rho,theta_r(idx));
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    Fx_N(idx) = Fx;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler(r, r_dot, dt);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dt);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dt);
        % Questionable but we do what we can
        s_m(idx+1) = integrate_euler(s, s_dot, dt);
        e_m(idx+1) = integrate_euler(e, e_dot, dt);
        dpsi_rad(idx+1) = integrate_euler(dpsi, dpsi_dot, dt);
        ax_mps2(idx+1) = ux_dot-r*uy_mps(idx+1);
        ay_mps2(idx+1) = uy_dot+r*ux_mps(idx+1);
        a_tot(idx+1) = sqrt((ax_mps2(idx+1)^2)+(ay_mps2(idx+1)^2));
    end
end

figure(1);
subplot(3,1,1);
    plot(t_s, theta_r);
    title('PART 2 CONTROLLER PERFORMANCE ON STRAIGHT ROAD');
    ylabel('Grade [rads]');
subplot(3,1,2);
    plot(t_s, ux_mps);
    hold on;
    plot(t_s, ux_des_mps);
    ylabel('Long. Vel. [m/s]');
    legend('Ux', 'Ux_{des}')
subplot(3,1,3);
    plot(t_s, ux_des_mps - ux_mps);
    ylabel('\Delta Ux}');
    xlabel('Time [s]')
    
animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)

%% Part 3
clear all; 
close all;
setup_niki;
load('project_data.mat');

% append speed profile to path

g = 9.81;                   	% gravity acceleration, meters/sec^2
setup_niki;
x=1;

t_final = 100;
dt = 0.01;
t_s = 0:dt:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);
Fx_N        = zeros(N,1);
ay_mps2     = zeros(N,1);
ax_mps2     = zeros(N,1);
a_tot       = zeros(N,1);


% set initial conditions
ux_mps(1)       = 0.001;
e_m(1)          = 0.0000001;
frr             = 0.015;
CdA             = 0.594; % m^(2)
theta_r         = zeros(N,1);
%     theta_r(round(0.1*N):round(0.15*N)) = 0.08;
rho             = 1.225; % kg/m^(3)
mode = 2; %1 = feedback/forward, 2 = PID

for idx = 1:N
   % look up K
    K = interp1(path.s_m, path.k_1pm, s_m(idx));  
    % current states
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);

    [ delta, Fx ] = me227_controller(s, e, dpsi, ux, uy, r, mode, path); 
    %Calculate the Dynamics with the Nonlinear Bike Model
    [r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model(r, uy, ux, dpsi, e, delta, Fx, K, veh,...
            tire_f, tire_r,frr,CdA,rho,theta_r(idx));
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    Fx_N(idx) = Fx;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler(r, r_dot, dt);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dt);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dt);
        % Questionable but we do what we can
        s_m(idx+1) = integrate_euler(s, s_dot, dt);
        e_m(idx+1) = integrate_euler(e, e_dot, dt);
        dpsi_rad(idx+1) = integrate_euler(dpsi, dpsi_dot, dt);
        ax_mps2(idx+1) = ux_dot-r*uy_mps(idx+1);
        ay_mps2(idx+1) = uy_dot+r*ux_mps(idx+1);
        a_tot(idx+1) = sqrt((ax_mps2(idx+1)^2)+(ay_mps2(idx+1)^2));
    end
end

figure(2);
subplot(4,1,1);
    plot(t_s,e_m);
    ylabel('Lateral Error [m]');
    if mode == 1
        title('Lookahead Controller Performance w/ No Noise');
    else
        title('PID Controller Performance w/ No Noise');
    end
subplot(4,1,2);
    plot(t_s,ay_mps2);
    ylabel('Lateral Acc.');
    ylim(1.2*[-4,4]);
subplot(4,1,3);
    plot(t_s,ax_mps2);
    ylabel('Long. Acc.');
    ylim(1.2*[-4,3]);
subplot(4,1,4);
    plot(t_s,a_tot);
    ylabel('Total Acc.');
    ylim(1.2*[-4,4]);
    xlabel('Time [s]');
 
figure(3);
subplot(2,1,1)
    plot(t_s, delta_rad);
    ylabel('Delta [rads]');
    title('Controller Outputs');
subplot(2,1,2)
    plot(t_s, Fx_N);
    ylabel('Fx [N]');
    xlabel('Time [s]');
    

% animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)
%% Part 4: Measurement Noise and Grade
close all;

% append speed profile to path
path.UxDes = final_vel;
path.axDes = final_ax;

g = 9.81;                   	% gravity acceleration, meters/sec^2
setup_niki;
t_final = 100;
dt = 0.01;
t_s = 0:dt:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);
Fx_N        = zeros(N,1);
ay_mps2     = zeros(N,1);
ax_mps2     = zeros(N,1);
a_tot       = zeros(N,1);


% set initial conditions
ux_mps(1)       = 0.001;
e_m(1)          = 0.0000001;
frr             = 0.015;
CdA             = 0.594; % m^(2)
theta_r         = zeros(N,1);
%     theta_r(round(0.1*N):round(0.15*N)) = 0.08;
rho             = 1.225; % kg/m^(3)
mode = 2; %1 = feedback/forward, 2 = PID

for idx = 1:N
   % look up K
    K = interp1(path.s_m, path.k_1pm, s_m(idx));  
    % current states
    r = r_radps(idx);
    uy = uy_mps(idx) ;
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);

    ux_noisy = abs(ux_mps(idx) + 0.25*normrnd(0,1))+0.001; %
    e_noisy = e_m(idx)+ 0.05*normrnd(0,1);
    [ delta, Fx ] = me227_controller2(s, e_noisy, dpsi, ux_noisy, uy, r, mode, path); 

    %Calculate the Dynamics with the Nonlinear Bike Model
    [r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model(r, uy, ux, dpsi, e, delta, Fx, K, veh,...
            tire_f, tire_r,frr,CdA,rho,theta_r(idx));
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    Fx_N(idx) = Fx;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler(r, r_dot, dt);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dt);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dt);
        % Questionable but we do what we can
        s_m(idx+1) = integrate_euler(s, s_dot, dt);
        e_m(idx+1) = integrate_euler(e, e_dot, dt);
        dpsi_rad(idx+1) = integrate_euler(dpsi, dpsi_dot, dt);
        ax_mps2(idx+1) = ux_dot-r*uy_mps(idx+1);
        ay_mps2(idx+1) = uy_dot+r*ux_mps(idx+1);
        a_tot(idx+1) = sqrt((ax_mps2(idx+1)^2)+(ay_mps2(idx+1)^2));
    end
end

figure(2);
subplot(4,1,1);
    plot(t_s,e_m);
    ylabel('Lateral Error [m]');
    if mode == 1
        title('Lookahead Controller Performance with Noise')
    else
        title('PID Controller Performance with Noise')
    end
subplot(4,1,2);
    plot(t_s,ay_mps2);
    ylabel('Lateral Acc.');
    ylim(1.2*[-4,4]);
subplot(4,1,3);
    plot(t_s,ax_mps2);
    ylabel('Long. Acc.');
    ylim(1.2*[-4,3]);
subplot(4,1,4);
    plot(t_s,a_tot);
    ylabel('Total Acc.');
    ylim(1.2*[-4,4]);
    xlabel('Time [s]'); 
    
figure;
subplot(2,1,1)
    plot(t_s, delta_rad);
    ylabel('Delta [rads]');
    title('Controller Outputs');
subplot(2,1,2)
    plot(t_s, Fx_N);
    ylabel('Fx [N]');
    xlabel('Time [s]');

animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)
%% Functions
function Ux = integrate_backwards(s, K, a_max, a_xmin)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Ux = zeros(1, length(s));
    Ux(end) = sqrt(a_max/K(end));
    for i = (length(Ux) - 1):-1:1
        dUxds = sqrt(a_max^2 - (K(i+1)*Ux(i+1)^2)^2)/Ux(i+1);
%         if dUxds*Ux(i+1) > abs(a_xmin) % Enforce longitudinal acceleration constraint
%             dUxds = a_xmin/Ux(i+1);
%         end
        Ux(i) = Ux(i + 1) + dUxds*(s(i+1) - s(i));
    end
end

function Ux = integrate_forwards(s, K, a_max, a_xmax)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Ux = zeros(1, length(s));
    Ux(1) = sqrt(a_max/K(1));
    for i = 2:length(Ux)
        dUxds = sqrt(a_max^2 - (K(i-1)*Ux(i-1)^2)^2)/Ux(i-1);
        if dUxds*Ux(i-1) > a_xmax % Enforce longitudinal acceleration constraint
            dUxds = a_xmax/Ux(i-1);
        end
        Ux(i) = Ux(i-1) + dUxds*(s(i) - s(i-1));
    end
end

function Ux = const_accel(s, ax)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Ux = zeros(1, length(s));
    if ax > 0
        Ux(1) = 0.5;
        for i = 2:length(Ux)
            dUxds = ax/Ux(i - 1);
            Ux(i) = Ux(i - 1) + dUxds*(s(i) - s(i-1));
        end
    else
        stopping_index = length(s(s<(s(end) - 3))) - 1; % Stop 3 meters before end of path (reach Ux = 0.5 one index before this)
        Ux(stopping_index) = 0.5;
        for i = stopping_index - 1:-1:1 
            dUxds = ax/Ux(i + 1);
            Ux(i) = Ux(i + 1) - dUxds*(s(i+1) - s(i));
        end
    end
end

%Calculate Forces with the Fiala Nonlinear Tire Model
function Fy = fiala_model(alpha, tire)
%   Calculate tire forces with the fiala model
    a_sl = atan(3*tire.mu*tire.Fz/tire.Ca);
    if abs(alpha)< a_sl
        Fy = (0-(tire.Ca*tan(alpha)))+((tire.Ca^2*(2-((tire.mu_s)/(tire.mu)))*abs(tan(alpha))*tan(alpha))/(3*tire.mu*tire.Fz))-(((tire.Ca^3 * (tan(alpha))^3)/(9*tire.mu^2*tire.Fz^2))*(1-((2*tire.mu_s)/(3*tire.mu))));
    else
       Fy=-tire.mu_s*tire.Fz*sign(alpha);
    end
end

%Calculate the Nonlinear Bicycle Model Dynamics
function [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
    nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f,...
    tire_r,frr,CdA,rho,theta_r)
%KINEMATIC_MODEL
%   Calculate state derivatives for the kinematic vehicle model
% slip angles
[alphaF, alphaR] = slip_angles( r, uy, ux, delta, veh);

% lateral tire forces
fyf = fiala_model(alphaF, tire_f);
fyr = fiala_model(alphaR, tire_r);

%Split longitudinal force based on drive and brakedistribution
if Fx > 0
    fxf = Fx;
    fxr = 0;
else
    fxf = 0.5*Fx;
    fxr = 0.5*Fx;
end
% dynamics
uy_dot=(fyf*cos(delta)+fyr+fxf*sin(delta))/veh.m-r*ux;
    g = 9.81;
    Frr=frr*veh.m*g;
    Fd=0.5*rho*CdA*(ux^2);
ux_dot=(fxr+fxf-Frr-Fd-veh.m*g*sin(theta_r))/veh.m;
s_dot=(1/(1-e*K))*(ux*cos(dpsi)-uy*sin(dpsi));
r_dot=(veh.a*fyf*cos(delta)+veh.a*fxf*sin(delta)-veh.b*fyr)/veh.Iz;
e_dot=uy*cos(dpsi)+ux*sin(dpsi);
dpsi_dot=r-K*s_dot;
end

%Calculate the Slip Angles Here:
function [alphaF, alphaR] = slip_angles( r, uy, ux, delta, veh)
%slip_angles
%   calculate the tire slip angles 
alphaF=((uy+veh.a*r)/ux)-delta;
alphaR=(uy-veh.b*r)/ux;
end

%Use standard Euler Integration
function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state
x1=x0+x0_dot*dt;
end
