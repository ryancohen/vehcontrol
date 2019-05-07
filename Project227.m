clc
clear
close all
a_max = 4; a_xmax = 3; a_xmin = -4; a_ymax = 4;
a_max = 0.99*a_max;
a_xmin = 0.985*a_xmin;
load('project_data.mat')

max_front = islocalmax(path.k_1pm, 'FlatSelection', 'first'); % Start of regions of maximum curvature
max_rear = islocalmax(path.k_1pm, 'FlatSelection', 'last'); % Endpoints of regions of maximum curvature
plot(path.s_m, path.k_1pm)
hold on
plot(path.s_m(max_front), path.k_1pm(max_front), 'r*', path.s_m(max_rear), path.k_1pm(max_rear), 'b*')
title('Curvature of Path, with Constant Radii Marked'); xlabel('s [m]'); ylabel('K [1/m]')

vel_profiles = zeros(sum(max_front) + 2, length(max_front)); % Separate velocity profile for each curve, plus for start and endpoints
in_max = 0;
start = 0;

for i = 1:length(max_front)
    if in_max == 0
        if max_front(i) == 1
            start = i;
            vel_profiles(sum(max_front(1:start)), 1:start) = integrate_backwards(path.s_m(1:start), path.k_1pm(1:start), a_max, a_xmin);
            in_max = 1;
        end
    else
        if max_rear(i) == 1
            vel_profiles(sum(max_front(1:start)), start:i) = vel_profiles(sum(max_front(1:start)), start);
            vel_profiles(sum(max_front(1:start)), i:end) = integrate_forwards(path.s_m(i:end), path.k_1pm(i:end), a_max, a_xmax);
            in_max = 0;
        end
    end
end

vel_profiles(end - 1, :) = const_accel(path.s_m, 0.85*a_xmax);
vel_profiles(end , :) = const_accel(path.s_m, a_xmin);

figure
plot(path.s_m, vel_profiles)
title('All Proposed Velocity Profiles'); xlabel('s [m]'); ylabel('Ux [m/s]');

final_vel = min(vel_profiles, [], 1);
figure
plot(path.s_m, final_vel)
title('Final (Minimum) Desired Velocity Profile'); xlabel('s [m]'); ylabel('Ux [m/s]')

final_ax = zeros(1, length(final_vel));
final_ay = zeros(1, length(final_vel));
for i = 2:(length(final_vel) - 1)
    final_ax(i) = (final_vel(i+1) - final_vel(i))/(path.s_m(i+1) - path.s_m(i)) * final_vel(i+1);
    final_ay(i) = path.k_1pm(i) * final_vel(i)^2;
end
final_a = sqrt(final_ax.^2 + final_ay.^2);

figure
plot(path.s_m, final_ax, path.s_m, final_ay, path.s_m, final_a)
legend('a_x', 'a_y', 'a')
title('Desired Acceleration Profiles'); xlabel('s [m]'); ylabel('acceleration [m/s^2]');

max(real(final_ax(3:end)))
min(real(final_ax))
max(real(final_ay))
max(real(final_a(3:end)))

%% Part 2
% assuming grade is constant
% gain choice?
clc;
close all;
% Path is already defined! with s_m, k_1pm, psi_rad, posE_m, posN_m

% append speed profile to path
path.UxDes = final_vel;

g = 9.81;                   	% gravity acceleration, meters/sec^2

setup_niki;

x=1;

% allocate space for simulation data
N = length(path.s_m);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);


% set initial conditions
ux_mps(1)       = 13;
e_m(1)          = 1;
frr             = 0.015;
CdA             = 0.594; % m^(2)
theta_r         = 0;
rho             = 1.225; % kg/m^(3)
mode = 2;

t_final = 8;
t_s = linspace(0, t_final, N);
dt = t_s(2) - t_s(1);

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

    %Note Mode will be used to select which controller is active for the
    %project, but for this homework use Mode == 1 to select the feedback
    %controller and Mode == 2 to select the feedforward plus feedback
    %controller
    [ delta, Fx ] = me227_controller(s, e, dpsi, ux, uy, r, mode, path);
    
    %Calculate the Dynamics with the Nonlinear Bike Model
    [r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model(r, uy, ux, dpsi, e, delta, Fx, K, veh,...
            tire_f, tire_r,frr,CdA,rho,theta_r);
        
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler(r, r_dot, dt);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dt);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dt);
        % Questionable but we do what we can
        s_m(idx+1) = integrate_euler(s, s_dot, dt);
        e_m(idx+1) = integrate_euler(e, e_dot, dt);
        dpsi_rad(idx+1) = integrate_euler(dpsi, dpsi_dot, dt);    
    end
end

figure(7);
title('Problem 3.1')
subplot(2,3,1); hold on; grid on;
    plot(t_s, r_radps)
    xlabel('Time [s]')
    ylabel('r [radps]')
subplot(2,3,2); hold on; grid on;
    plot(t_s, uy_mps)
    xlabel('Time [s]')
    ylabel('u_y [mps]')
subplot(2,3,3); hold on; grid on;
    plot(t_s, ux_mps)
    xlabel('Time [s]')
    ylabel('u_x [mps]')
subplot(2,3,4); hold on; grid on;
    plot(t_s, dpsi_rad)
    xlabel('Time [s]')
    ylabel('\Delta\psi [rad]')
subplot(2,3,5); hold on; grid on;
    plot(t_s, e_m)
    xlabel('Time [s]')
    ylabel('e [m]')
subplot(2,3,6); hold on; grid on;
    plot(t_s, s_m)
    xlabel('Time [s]')
    ylabel('s [m]')


% animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)
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
    Fd=0.5*rho*CdA*ux^(2);
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
