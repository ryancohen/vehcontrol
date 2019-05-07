function [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
%ME227 Controller:
% Define your vehicle parameters here (you must define all of your
% parameters here, you may not reference and outside function in order to 
% function on the GTI)
L = 2.631;
a = 0.48*L;
b = 0.52*L;
m = 1926.2;

Car = 180*1000;
Caf = 110*1000;

mf = m*(b/L);
mr = m*(a/L);
K_us = (mf/Caf)-(mr/Car);
g = 9.81;


% Define your controller parameters here:
kla = 3500; %N/m
xla = 15; %m

% Find Uxdesired for the current distance along the path via interpolation
ux_desired = interp1(path.s_m, path.UxDes, s);

% Find Curvature for the current distance along the path via interpolation
K = interp1(path.s_m, path.k_1pm, s);

% Use the Lateral Control Law to Caclulate Delta
delta = (-kla/Caf)*(e+xla*dpsi);
if Mode == 1
    % Calculate the feedback steering command with lateral error and heading error
else
    dpsi_ss = K*((m*a*ux*ux/(L*Car))-b);
    delta_ff = (kla*xla*dpsi_ss/Caf) + K*(L+(K_us*ux*ux));
    delta = delta + delta_ff;
    % Calculate the steering command with both feedback and feedforward control
end

Kdriver = m*0.1*g;
Fx = Kdriver*(ux_desired-ux);
% Use the Longitudinal Control Law to Calcuate Fx
end