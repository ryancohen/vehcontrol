function [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
%ME227 Controller:
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
% 
% Here you should use the inputs from above to calculate the inputs to the
% vehicle. These will be delta and Fx and will be determined based upon
% your control laws below. 
% For the project you wil use this same input output structure and in this
% homework you will use this control structure for defining the control
% inputs in your simulation. 

% Define your vehicle parameters here (you must define all of your
% parameters here, you may not reference and outside function in order to 
% function on the GTI)

% Define your controller parameters here:
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

%Find Uxdesired for the current distance along the path via interpolation
uxdes = interp1(path.s_m, path.UxDes, s);
axdes = interp1(path.s_m, path.axDes, s);

%Find Curvature for the current distance along the path via interpolation
K = interp1(path.s_m, path.k_1pm, s);

% Use the Lateral Control Law to Caclulate Delta
if Mode == 1
    kla = 7100; %N/m
    xla = 20; %m
    delta = (-kla/Caf)*(e+xla*dpsi);
    dpsi_ss = K*((m*a*ux*ux/(L*Car))-b);
    delta_ff = (kla*xla*dpsi_ss/Caf) + K*(L+(K_us*ux*ux));
    delta = delta + delta_ff;    
   % Calculate the feedback steering command with lateral error and heading error
else
    % r = dpsi_dot, dpsi
    Kp = -1;
    Ki = -0.8; % -0.518 = gucci
    Kd = -0.17; %%% THIS IS FOR YOU CHRIS
    s_dot = (1/(1-e*K))*(ux*cos(dpsi)-uy*sin(dpsi));
    dpsi_dot = r-K*s_dot;
    delta = Kp*dpsi+Kd*dpsi_dot+Ki*e+0;
    % Calculate the steering command with PID
end

% Use the Longitudinal Control Law to Calcuate Fx
frr             = 0.015;
CdA             = 0.594; % m^(2)
rho             = 1.225; % kg/m^(3)
Frr = frr*m*g;
Fd = 0.5*rho*CdA*(ux^2);
Kdriver = 0.5*m*0.1*g;
Fx = m*axdes+Frr+Fd+Kdriver*(uxdes-ux);
% Fx = Fd+Frr+Kdriver*(uxdes-ux);
end
