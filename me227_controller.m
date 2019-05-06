function [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
%ME227 Controller:
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
% 
% Here you should use the inputs from above to calculate the inputs to the
% vehicle. These will be delta and Fx and will be determined based upon
% your control laws below. 
%
% For the project you wil use this same input output structure and in this
% homework you will use this control structure for defining the control
% inputs in your simulation. 

% Define your vehicle parameters here (you must define all of your
% parameters here, you may not reference and outside function in order to 
% function on the GTI)

% Define your controller parameters here:


%Find Uxdesired for the current distance along the path via interpolation
uxdes = interp1(path.s_m, path.UxDes, s);
axdes = interp1(path.s_m, path.axDes, s);

%Find Curvature for the current distance along the path via interpolation
K = interp1(path.s_m, path.k_1pm, s);

% Use the Lateral Control Law to Caclulate Delta
if Mode == 1
    % Calculate the feedback steering command with lateral error and heading error
else
    % Calculate the steering command with both feedback and feedforward control
end
    
% Use the Longitudinal Control Law to Calcuate Fx

end
