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