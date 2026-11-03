%% lls_offline.m
% Batch linear least squares (LLS) estimate of a first-order DC motor model,
% computed in double precision from the data logged by dc_motor_lls.slx or
% dc_motor_rls.slx (or their virtual versions).
%
%   y(k) = -a1*y(k-1) + b0*u(k-1) + c,      theta = [a1; b0; c]
%   G(z) = b0/(z + a1),  K = b0/(1 + a1),  tau = -Te/ln(-a1)
%   theta = (Phi'*Phi) \ (Phi'*Y)
%
% Run the Simulink model, stop it, then run this script.
% u_data is the motor voltage [V] and y_data the filtered speed [RPM]. The
% speed is low-pass filtered in the model, so the voltage is passed through
% the SAME filter here: this leaves the motor model unchanged but stops the
% filter's lag from biasing the estimated time constant.
% ACE-Lab, ACE-CORE.

Ts = 0.01;                % sample time of the logged data [s]
D  = 5;                   % decimation, as in the model (Te = D*Ts)
Te = D*Ts;                % estimation interval [s]
t_start = 1;              % excitation start time [s] (earlier data ignored)

[u, y] = getLoggedData();
uf = filter(0.2, [1 -0.8], u);          % same low-pass filter as the speed

%% Decimate: average u over each interval, sample y at the start of each one
n  = floor(numel(y)/D);
ud = mean(reshape(uf(1:n*D), D, n), 1)';
yd = y(1:D:n*D);
td = (0:n-1)'*Te;
keep = td >= t_start;
ud = ud(keep);  yd = yd(keep);  td = td(keep);

%% Regressor matrix and least squares solution
Phi = [-yd(1:end-1), ud(1:end-1), ones(numel(yd)-1, 1)];   % [-y(k-1), u(k-1), 1]
Y   = yd(2:end);                                           % y(k)
theta = (Phi'*Phi) \ (Phi'*Y)                              % [a1; b0; c]

a1 = theta(1);  b0 = theta(2);  c = theta(3);
Gd     = tf(b0, [1 a1], Te)             % discrete-time model
K      = b0/(1 + a1)                    % gain [RPM/V]
Tau    = -Te/log(-a1)                   % time constant [s]
V_dead = -c/b0                          % dead-band voltage [V]

%% Simulate the model from the measured input and compare
ysim = zeros(size(yd));  ysim(1) = yd(1);
for k = 2:numel(yd)
    ysim(k) = -a1*ysim(k-1) + b0*ud(k-1) + c;
end
fit = 100*(1 - norm(yd - ysim)/norm(yd - mean(yd)))   % fit [%]

figure('Name', 'LLS identification');
subplot(2,1,1);
plot(td, yd, 'Color', [0.5 0.5 0.5]); hold on;
plot(td, ysim, 'r', 'LineWidth', 1.5); grid on;
ylabel('Speed (RPM)');
legend('Measured (filtered)', sprintf('LLS model, fit %.1f %%', fit), 'Location', 'Best');
title(sprintf('LLS: K = %.2f RPM/V, \\tau = %.3f s, dead-band = %.2f V', K, Tau, V_dead));
subplot(2,1,2);
stairs(td, ud, 'k'); grid on;
xlabel('Time (s)'); ylabel('Filtered voltage (V)');

%% ------------------------------------------------------------------------
function [u, y] = getLoggedData()
% Logged data, whether saved in the 'out' object or in the base workspace
if evalin('base', 'exist(''out'', ''var'')') && ...
        any(strcmp(evalin('base', 'out.who'), 'u_data'))
    u = evalin('base', 'out.u_data');  y = evalin('base', 'out.y_data');
else
    u = evalin('base', 'u_data');      y = evalin('base', 'y_data');
end
u = double(u(:));  y = double(y(:));
end
