%% plot_estimates.m
% Plot how the LLS or RLS estimates logged by dc_motor_lls.slx / dc_motor_rls.slx
% (or their virtual versions) change during the experiment.
%
%   y(k) = -a1*y(k-1) + b0*u(k-1) + c,      theta = [a1; b0; c]
%   K = b0/(1 + a1),  tau = -Te/ln(-a1)
%
% Run the Simulink model, stop it, then run this script.
% ACE-Lab, ACE-CORE.

Ts = 0.01;  D = 5;  Te = D*Ts;

if evalin('base', 'exist(''out'', ''var'')') && ...
        any(strcmp(evalin('base', 'out.who'), 'theta_est'))
    get = @(v) evalin('base', ['out.' v]);
else
    get = @(v) evalin('base', v);
end
u  = double(reshape(get('u_data'), [], 1));
y  = double(reshape(get('y_data'), [], 1));
th = double(get('theta_est'));
if ndims(th) == 3, th = reshape(th, 3, [])'; elseif size(th, 2) ~= 3, th = th'; end
t  = (0:numel(y)-1)'*Ts;

a1 = th(:,1);  b0 = th(:,2);
K   = b0./(1 + a1);
Tau = -Te./log(-a1);  Tau(-a1 <= 0 | -a1 >= 1) = NaN;
fprintf('Final estimates: a1 = %.4f, b0 = %.4f, c = %.3f, K = %.2f RPM/V, tau = %.3f s\n', ...
    a1(end), b0(end), th(end,3), K(end), Tau(end));

figure('Name', 'Parameter estimates');
subplot(3,1,1);
yyaxis left;  plot(t, y);  ylabel('Speed (RPM)');
yyaxis right; stairs(t, u); ylabel('Voltage (V)');
grid on; title('Open-loop experiment');
subplot(3,1,2);
plot(t, K, 'LineWidth', 1.5); grid on; ylabel('K (RPM/V)');
ylim([0, 2*max(K(end), eps)]);
subplot(3,1,3);
plot(t, Tau, 'LineWidth', 1.5); grid on; ylabel('\tau (s)'); xlabel('Time (s)');
ylim([0, 3*max(Tau(end), eps)]);
