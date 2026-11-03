%% plot_str_results.m
% Plot the results of a run of dc_motor_str.slx or dc_motor_str_virtual.slx:
% speed and reference, motor voltage, PI gains and the estimated model.
% Run the Simulink model, stop it, then run this script.
% ACE-Lab, ACE-CORE.

Ts = 0.01;  D = 5;  Te = D*Ts;

if evalin('base', 'exist(''out'', ''var'')') && ...
        any(strcmp(evalin('base', 'out.who'), 'y_data'))
    get = @(v) evalin('base', ['out.' v]);
else
    get = @(v) evalin('base', v);
end
r  = double(reshape(get('r_data'), [], 1));
y  = double(reshape(get('y_data'), [], 1));
u  = double(reshape(get('u_data'), [], 1));
Kp = double(reshape(get('Kp_data'), [], 1));
Ki = double(reshape(get('Ki_data'), [], 1));
th = double(get('theta_est'));
if ndims(th) == 3, th = reshape(th, 3, [])'; elseif size(th, 2) ~= 3, th = th'; end
t  = (0:numel(y)-1)'*Ts;

K   = th(:,2)./(1 + th(:,1));
Tau = -Te./log(-th(:,1));  Tau(-th(:,1) <= 0 | -th(:,1) >= 1) = NaN;
fprintf('Final model: K = %.2f RPM/V, tau = %.3f s;  final gains: Kp = %.4f, Ki = %.4f\n', ...
    K(end), Tau(end), Kp(end), Ki(end));

figure('Name', 'Self-tuning PI speed control');
subplot(4,1,1);
plot(t, r, 'k--', t, y, 'b', 'LineWidth', 1.2); grid on;
ylabel('Speed (RPM)'); legend('Reference', 'Measured (filtered)', 'Location', 'Best');
title('Self-tuning PI speed control');
subplot(4,1,2);
plot(t, u, 'LineWidth', 1.2); grid on; ylabel('Voltage (V)');
subplot(4,1,3);
yyaxis left;  plot(t, Kp, 'LineWidth', 1.5); ylabel('K_p (V/RPM)');
yyaxis right; plot(t, Ki, 'LineWidth', 1.5); ylabel('K_i (V/(RPM s))'); grid on;
subplot(4,1,4);
yyaxis left;  plot(t, K, 'LineWidth', 1.5); ylabel('K (RPM/V)');
yyaxis right; plot(t, Tau, 'LineWidth', 1.5); ylabel('\tau (s)'); grid on;
xlabel('Time (s)');
