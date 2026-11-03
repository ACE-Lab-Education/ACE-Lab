%% compare_all.m
% Compares four ways of controlling the speed of the same virtual DC motor,
% for two scenarios in which the motor changes part-way through (t = 40 s):
%   A: K = 16 RPM/V, gain drops to 60% (e.g. extra load)
%   B: K = 10 RPM/V, gain doubles     (e.g. load removed)
% (tau = 0.25 s and a 0.5 V dead-band in both.)
%
%   1. Fixed PI            - hand-picked gains (Kp = 0.1, Ki = 0.5)
%   2. LLS + pole placement - open-loop LLS identification of the motor at the
%                            start, then a PI designed once by pole placement
%                            (zeta = 1, wn = 6 rad/s); gains then stay fixed
%   3. RLS self-tuning PI  - RLS estimation + pole placement, every sample
%   4. RL agent            - trained TD3 agent re-tunes Kp and Ki every 3 s
%
% Uses: ..\dc_motor_lls_virtual.slx, ..\SelfTuningPI\dc_motor_str_virtual.slx,
%       rl_pi_tuner.slx and rl_pi_tuner_agent.mat (run train_rl_tuner.m first).
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));  cd(here);
root = fullfile(here, '..');  strDir = fullfile(root, 'SelfTuningPI');
addpath(root, strDir);
T = 80;  Ts = 0.01;  zeta = 1;  wn = 6;
S = load('rl_pi_tuner_agent.mat', 'agent');  assignin('base', 'agent', S.agent);
load_system(fullfile(root, 'dc_motor_lls_virtual.slx'));
load_system(fullfile(strDir, 'dc_motor_str_virtual.slx'));
load_system('rl_pi_tuner');

scen = struct('name', {'A: gain drops to 60%', 'B: gain doubles'}, 'K', {16, 10}, 'f', {0.6, 2.0});
methods = {'Fixed PI', 'LLS + pole placement', 'RLS self-tuning PI', 'RL agent'};
R = struct();
for s = 1:numel(scen)
    motor = {'K_true', scen(s).K; 'tau_true', 0.25; 'V_dead', 0.5; 't_change', 40; 'K_factor', scen(s).f};

    % --- 2. LLS identification (open loop, motor before the change) ---
    in = Simulink.SimulationInput('dc_motor_lls_virtual');
    in = setVariable(in, 'K_true', scen(s).K, 'Workspace', 'dc_motor_lls_virtual');
    in = setVariable(in, 'tau_true', 0.25, 'Workspace', 'dc_motor_lls_virtual');
    in = setVariable(in, 'V_dead', 0.5, 'Workspace', 'dc_motor_lls_virtual');
    out = sim(in);
    th = squeeze(out.theta_est);  if size(th, 1) == 3, th = th'; end
    a1 = th(end,1);  b0 = th(end,2);
    K = b0/(1 + a1);  p = (-a1)^(1/5);           % model at Ts (estimated at 5*Ts)
    bs = K*(1 - p);
    p1 = -2*exp(-zeta*wn*Ts);  p2 = exp(-2*zeta*wn*Ts);
    q0 = (p1 + 1 + p)/bs;  q1 = (p2 - p)/bs;
    Kp_lls = -q1;  Ki_lls = (q0 + q1)/Ts;
    fprintf('Scenario %s: LLS model K = %.2f RPM/V, tau = %.3f s -> Kp = %.3f, Ki = %.3f\n', ...
        scen(s).name, K, -0.05/log(-a1), Kp_lls, Ki_lls);

    % --- 1-3. fixed PI, LLS-designed PI and RLS self-tuning PI ---
    cfg = {0, 0.1, 0.5; 0, Kp_lls, Ki_lls; 1, 0.1, 0.5};
    for k = 1:3
        in = Simulink.SimulationInput('dc_motor_str_virtual');
        in = setVariable(in, 'adapt_on', cfg{k,1}, 'Workspace', 'dc_motor_str_virtual');
        in = setVariable(in, 'Kp0', cfg{k,2}, 'Workspace', 'dc_motor_str_virtual');
        in = setVariable(in, 'Ki0', cfg{k,3}, 'Workspace', 'dc_motor_str_virtual');
        for j = 1:size(motor, 1)
            in = setVariable(in, motor{j,1}, motor{j,2}, 'Workspace', 'dc_motor_str_virtual');
        end
        in = setModelParameter(in, 'StopTime', num2str(T));
        out = sim(in);
        R(s, k).r = out.r_data(:);  R(s, k).y = out.y_data(:);
        R(s, k).Kp = out.Kp_data(:);  R(s, k).Ki = out.Ki_data(:);  R(s, k).u = out.u_data(:);
    end

    % --- 4. RL agent ---
    in = Simulink.SimulationInput('rl_pi_tuner');
    for j = 1:size(motor, 1)
        in = setVariable(in, motor{j,1}, motor{j,2}, 'Workspace', 'rl_pi_tuner');
    end
    in = setModelParameter(in, 'StopTime', num2str(T));
    out = sim(in);
    R(s, 4).r = out.r_data(:);  R(s, 4).y = out.y_data(:);
    R(s, 4).Kp = out.Kp_data(:);  R(s, 4).Ki = out.Ki_data(:);  R(s, 4).u = out.u_data(:);
end

%% Performance measures
fprintf('\n%-22s %-22s %10s %10s %12s %14s\n', 'Scenario', 'Method', 'RMS 20-40', ...
    'RMS 60-80', 'Overshoot', 'Kp, Ki at end');
fprintf('%-22s %-22s %10s %10s %12s\n', '', '', '(RPM)', '(RPM)', '60-80 (RPM)');
M = zeros(numel(scen), 4, 3);
for s = 1:numel(scen)
    for k = 1:4
        n = min(numel(R(s,k).y), round(T/Ts) + 1);
        t = (0:n-1)'*Ts;  r = R(s,k).r(1:n);  y = R(s,k).y(1:n);  e = r - y;
        w1 = t >= 20 & t < 40;  w2 = t >= 60 & t < 80;
        % mean overshoot past each new reference after t = 60 s
        stp = find(diff(r) ~= 0) + 1;  stp = stp(t(stp) >= 60);
        os = zeros(numel(stp), 1);
        for i = 1:numel(stp)
            idx = stp(i):min(stp(i) + round(3/Ts) - 1, n);
            os(i) = max(0, max(sign(r(stp(i)) - r(stp(i)-1))*(y(idx) - r(idx))));
        end
        M(s, k, :) = [sqrt(mean(e(w1).^2)), sqrt(mean(e(w2).^2)), mean(os)];
        fprintf('%-22s %-22s %10.2f %10.2f %12.2f %8.3f, %5.3f\n', scen(s).name, methods{k}, ...
            M(s,k,1), M(s,k,2), M(s,k,3), R(s,k).Kp(n), R(s,k).Ki(n));
    end
end
save('compare_all_results.mat', 'R', 'M', 'methods', 'scen');

%% Figures
cols = lines(4);
for s = 1:numel(scen)
    figure('Name', ['Scenario ' scen(s).name], 'Position', [60 60 1100 820]);
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact');
    title(tl, ['Scenario ' scen(s).name ' at t = 40 s']);
    for k = 1:4
        n = min(numel(R(s,k).y), round(T/Ts) + 1);  t = (0:n-1)'*Ts;
        nexttile;
        plot(t, R(s,k).r(1:n), 'k--', t, R(s,k).y(1:n), 'Color', cols(k,:), 'LineWidth', 1.1);
        grid on; ylabel('RPM'); xline(40, 'r:'); ylim([0 50]);
        title(sprintf('%s  (RMS %.2f / %.2f RPM)', methods{k}, M(s,k,1), M(s,k,2)));
        nexttile;
        yyaxis left;  plot(t, R(s,k).Kp(1:n), 'LineWidth', 1.2); ylabel('K_p');
        yyaxis right; plot(t, R(s,k).Ki(1:n), 'LineWidth', 1.2); ylabel('K_i');
        grid on; xline(40, 'r:'); title('PI gains');
    end
    exportgraphics(gcf, sprintf('compare_all_scenario_%c.png', 'A' + s - 1), 'Resolution', 130);
end
figure('Name', 'Summary', 'Position', [100 100 900 380]);
tiledlayout(1, 2, 'TileSpacing', 'compact');
for s = 1:numel(scen)
    nexttile;
    b = bar(categorical(methods, methods), squeeze(M(s, :, 1:2)));
    b(1).FaceColor = [0.6 0.6 0.6];  b(2).FaceColor = [0.85 0.33 0.1];
    ylabel('RMS tracking error (RPM)'); grid on;
    legend('before change (20-40 s)', 'after change (60-80 s)', 'Location', 'northoutside');
    title(['Scenario ' scen(s).name]);
end
exportgraphics(gcf, 'compare_all_summary.png', 'Resolution', 130);
