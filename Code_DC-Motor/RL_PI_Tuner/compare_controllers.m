%% compare_controllers.m
% Compares three ways of controlling the speed of the same virtual motor.
% Scenario A (default): K = 16 RPM/V, tau = 0.25 s, dead-band 0.5 V, and the
% gain drops by 40% at t = 40 s. Scenario B: K = 10 RPM/V and the gain
% doubles at t = 40 s. Choose with the variable 'scenario' ('A' or 'B').
%   1. fixed PI (Kp = 0.1, Ki = 0.5)
%   2. RLS self-tuning PI (pole placement)      - ..\SelfTuningPI
%   3. RL agent tuning the PI (trained TD3)     - rl_pi_tuner.slx
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));  cd(here);
strDir = fullfile(here, '..', 'SelfTuningPI');
addpath(strDir);
T = 80;  Ts = 0.01;
if ~exist('scenario', 'var'), scenario = 'A'; end
if scenario == 'A'
    motor = {'K_true', 16; 'tau_true', 0.25; 'V_dead', 0.5; 't_change', 40; 'K_factor', 0.6};
else
    motor = {'K_true', 10; 'tau_true', 0.25; 'V_dead', 0.5; 't_change', 40; 'K_factor', 2.0};
end
fprintf('\nScenario %s: K = %g RPM/V, gain x%g at t = 40 s\n', scenario, motor{1,2}, motor{5,2});

res = struct('name', {}, 'r', {}, 'y', {}, 'Kp', {}, 'Ki', {});

% 1 and 2: the self-tuning PI model, with adaptation off and on
load_system(fullfile(strDir, 'dc_motor_str_virtual.slx'));
names = {'Fixed PI', 'RLS self-tuning PI'};
for adapt = [0 1]
    in = Simulink.SimulationInput('dc_motor_str_virtual');
    in = setVariable(in, 'adapt_on', adapt, 'Workspace', 'dc_motor_str_virtual');
    for k = 1:size(motor, 1)
        in = setVariable(in, motor{k,1}, motor{k,2}, 'Workspace', 'dc_motor_str_virtual');
    end
    in = setModelParameter(in, 'StopTime', num2str(T));
    out = sim(in);
    res(end+1) = struct('name', names{adapt+1}, 'r', out.r_data(:), 'y', out.y_data(:), ...
        'Kp', out.Kp_data(:), 'Ki', out.Ki_data(:)); %#ok<SAGROW>
end

% 3: the RL agent
S = load('rl_pi_tuner_agent.mat', 'agent');
assignin('base', 'agent', S.agent);
load_system('rl_pi_tuner');
in = Simulink.SimulationInput('rl_pi_tuner');
for k = 1:size(motor, 1)
    in = setVariable(in, motor{k,1}, motor{k,2}, 'Workspace', 'rl_pi_tuner');
end
in = setModelParameter(in, 'StopTime', num2str(T));
out = sim(in);
res(end+1) = struct('name', 'RL agent tuning the PI', 'r', out.r_data(:), 'y', out.y_data(:), ...
    'Kp', out.Kp_data(:), 'Ki', out.Ki_data(:));

%% Results
fprintf('\n%-26s %18s %18s %16s\n', '', 'RMS error 20-40 s', 'RMS error 60-80 s', 'final Kp, Ki');
figure('Name', ['Scenario ' scenario ': fixed PI vs RLS self-tuning vs RL agent']);
for k = 1:numel(res)
    n = min(numel(res(k).y), round(T/Ts) + 1);
    t = (0:n-1)'*Ts;
    e = res(k).r(1:n) - res(k).y(1:n);
    w1 = t >= 20 & t < 40;  w2 = t >= 60 & t < 80;
    fprintf('%-26s %14.2f RPM %14.2f RPM   %6.3f, %5.3f\n', res(k).name, ...
        sqrt(mean(e(w1).^2)), sqrt(mean(e(w2).^2)), res(k).Kp(n), res(k).Ki(n));
    subplot(numel(res), 2, 2*k - 1);
    plot(t, res(k).r(1:n), 'k--', t, res(k).y(1:n), 'b'); grid on;
    ylabel('RPM'); title(res(k).name); xline(40, 'r:');
    subplot(numel(res), 2, 2*k);
    yyaxis left;  plot(t, res(k).Kp(1:n)); ylabel('K_p');
    yyaxis right; plot(t, res(k).Ki(1:n)); ylabel('K_i'); grid on; xline(40, 'r:');
    title('PI gains');
end
xlabel('Time (s)');
