function in = randomMotor(in, mdl)
% randomMotor  Random virtual motor for one RL training episode
% (reset function used by train_rl_tuner.m). The values are spread around the
% real motor identified on the hardware (K = 16 RPM/V, tau = 0.054 s,
% dead-band 0.2 V), and the software load (controller output x0.6 to x2) is
% applied part-way through, as in the hardware tests, so the agent has to
% learn to re-tune.
in = setVariable(in, 'K_true',   16 + 10*rand,      'Workspace', mdl);  % 16-26 RPM/V
in = setVariable(in, 'tau_true', 0.03 + 0.09*rand,  'Workspace', mdl);  % 0.03-0.12 s
in = setVariable(in, 'V_dead',   0.5*rand,          'Workspace', mdl);  % 0-0.5 V
in = setVariable(in, 't_change', 20 + 50*rand,      'Workspace', mdl);  % 20-70 s
in = setVariable(in, 'K_factor', 0.6 + 1.4*rand,    'Workspace', mdl);  % x0.6-2.0
end
