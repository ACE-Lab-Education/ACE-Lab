%% test_sysid.m
% Desktop test of the identification models (no hardware needed):
%  1. checks that the hardware models update (compile) for the Arduino Mega 2560;
%  2. runs the virtual models and compares the estimates with the true
%     virtual-motor values and with lls_offline.m.
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));
cd(here);

for m = {'dc_motor_lls', 'dc_motor_rls'}
    try
        load_system(m{1});
        set_param(m{1}, 'SimulationCommand', 'update');
        fprintf('%s: updates OK for Arduino Mega 2560\n', m{1});
    catch err
        fprintf(2, '%s: %s\n', m{1}, err.getReport('basic'));
    end
end

res = {};
for m = {'dc_motor_lls_virtual', 'dc_motor_rls_virtual'}
    load_system(m{1});
    out = sim(m{1});
    assignin('base', 'out', out);
    th = squeeze(out.theta_est);  if size(th, 1) == 3, th = th'; end
    a1 = th(end,1);  b0 = th(end,2);
    res(end+1, :) = {m{1}, b0/(1 + a1), -0.05/log(-a1), -th(end,3)/b0}; %#ok<SAGROW>
    evalc('lls_offline');
    res(end+1, :) = {[m{1} ' -> lls_offline'], K, Tau, V_dead}; %#ok<SAGROW>
end
mws = get_param('dc_motor_lls_virtual', 'ModelWorkspace');
fprintf('\n%-38s %10s %10s %10s\n', '', 'K (RPM/V)', 'tau (s)', 'dead (V)');
fprintf('%-38s %10.2f %10.3f %10.2f\n', 'True virtual motor', ...
    getVariable(mws, 'K_true'), getVariable(mws, 'tau_true'), getVariable(mws, 'V_dead'));
for k = 1:size(res, 1)
    fprintf('%-38s %10.2f %10.3f %10.2f\n', res{k,:});
end
