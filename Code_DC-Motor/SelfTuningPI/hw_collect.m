function res = hw_collect()
% hw_collect  After a Monitor & Tune run of dc_motor_str.slx (set up with
% hw_setup), save the logged data and check the requirements.
% ACE-Lab, ACE-CORE.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'RL_PI_Tuner'));
method = evalin('base', 'hw_method');  scen = evalin('base', 'hw_scenario');
v = {'r_data', 'y_data', 'u_data', 'Kp_data', 'Ki_data', 'theta_est'};
out = struct();
for k = 1:numel(v)
    if evalin('base', sprintf('exist(''%s'', ''var'')', v{k}))
        out.(v{k}) = evalin('base', v{k});
    end
end
fn = fullfile(here, sprintf('hw_%s_%s.mat', regexprep(method, '\W', ''), scen));
save(fn, '-struct', 'out');
res = analyse_hw_run(out, method, scen, 40);
figure('Name', sprintf('%s, scenario %s (hardware)', method, scen));
t = res.t;
subplot(3,1,1); plot(t, res.r, 'k--', t, res.y, 'b'); grid on; ylabel('RPM'); xline(40, 'r:');
title(sprintf('%s, scenario %s (hardware)', method, scen));
subplot(3,1,2); plot(t, res.u); grid on; ylabel('Controller output (V)'); xline(40, 'r:');
subplot(3,1,3); yyaxis left; plot(t, res.Kp); ylabel('K_p'); yyaxis right; plot(t, res.Ki); ylabel('K_i');
grid on; xlabel('Time (s)'); xline(40, 'r:');
exportgraphics(gcf, strrep(fn, '.mat', '.png'), 'Resolution', 110);
end
