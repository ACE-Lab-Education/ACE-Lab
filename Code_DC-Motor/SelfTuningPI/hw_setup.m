function hw_setup(method, scenario)
% hw_setup  Configure dc_motor_str.slx for one hardware test, then press
% Monitor & Tune.
%   method   'Fixed PI', 'LLS + PI' or 'RLS self-tuning PI'
%   scenario 'A' (software load: gain x0.6 at t = 40 s) or
%            'B' (gain x2.0 at t = 40 s)
% LLS + PI uses gains designed by pole placement from the open-loop LLS
% identification of the real motor (..\hw_id_noload2.mat).
% ACE-Lab, ACE-CORE.
m = 'dc_motor_str';
here = fileparts(mfilename('fullpath'));
if ~bdIsLoaded(m), load_system(fullfile(here, [m '.slx'])); end
mws = get_param(m, 'ModelWorkspace');
mws.reload;
if upper(scenario) == 'A', assignin(mws, 'K_factor', 0.6); else, assignin(mws, 'K_factor', 2.0); end
switch method
    case 'Fixed PI'
        assignin(mws, 'adapt_on', 0);  assignin(mws, 'Kp0', 0.1);  assignin(mws, 'Ki0', 0.5);
    case 'LLS + PI'
        [Kp, Ki] = lls_pi_gains(fullfile(here, '..', 'hw_id_noload2.mat'));
        assignin(mws, 'adapt_on', 0);  assignin(mws, 'Kp0', Kp);  assignin(mws, 'Ki0', Ki);
    case 'RLS self-tuning PI'
        assignin(mws, 'adapt_on', 1);
    otherwise
        error('Unknown method %s', method);
end
assignin('base', 'hw_method', method);  assignin('base', 'hw_scenario', upper(scenario));
fprintf('dc_motor_str set up for %s, scenario %s (K_factor = %g). Press Monitor & Tune.\n', ...
    method, upper(scenario), getVariable(mws, 'K_factor'));
open_system(m);
end

function [Kp, Ki] = lls_pi_gains(f)
% PI gains by pole placement (zeta = 1, wn = 6 rad/s, Ts = 0.01 s) from a
% batch LLS fit of the open-loop identification data in file f
S = load(f, 'u_data', 'y_data');
Ts = 0.01;  D = 5;  Te = D*Ts;  zeta = 1;  wn = 6;
u = filter(0.2, [1 -0.8], double(S.u_data(:)));  y = double(S.y_data(:));
n = floor(numel(y)/D);
ud = mean(reshape(u(1:n*D), D, n), 1)';  yd = y(1:D:n*D);
k = (0:n-1)'*Te >= 1;  ud = ud(k);  yd = yd(k);
th = [-yd(1:end-1), ud(1:end-1), ones(numel(yd)-1, 1)] \ yd(2:end);
K = th(2)/(1 + th(1));  p = (-th(1))^(1/D);  bs = K*(1 - p);
p1 = -2*exp(-zeta*wn*Ts);  p2 = exp(-2*zeta*wn*Ts);
q0 = (p1 + 1 + p)/bs;  q1 = (p2 - p)/bs;
Kp = max(-q1, 0);  Ki = max((q0 + q1)/Ts, 0);
fprintf('LLS model: K = %.2f RPM/V, tau = %.3f s -> Kp = %.4f, Ki = %.4f\n', K, -Te/log(-th(1)), Kp, Ki);
end
