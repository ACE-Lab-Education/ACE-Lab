%% ace_step4_verify.m  - check the closed-loop run against the requirements
% Run after ace_cl_arduino (Monitor & Tune, 8 s) or ace_cl_virtual, with
% `out` in the workspace. Uses the encoder speed measured by the Uno.
%   Requirement 1: rise time (10-90 %) < 1 s
%   Requirement 2: zero steady-state error at 30 rpm
%                  (mean error over the last 2 s within +/-0.1 rpm,
%                   i.e. below the 0.84 rpm encoder resolution per sample)

ace_params;
if ~exist('cl_source', 'var'), cl_source = 'hardware'; end
[tr_, r]  = aceGetSignal(out, 'ref_rpm');
[tf_, yf] = aceGetSignal(out, 'rpm_filt');
[tw_, yr] = aceGetSignal(out, 'rpm_raw');
[td_, d]  = aceGetSignal(out, 'pwm_duty');
t  = (0:C.Ts:min([tr_(end) tf_(end) tw_(end) td_(end)]))';
r  = interp1(tr_, r,  t, 'previous', 'extrap');
yf = interp1(tf_, yf, t, 'previous', 'extrap');
yr = interp1(tw_, yr, t, 'previous', 'extrap');
d  = interp1(td_, d,  t, 'previous', 'extrap');

kStep = find(r > 0.5*C.ref_rpm, 1);  tStep = t(kStep);
s  = t >= tStep;
si = stepinfo(yf(s), t(s) - tStep, C.ref_rpm, 0);
last = t > t(end) - 2;
sse  = mean(C.ref_rpm - yf(last));
sseR = mean(C.ref_rpm - yr(last));

pass1 = si.RiseTime < 1.0;  pass2 = abs(sse) <= 0.1;
fprintf('\n=== Closed-loop verification (%s) ===\n', cl_source);
fprintf('Gains: Kp = %.4f, Ki = %.4f  (%s)\n', C.Kp, C.Ki, C.gain_source);
fprintf('Rise time (10-90 %%):   %.3f s   [< 1 s]      %s\n', si.RiseTime, pf(pass1));
fprintf('Overshoot:             %.2f %%\n', si.Overshoot);
fprintf('Settling time (2 %%):   %.3f s\n', si.SettlingTime);
fprintf('Steady-state error:    %.3f rpm (filtered), %.3f rpm (raw)  [0 +/-0.1]  %s\n', sse, sseR, pf(pass2));
fprintf('Steady-state PWM duty: %.1f / 255\n', mean(d(last)));
if pass1 && pass2, fprintf('REQUIREMENTS MET.\n'); else, fprintf('REQUIREMENTS NOT MET - see plot.\n'); end

V = table(string(cl_source), C.Kp, C.Ki, si.RiseTime, si.Overshoot, si.SettlingTime, sse, sseR, pass1, pass2, ...
    'VariableNames', {'Source','Kp','Ki','RiseTime_s','Overshoot_pct','Settling_s','SSE_filt_rpm','SSE_raw_rpm','PassRise','PassSSE'});
fn = sprintf('ace_verify_%s', regexprep(char(cl_source), '\W', '_'));
writetable(V, [fn '.csv']);

figure('Name', ['ACE-Lab closed-loop verification - ' char(cl_source)], 'Position', [120 120 950 620]);
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; plot(t, yr, 'Color', [.7 .7 .7]); hold on; plot(t, yf, 'LineWidth', 1.5);
stairs(t, r, 'k--'); grid on; ylabel('Output speed [rpm]');
yline([0.1 0.9]*C.ref_rpm, ':', 'Color', [.5 .5 .5]);
title(sprintf('%s: rise %.2f s, SS error %.3f rpm  (Kp=%.3f, Ki=%.3f)', cl_source, si.RiseTime, sse, C.Kp, C.Ki));
legend('Encoder raw', 'Encoder filtered', 'Set-point', 'Location', 'southeast');
nexttile; stairs(t, d, 'LineWidth', 1.2); grid on; ylabel('PWM duty [0-255]'); xlabel('Time [s]');
exportgraphics(gcf, [fn '.png'], 'Resolution', 150);

function s = pf(p), if p, s = 'PASS'; else, s = 'FAIL'; end, end
