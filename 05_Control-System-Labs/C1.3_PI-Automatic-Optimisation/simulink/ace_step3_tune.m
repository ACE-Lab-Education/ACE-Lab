%% ace_step3_tune.m  - optimise PI gains on the identified model
% Loads ace_id_model.mat and tunes Kp, Ki with fminsearch (no extra
% toolbox). Each cost evaluation simulates the real control loop exactly as
% it runs on the Uno: 50 Hz PI with clamping anti-windup, 0-Vs saturation,
% 8-bit PWM quantisation, encoder count quantisation (3575 counts/output rev)
% and the speed low-pass filter. Robust: worst case over plant gain x0.8,
% x1.0, x1.2. Requirements: rise time < 1 s, zero steady-state error at
% 30 rpm. Target used in the optimiser: rise time <= 0.6 s (margin), <5 %
% overshoot. Saves ace_tuned_gains.mat (read by ace_params.m).

ace_params;
load('ace_id_model.mat', 'ID');
scales = [0.8 1.0 1.2];
tr_target = 0.6;

cost = @(x) max(arrayfun(@(sc) loopCost(exp(x), ID, C, sc, tr_target), scales));
x0   = log([0.03 0.40]);
opts = optimset('Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-5, 'MaxFunEvals', 300);
fprintf('\nOptimising PI gains on the identified model (%s data)...\n', ID.source);
xb   = fminsearch(cost, x0, opts);
Kp = exp(xb(1));  Ki = exp(xb(2));

% ---- report predicted performance ----
fprintf('\nTuned gains: Kp = %.4f V/rpm, Ki = %.4f V/(rpm*s)\n', Kp, Ki);
fprintf('%-12s %10s %12s %14s\n', 'Plant gain', 'Rise [s]', 'Overshoot %', 'SS error [rpm]');
figure('Name', 'ACE-Lab tuned response (identified model)', 'Position', [100 100 900 450]);
hold on; grid on;
pred = zeros(numel(scales), 3);
for i = 1:numel(scales)
    [~, M, S] = loopCost([Kp Ki], ID, C, scales(i), tr_target);
    pred(i,:) = [M.tr M.os M.sse];
    fprintf('x%-11.1f %10.3f %12.2f %14.4f\n', scales(i), M.tr, M.os, M.sse);
    plot(S.t, S.yf, 'LineWidth', 1.3, 'DisplayName', sprintf('plant gain x%.1f', scales(i)));
end
yline(C.ref_rpm, 'k--', 'HandleVisibility', 'off');
xlabel('Time after set-point step [s]'); ylabel('Measured speed (filtered) [rpm]');
title(sprintf('Optimised PI on identified model: Kp=%.3f, Ki=%.3f', Kp, Ki));
legend('Location', 'southeast');
exportgraphics(gcf, 'ace_tuned_response.png', 'Resolution', 150);

source = sprintf('fminsearch on %s ID model (%s)', ID.source, datestr(now, 'yyyy-mm-dd HH:MM')); %#ok<TNOW1,DATST>
save('ace_tuned_gains.mat', 'Kp', 'Ki', 'source', 'pred', 'scales');
fprintf('Saved ace_tuned_gains.mat - ace_cl_arduino / ace_cl_virtual now use these gains.\n');

%% ------------------------------------------------------------------------
function [J, M, S] = loopCost(g, ID, C, sc, tr_target)
Kp = g(1); Ki = g(2);
Ts = C.Ts; T = 4; n = round(T/Ts); ref = C.ref_rpm;
na = ID.order; a = ID.a; b = ID.b*sc; c = ID.c;
yh = zeros(1, na); uh = zeros(1, na);       % plant history (newest first)
I = 0; yf = 0; pos = 0; cnt_prev = 0;
Y = zeros(n,1); YF = zeros(n,1); D = zeros(n,1);
for k = 1:n
    % plant (identified ARX) - true average speed over the last sample
    y = max(c + a*yh' + b*uh', 0);
    pos = pos + y*Ts/60*C.CPR_out;  cnt = floor(pos);
    ym = (cnt - cnt_prev)*60/(C.CPR_out*Ts); cnt_prev = cnt;   % encoder estimate
    yf = C.a*yf + (1 - C.a)*ym;                                  % low-pass
    e = ref - yf;
    v = Kp*e + I;  vs = min(max(v, 0), C.Vs);
    if ~((v > C.Vs && e > 0) || (v < 0 && e < 0)), I = I + Ki*Ts*e; end
    duty = min(max(round(vs*255/C.Vs), 0), 255);  V = duty*C.Vs/255;
    yh = [y yh(1:end-1)]; uh = [V uh(1:end-1)];
    Y(k) = y; YF(k) = yf; D(k) = duty;
end
t = (1:n)'*Ts;
i10 = find(YF >= 0.1*ref, 1); i90 = find(YF >= 0.9*ref, 1);
if isempty(i10) || isempty(i90), tr = 10; else, tr = t(i90) - t(i10); end
os  = max(0, (max(YF) - ref)/ref*100);
ss  = t > T - 1.5;
sse = mean(ref - YF(ss));
itae = sum(t.*abs(ref - YF))*Ts/(ref*T^2/2);
jit  = mean(abs(diff(D(ss))))/255;              % actuator chatter
J = itae + 0.5*jit + 20*max(0, tr/tr_target - 1)^2 + 5*max(0, os/5 - 1)^2 ...
    + 20*(sse/0.5)^2;
M = struct('tr', tr, 'os', os, 'sse', sse);
S = struct('t', t, 'y', Y, 'yf', YF, 'duty', D);
end
