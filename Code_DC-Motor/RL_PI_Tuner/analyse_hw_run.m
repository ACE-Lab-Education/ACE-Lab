function res = analyse_hw_run(out, method, scenario, tLoad)
% analyse_hw_run  Requirement check for one closed-loop hardware run
%   res = analyse_hw_run(out, method, scenario)
%   res = analyse_hw_run(out, method, scenario, tLoad)
%
%   out       simulation output of dc_motor_str.slx or dc_motor_rl.slx after a
%             Monitor & Tune run (r_data, y_data, u_data, Kp_data, Ki_data)
%   method    e.g. 'Fixed PI', 'LLS + PI', 'RLS self-tuning PI', 'RL agent'
%   scenario  'A' (load added) or 'B' (load removed)
%   tLoad     time the load was changed [s]; if omitted it is found from the
%             data, as the largest change in the steady-state voltage needed
%             at the same reference speed
%
% The result is added to hw_results.mat. Requirements as verify_requirements.m:
%   REQ-1 rise time < 1 s, REQ-2 |SSE| <= 0.1 RPM, REQ-3 overshoot <= 5 %,
%   REQ-4 chatter <= 5 PWM counts/sample, REQ-5 met again within 10 s of the
%   load change.
% ACE-Lab, ACE-CORE.

Ts = 0.01;  Vs = 6;  Tstep = 3;
g = @(v) double(reshape(out.(v), [], 1));
r = g('r_data');  y = g('y_data');  u = g('u_data');  Kp = g('Kp_data');  Ki = g('Ki_data');
n = min([numel(r) numel(y) numel(u) numel(Kp)]);
r = r(1:n);  y = y(1:n);  u = u(1:n);  Kp = Kp(1:n);  Ki = Ki(1:n);
t = (0:n-1)'*Ts;  duty = round(min(max(u, 0), Vs)*255/Vs);

% ---- per-step measures ----
stp = find(diff(r) ~= 0) + 1;
P = nan(numel(stp), 7);      % [t_step, r_new, tr, os%, sse, chatter, u_ss]
for i = 1:numel(stp)
    i0 = stp(i);  i1 = min(i0 + round(Tstep/Ts) - 1, n);
    r0 = r(i0-1);  r1 = r(i0);  z = (y(i0:i1) - r0)/(r1 - r0);
    k10 = find(z >= 0.1, 1);  k90 = find(z >= 0.9, 1);
    if isempty(k10) || isempty(k90), tr = Inf; else, tr = (k90 - k10)*Ts; end
    last = max(i1 - round(1/Ts) + 1, i0):i1;
    P(i,:) = [t(i0), r1, tr, max(0, max(z) - 1)*100, abs(mean(r(last) - y(last))), ...
              mean(abs(diff(duty(last)))), mean(u(last))];
end

% ---- time of the load change ----
if nargin < 4 || isempty(tLoad)
    best = 0;  tLoad = NaN;
    for i = 3:size(P, 1)
        j = i - 2;                                   % previous step at the same speed
        d = abs(P(i,7) - P(j,7));
        if P(i,1) > 15 && d > best, best = d;  tLoad = P(i,1) - Tstep; end
    end
    fprintf('Load change detected at about t = %.0f s (steady-state voltage changed by %.2f V)\n', tLoad, best);
end

% ---- requirements before / after ----
lim = [1.0 0.1 5 5];
before = P(:,1) >= 10 & P(:,1) + Tstep <= tLoad;
after  = P(:,1) >= tLoad + 10;
W = {before, after};  wc = cell(1, 2);
for w = 1:2
    Q = P(W{w}, :);
    v = [max(Q(:,3)), max(Q(:,5)), max(Q(:,4)), max(Q(:,6))];
    wc{w} = struct('tr', v(1), 'sse', v(2), 'os', v(3), 'chat', v(4), ...
        'ok', [v(1) < lim(1), v(2) <= lim(2), v(3) <= lim(3), v(4) <= lim(4)], 'nsteps', size(Q, 1));
end
res = struct('method', method, 'scenario', scenario, 'tLoad', tLoad, 'before', wc{1}, ...
    'after', wc{2}, 'P', P, 't', t, 'r', r, 'y', y, 'u', u, 'Kp', Kp, 'Ki', Ki, ...
    'when', datestr(now)); %#ok<TNOW1,DATST>
pf = {'FAIL', 'PASS'};
for w = 1:2
    nm = {'before', 'after '};  c = wc{w};
    fprintf('%s %-20s %s: rise %.2f s, SSE %.2f RPM, OS %.1f %%, chatter %.1f  -> REQ-1..4: %s %s %s %s\n', ...
        scenario, method, nm{w}, c.tr, c.sse, c.os, c.chat, pf{c.ok(1)+1}, pf{c.ok(2)+1}, pf{c.ok(3)+1}, pf{c.ok(4)+1});
end
fprintf('REQ-5 (requirements met again after the load change): %s\n', pf{all(wc{2}.ok)+1});

% ---- store ----
f = fullfile(fileparts(mfilename('fullpath')), 'hw_results.mat');
if exist(f, 'file'), S = load(f, 'HW'); HW = S.HW; else, HW = struct([]); end
if ~isempty(HW)
    keep = ~(strcmp({HW.method}, method) & strcmp({HW.scenario}, scenario));
    HW = HW(keep);
end
if isempty(HW), HW = res; else, HW(end+1) = res; end
save(f, 'HW');
end
