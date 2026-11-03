%% verify_requirements.m
% Checks each controller in compare_all.m against the speed-control
% requirements (run compare_all.m first; it saves compare_all_results.mat).
%
% Requirements (from the earlier ACE-Lab speed-control work, ace_step3/4):
%   REQ-1  Rise time (10-90 % of each reference step) < 1.0 s
%   REQ-2  Zero steady-state error: |mean error| over the last 1 s of each
%          step <= 0.1 RPM
%   REQ-3  Overshoot <= 5 % of the step size (1 RPM for a 20 RPM step)
%   REQ-4  Actuator activity: mean |change in PWM duty| over the last 1 s of
%          each step <= 5 counts per sample (limits chatter, heating and wear)
%   REQ-5  Robustness: REQ-1 to REQ-4 are met again within 10 s of a change in
%          the motor (gain x0.6 or x2), i.e. for every step from t = 50 s
%
% REQ-1 to REQ-3 describe the response; REQ-4 is about the actuator and
% REQ-5 about operating conditions - requirements are not only about the
% step response.
% ACE-Lab, ACE-CORE.

load('compare_all_results.mat', 'R', 'methods', 'scen');
Ts = 0.01;  Vs = 6;  Tstep = 3;
lim = struct('tr', 1.0, 'sse', 0.1, 'os', 5, 'chat', 5);
win = {[10 37], [50 80]};   % steps that finish before the change (t = 40 s) / after re-tuning
rows = {};
for s = 1:numel(scen)
    for k = 1:numel(methods)
        r = R(s,k).r;  y = R(s,k).y;  u = R(s,k).u;
        n = min([numel(r), numel(y), numel(u)]);
        r = r(1:n);  y = y(1:n);  duty = round(min(max(u(1:n), 0), Vs)*255/Vs);
        t = (0:n-1)'*Ts;
        stp = find(diff(r) ~= 0) + 1;
        P = zeros(numel(stp), 5);         % [t_step, tr, os%, sse, chatter]
        for i = 1:numel(stp)
            i0 = stp(i);  i1 = min(i0 + round(Tstep/Ts) - 1, n);
            r0 = r(i0-1);  r1 = r(i0);  z = (y(i0:i1) - r0)/(r1 - r0);   % normalised response
            k10 = find(z >= 0.1, 1);  k90 = find(z >= 0.9, 1);
            if isempty(k10) || isempty(k90), tr = Inf; else, tr = (k90 - k10)*Ts; end
            os = max(0, max(z) - 1)*100;
            last = max(i1 - round(1/Ts) + 1, i0):i1;
            sse = abs(mean(r(last) - y(last)));
            chat = mean(abs(diff(duty(last))));
            P(i,:) = [t(i0), tr, os, sse, chat];
        end
        for w = 1:2
            sel = P(:,1) >= win{w}(1) & P(:,1) < win{w}(2);
            Q = P(sel, :);
            ok = [all(Q(:,2) < lim.tr), all(Q(:,4) <= lim.sse), all(Q(:,3) <= lim.os), ...
                  all(Q(:,5) <= lim.chat)];
            rows(end+1, :) = {scen(s).name(1), methods{k}, w, max(Q(:,2)), max(Q(:,4)), ...
                max(Q(:,3)), max(Q(:,5)), ok}; %#ok<SAGROW>
        end
    end
end

%% Report
fprintf('\nWorst case over the steps in each window (limits in brackets)\n');
fprintf('%-3s %-22s %-12s %13s %13s %13s %15s   %s\n', '', 'Method', 'Window', ...
    'Rise [<1 s]', 'SSE [<=0.1]', 'OS [<=5 %]', 'Chatter [<=5]', 'REQ-1 2 3 4');
summary = zeros(numel(scen), numel(methods), 2);
for i = 1:size(rows, 1)
    wn = {'10-40 s', '50-80 s'};   % windows (steps starting 10-37 s and 50-77 s)
    ok = rows{i,8};
    fprintf('%-3s %-22s %-12s %11.2f s %9.2f RPM %11.1f %% %10.1f cnts   %s %s %s %s\n', ...
        rows{i,1}, rows{i,2}, wn{rows{i,3}}, rows{i,4}, rows{i,5}, rows{i,6}, rows{i,7}, ...
        pf(ok(1)), pf(ok(2)), pf(ok(3)), pf(ok(4)));
    s = double(rows{i,1}) - double('A') + 1;  k = find(strcmp(methods, rows{i,2}));
    summary(s, k, rows{i,3}) = all(ok);
end
fprintf('\nREQ-5 (all requirements met again after the motor changes):\n');
for s = 1:numel(scen)
    for k = 1:numel(methods)
        fprintf('  %-22s %-22s before: %s   after: %s   REQ-5: %s\n', scen(s).name, methods{k}, ...
            pf(summary(s,k,1)), pf(summary(s,k,2)), pf(summary(s,k,2)));
    end
end
save('requirements_results.mat', 'rows', 'summary', 'lim');

function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
