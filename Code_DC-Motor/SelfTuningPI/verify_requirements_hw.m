%% verify_requirements_hw.m
% Checks the hardware runs (hw_<method>_<scenario>.mat, saved by hw_collect)
% against the speed-control requirements:
%   REQ-1  Rise time (10-90 % of each reference step) < 1.0 s
%   REQ-2  |mean error| over the last 1 s of each step <= 0.1 RPM
%   REQ-3  Overshoot <= 5 % of the step size (1 RPM for a 20 RPM step)
%   REQ-4  Mean |change in PWM duty| over the last 1 s of each step <= 5 counts/sample
%   REQ-5  REQ-1 to REQ-4 met again within 10 s of the load change (t = 40 s),
%          i.e. for every step from t = 50 s
% A requirement passes only if EVERY step in the window meets it; the
% percentage of steps that meet it is also printed. Set smooth_w > 1 to judge
% rise time and overshoot on a moving average of the speed (to see how much
% of the overshoot is short speed dips rather than controller overshoot).
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));
runs = {'FixedPI','LLSPI','RLSselftuningPI','RLagent'};
scen = {'A', 0.6; 'B', 2.0};
Ts = 0.01;  V_motor = 6;  V_supply = 9;  t_load = 40;  smooth_w = 1;
lim = [1.0 0.1 5 5];                      % rise, SSE, OS, chatter
win = [10 37; 50 100];
fprintf('\n%-3s %-17s %-7s %8s %8s %8s %8s   %-28s REQ-5\n', '', 'Method', 'Window', ...
    'Rise', 'SSE', 'OS %', 'Chatter', 'REQ-1   REQ-2   REQ-3   REQ-4');
for s = 1:size(scen, 1)
    for k = 1:numel(runs)
        f = fullfile(here, sprintf('hw_%s_%s.mat', runs{k}, scen{s,1}));
        if ~exist(f, 'file'), continue; end
        d = load(f);
        r = d.r_data(:);  y = d.y_data(:);  u = d.u_data(:);
        n = min([numel(r) numel(y) numel(u)]);  r = r(1:n);  y = y(1:n);  u = u(1:n);
        t = (0:n-1)'*Ts;
        ua = min(max(u.*(1 + (scen{s,2} - 1)*(t >= t_load)), 0), V_motor);   % voltage at the motor
        duty = round(ua*255/V_supply);
        ys = movmean(y, smooth_w);
        stp = find(diff(r) ~= 0) + 1;  P = zeros(numel(stp), 5);
        for i = 1:numel(stp)
            i0 = stp(i);  i1 = min(i0 + 299, n);  r0 = r(i0-1);  r1 = r(i0);
            z = (ys(i0:i1) - r0)/(r1 - r0);
            k10 = find(z >= 0.1, 1);  k90 = find(z >= 0.9, 1);
            if isempty(k10) || isempty(k90), tr = Inf; else, tr = (k90 - k10)*Ts; end
            last = max(i1 - 99, i0):i1;
            P(i,:) = [t(i0), tr, abs(mean(r(last) - y(last))), max(0, max(z) - 1)*100, ...
                      mean(abs(diff(duty(last))))];
        end
        allok = false(1, 2);
        for w = 1:2
            Q = P(P(:,1) >= win(w,1) & P(:,1) < win(w,2), :);
            ok = [Q(:,2) < lim(1), Q(:,3) <= lim(2), Q(:,4) <= lim(3), Q(:,5) <= lim(4)];
            allok(w) = all(ok(:));
            txt = '';
            for j = 1:4, txt = [txt sprintf('%s%3.0f%% ', pf(all(ok(:,j))), 100*mean(ok(:,j)))]; end %#ok<AGROW>
            r5 = '';  if w == 2, r5 = pf(allok(2)); end
            fprintf('%-3s %-17s %-7s %6.2f s %8.2f %8.1f %8.1f   %s %s\n', scen{s,1}, runs{k}, ...
                sprintf('%d-%d s', win(w,1), win(w,2)), max(Q(:,2)), max(Q(:,3)), max(Q(:,4)), ...
                max(Q(:,5)), txt, r5);
        end
    end
end
fprintf('(worst step in each window; PASS/FAIL then the %% of steps that meet the requirement)\n');

function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
