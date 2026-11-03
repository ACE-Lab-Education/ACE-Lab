%% ace_step2_identify.m  - linear least-squares model from logged data
% Run after ace_id_arduino (or ace_id_virtual) so that `out` is in the
% workspace. Fits discrete ARX models from applied voltage u [V] to the
% measured speed y [rpm] at the control rate Ts:
%   order 1: y(k) = a1*y(k-1) + b1*u(k-1) + c
%   order 2: y(k) = a1*y(k-1) + a2*y(k-2) + b1*u(k-1) + b2*u(k-2) + c
% (c captures the friction dead-band). Parameters by linear least squares
% on the first 60 % of the data; the rest is used to validate by free-run
% simulation. Saves ace_id_data.mat and ace_id_model.mat.

ace_params;
if ~exist('id_source', 'var'), id_source = 'hardware'; end
if ~exist('out', 'var')
    if exist('ace_id_data.mat', 'file')
        load('ace_id_data.mat', 't', 'u', 'y');  fprintf('Using saved ace_id_data.mat\n');
    else
        error('No `out` in workspace - run ace_id_arduino (Monitor & Tune) first.');
    end
else
    [tu, u] = aceGetSignal(out, 'V_applied');
    [ty, y] = aceGetSignal(out, 'rpm_raw');
    t = (max(tu(1), ty(1)) : C.Ts : min(tu(end), ty(end)))';
    u = interp1(tu, u, t, 'previous');   y = interp1(ty, y, t, 'previous');
    save('ace_id_data.mat', 't', 'u', 'y', 'id_source');
end

% keep data once excitation is running (skip start-up transient)
keep = t >= C.id_start + 0.5;
t = t(keep); u = u(keep); y = y(keep);
n = numel(y);  nEst = round(0.6*n);

%% ---- Least squares ----
k  = (3:n)';
Phi1 = [y(k-1),           u(k-1),          ones(size(k))];
Phi2 = [y(k-1), y(k-2),   u(k-1), u(k-2),  ones(size(k))];
est  = k <= nEst;
th1 = Phi1(est,:) \ y(k(est));          % [a1 b1 c]
th2 = Phi2(est,:) \ y(k(est));          % [a1 a2 b1 b2 c]

ys1 = arxSim(th1(1), th1(2), th1(3), u, y);
ys2 = arxSim(th2(1:2), th2(3:4), th2(5), u, y);
val = (nEst+1):n;
fitv = @(ys) 100*(1 - norm(y(val)-ys(val))/norm(y(val)-mean(y(val))));
fit1 = fitv(ys1);  fit2 = fitv(ys2);

if fit2 > fit1 + 2
    ID.order = 2; ID.a = th2(1:2)'; ID.b = th2(3:4)'; ID.c = th2(5); ysim = ys2; ID.fit = fit2;
else
    ID.order = 1; ID.a = th1(1);    ID.b = th1(2);    ID.c = th1(3); ysim = ys1; ID.fit = fit1;
end
ID.Ts = C.Ts;  ID.source = id_source;
ID.K   = sum(ID.b)/(1 - sum(ID.a));                % static gain   [rpm/V]
ID.off = ID.c/(1 - sum(ID.a));                     % offset        [rpm]
ID.V_dead = -ID.off/ID.K;                          % dead-band     [V]
p = roots([1 -ID.a]);  ID.tau = -C.Ts/log(max(abs(p)));   % dominant time constant [s]
save('ace_id_model.mat', 'ID');

fprintf('\n=== Least-squares identification (%s data) ===\n', id_source);
fprintf('Order 1 fit: %.1f %%   Order 2 fit: %.1f %%  -> using order %d\n', fit1, fit2, ID.order);
fprintf('a = [%s]  b = [%s]  c = %.3f\n', num2str(ID.a, '%.4f '), num2str(ID.b, '%.4f '), ID.c);
fprintf('Static gain %.2f rpm/V, dead-band %.2f V, time constant %.3f s\n', ID.K, ID.V_dead, ID.tau);

figure('Name', 'ACE-Lab system identification', 'Position', [80 80 1000 600]);
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; stairs(t, u, 'LineWidth', 1.2); grid on; ylabel('Applied voltage [V]');
title(sprintf('Least-squares ARX model (order %d) from %s data', ID.order, id_source));
nexttile; plot(t, y, 'Color', [.6 .6 .6]); hold on; plot(t, ysim, 'LineWidth', 1.4);
xline(t(nEst), 'k--', 'estimation | validation'); grid on;
ylabel('Speed [rpm]'); xlabel('Time [s]');
legend('Measured (encoder)', sprintf('Model, validation fit %.1f %%', ID.fit), 'Location', 'best');
exportgraphics(gcf, 'ace_id_fit.png', 'Resolution', 150);

function ys = arxSim(a, b, c, u, y)
na = numel(a); ys = y;
for kk = 3:numel(y)
    ys(kk) = c;
    for j = 1:na, ys(kk) = ys(kk) + a(j)*ys(kk-j) + b(j)*u(kk-j); end
end
end
