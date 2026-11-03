%% ACE-LAB: AUTOMATIC PI CONTROLLER OPTIMISATION
% Grid search with actuator saturation and conditional anti-windup.
% Requires MATLAB R2020a or later for exportgraphics and tiledlayout.
%
% IMPORTANT MODEL ASSUMPTION:
% G(s) = 30/(0.5*s + 1) maps APPLIED VOLTAGE (V) to SPEED (RPM).
% The normalised PI output is limited to [0,1] and multiplied by 6 V.
% Check the units of your experimentally identified transfer function.
%
% Gain units: Kp = 1/RPM; Ki = 1/(RPM*s) for normalised PI output.

clear; clc; close all;

%% 1. Plant, reference and actuator
Kplant = 30;                % Steady-state plant gain (RPM/V)
tau = 0.5;                  % Plant time constant (s)
reference = 30;             % Step reference (RPM)
Vmax = 6;                   % Maximum applied motor voltage (V)
uMin = 0;                   % Normalised actuator lower limit
uMax = 1;                   % Normalised actuator upper limit

dt = 0.01;                  % Simulation step (s)
Tend = 5;                   % Simulation length (s)
t = 0:dt:Tend;

%% 2. Performance requirements
spec.TsMax = 3.0;           % Settling-time limit (s), using +/-2% band
spec.OSMax = 5.0;           % Overshoot limit (%)
spec.SSEMaxPct = 0.5;       % Final error at t=Tend, as % of reference

%% 3. Grid-search ranges
KpValues = linspace(0.001, 0.08, 60);
KiValues = linspace(0.001, 0.20, 60);

nKp = numel(KpValues);
nKi = numel(KiValues);
Jgrid = nan(nKp, nKi);
TsGrid = nan(nKp, nKi);
SaturationGrid = nan(nKp, nKi);
feasibleGrid = false(nKp, nKi);

bestOverall.cost = inf;
bestFeasible.cost = inf;

%% 4. Run every candidate controller
for i = 1:nKp
    for j = 1:nKi
        Kp = KpValues(i);
        Ki = KiValues(j);

        data = simulatePI(Kp, Ki, reference, t, Kplant, tau, ...
                          Vmax, uMin, uMax);
        m = measurePerformance(t, data, reference);

        % Base cost: smaller is better.
        % Weights are illustrative design choices, NOT universal constants.
        components.IAE = m.IAE;
        components.settling = 2.0*m.Ts;
        components.overshoot = 0.5*m.OS;
        components.finalError = 10.0*m.SSEpct;
        components.saturation = 50.0*m.satFraction;

        % Quadratic penalties for violating performance requirements.
        components.penalty = 1000*( ...
              max(0, m.Ts - spec.TsMax)^2 ...
            + max(0, m.OS - spec.OSMax)^2 ...
            + max(0, m.SSEpct - spec.SSEMaxPct)^2);

        J = components.IAE + components.settling + ...
            components.overshoot + components.finalError + ...
            components.saturation + components.penalty;

        feasible = m.Ts <= spec.TsMax && ...
                   m.OS <= spec.OSMax && ...
                   m.SSEpct <= spec.SSEMaxPct;

        Jgrid(i,j) = J;
        TsGrid(i,j) = m.Ts;
        SaturationGrid(i,j) = 100*m.satFraction;
        feasibleGrid(i,j) = feasible;

        if J < bestOverall.cost
            bestOverall = saveCandidate(Kp, Ki, J, data, m, components);
        end
        if feasible && J < bestFeasible.cost
            bestFeasible = saveCandidate(Kp, Ki, J, data, m, components);
        end
    end
end

%% 5. Select best feasible gains where possible
if isfinite(bestFeasible.cost)
    best = bestFeasible;
    fprintf('Selected the lowest-cost controller that meets all limits.\n');
else
    best = bestOverall;
    warning(['No candidate met every requirement. ' ...
             'Showing the lowest penalised-cost candidate instead.']);
end

%% 6. Print the selected result
fprintf('\nACE-LAB: OPTIMISED PI CONTROLLER\n');
fprintf('========================================\n');
fprintf('Kp                  = %.6f\n', best.Kp);
fprintf('Ki                  = %.6f\n', best.Ki);
fprintf('Total cost          = %.4f\n', best.cost);
fprintf('Settling time       = %.3f s (2%% band)\n', best.metrics.Ts);
fprintf('Overshoot           = %.3f %%\n', best.metrics.OS);
fprintf('Final error         = %.3f %%\n', best.metrics.SSEpct);
fprintf('IAE                 = %.3f RPM*s\n', best.metrics.IAE);
fprintf('Clipped time        = %.3f s\n', best.metrics.satTime);
fprintf('Clipped fraction    = %.2f %%\n', 100*best.metrics.satFraction);
fprintf('Peak applied V      = %.3f V\n', max(best.data.V));
fprintf('Requirements met    = %s\n', string( ...
    best.metrics.Ts <= spec.TsMax && ...
    best.metrics.OS <= spec.OSMax && ...
    best.metrics.SSEpct <= spec.SSEMaxPct));

%% 7. Presentation-ready figure defaults
axisFont = 16;
labelFont = 20;
titleFont = 21;
legendFont = 15;
plotWidth = 2.2;

%% FIGURE 1: Speed response
f1 = figure('Color','w','Position',[100 100 1100 650]);
plot(t, best.data.y, 'LineWidth', plotWidth); hold on;
yline(reference, '--', 'Reference', 'LineWidth', 1.6);
yline(reference*1.02, ':', '+2% limit', 'LineWidth', 1.2);
yline(reference*0.98, ':', '-2% limit', 'LineWidth', 1.2);
xline(spec.TsMax, '--', '3 s requirement', 'LineWidth', 1.5);
if best.metrics.Ts <= Tend
    xline(best.metrics.Ts, ':', 'Measured settling time', 'LineWidth', 1.5);
end
xlabel('Time (s)'); ylabel('Speed (RPM)');
title('Optimised Speed Response'); grid on;
xlim([0 Tend]);

%% FIGURE 2: Requested versus applied motor voltage
f2 = figure('Color','w','Position',[140 120 1100 650]);
plot(t, Vmax*best.data.uRaw, '--', 'LineWidth', plotWidth); hold on;
plot(t, best.data.V, '-', 'LineWidth', plotWidth);
yline(Vmax, ':', '6 V limit', 'LineWidth', 1.8);
yline(0, ':', '0 V limit', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Requested and Applied Motor Voltage');
legend('Requested before saturation', 'Applied after saturation', ...
       'Location', 'best');
grid on; xlim([0 Tend]);

%% FIGURE 3: Normalised PI control signal
f3 = figure('Color','w','Position',[180 140 1100 650]);
plot(t, best.data.uRaw, '--', 'LineWidth', plotWidth); hold on;
plot(t, best.data.uSat, '-', 'LineWidth', plotWidth);
yline(1, ':', 'Upper limit', 'LineWidth', 1.4);
yline(0, ':', 'Lower limit', 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('Normalised PI output');
title('PI Output Before and After Saturation');
legend('Requested output', 'Limited output', 'Location', 'best');
grid on; xlim([0 Tend]);

%% FIGURE 4: Saturation and tracking error
f4 = figure('Color','w','Position',[220 160 1100 760]);
tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
nexttile;
stairs(t, best.data.satFlag, 'LineWidth', plotWidth);
ylabel('Clipping flag');
title('When Does the Motor Voltage Saturate?');
ylim([-0.1 1.1]); yticks([0 1]); xlim([0 Tend]); grid on;
nexttile;
plot(t, best.data.error, 'LineWidth', plotWidth);
yline(0, ':', 'Zero error');
xlabel('Time (s)'); ylabel('Speed error (RPM)');
xlim([0 Tend]); grid on;

%% FIGURE 5: 3-D optimisation cost surface
[KpMesh, KiMesh] = meshgrid(KpValues, KiValues);
f5 = figure('Color','w','Position',[260 180 1100 700]);
surf(KpMesh, KiMesh, Jgrid', 'EdgeColor','none'); hold on;
plot3(best.Kp, best.Ki, best.cost, 'kp', ...
      'MarkerSize',16, 'MarkerFaceColor','w', 'LineWidth',2);
xlabel('K_p'); ylabel('K_i'); zlabel('Total cost');
title('PI Optimisation Cost Surface');
colorbar; grid on; view(45,30);

%% FIGURE 6: 2-D optimisation map
f6 = figure('Color','w','Position',[300 200 1100 700]);
contourf(KpMesh, KiMesh, Jgrid', 30, 'LineStyle','none'); hold on;
plot(best.Kp, best.Ki, 'kp', 'MarkerSize',17, ...
     'MarkerFaceColor','w', 'LineWidth',2);
xlabel('K_p'); ylabel('K_i');
title('PI Optimisation Map and Selected Gains');
cb = colorbar; cb.Label.String = 'Total cost'; grid on;

%% FIGURE 7: Contributions to the selected controller's cost
f7 = figure('Color','w','Position',[340 220 1100 650]);
c = best.components;
values = [c.IAE, c.settling, c.overshoot, ...
          c.finalError, c.saturation, c.penalty];
bar(values);
xticklabels({'IAE','Settling','Overshoot','Final error', ...
             'Saturation','Penalty'});
xtickangle(20); ylabel('Cost contribution');
title('Why Did the Optimiser Select These Gains?');
grid on;

%% 8. Enlarge all graph labels, ticks, titles and legends
figs = [f1 f2 f3 f4 f5 f6 f7];
for fig = figs
    allAxes = findall(fig, 'Type', 'axes');
    for ax = allAxes'
        ax.FontSize = axisFont;
        ax.LineWidth = 1.3;
        ax.XLabel.FontSize = labelFont;
        ax.YLabel.FontSize = labelFont;
        ax.ZLabel.FontSize = labelFont;
        ax.Title.FontSize = titleFont;
    end
    allLegends = findall(fig, 'Type', 'legend');
    for lgd = allLegends'
        lgd.FontSize = legendFont;
    end
end

%% 9. Optional 300-DPI export for PowerPoint
exportFigures = true;
if exportFigures
    folder = 'PI_Optimisation_Plots';
    if ~exist(folder,'dir')
        mkdir(folder);
    end
    for k = 1:numel(figs)
        exportgraphics(figs(k), fullfile(folder, ...
            sprintf('PI_Optimisation_Figure_%d.png',k)), ...
            'Resolution', 300);
    end
    fprintf('\nSaved seven 300-DPI figures to %s.\n', folder);
end

%% ------------------- LOCAL FUNCTIONS -------------------
function data = simulatePI(Kp, Ki, r, t, Kplant, tau, Vmax, uMin, uMax)
    N = numel(t);
    dt = t(2) - t(1);
    a = exp(-dt/tau);       % Exact step-to-step first-order update

    y = zeros(1,N);
    error = zeros(1,N);
    uRaw = zeros(1,N);
    uSat = zeros(1,N);
    V = zeros(1,N);
    satFlag = false(1,N);
    Iterm = 0;

    for k = 1:N
        error(k) = r - y(k);
        uRaw(k) = Kp*error(k) + Iterm;
        uSat(k) = min(max(uRaw(k),uMin),uMax);
        V(k) = Vmax*uSat(k);
        satFlag(k) = uRaw(k) > uMax + 1e-10 || ...
                     uRaw(k) < uMin - 1e-10;

        if k < N
            % Conditional integration prevents integral wind-up.
            % Continue integrating when the error drives the requested
            % control BACK towards its permitted range.
            integrate = ~satFlag(k) || ...
                (uRaw(k) > uMax && error(k) < 0) || ...
                (uRaw(k) < uMin && error(k) > 0);
            if integrate
                Iterm = Iterm + Ki*error(k)*dt;
            end

            % Plant: tau*dy/dt + y = Kplant*V
            y(k+1) = a*y(k) + (1-a)*Kplant*V(k);
        end
    end

    data.y = y;
    data.error = error;
    data.uRaw = uRaw;
    data.uSat = uSat;
    data.V = V;
    data.satFlag = satFlag;
end

function m = measurePerformance(t, data, r)
    y = data.y;
    m.IAE = trapz(t, abs(data.error));
    m.OS = max(0, 100*(max(y)-r)/r);
    m.SSEpct = 100*abs(r-y(end))/r;

    % Settling time: first time after which the output remains within
    % +/-2% of the reference for the rest of the simulation.
    lastOutside = find(abs(y-r) > 0.02*abs(r), 1, 'last');
    if isempty(lastOutside)
        m.Ts = 0;
    elseif lastOutside == numel(t)
        m.Ts = t(end) + (t(2)-t(1)); % No settling observed
    else
        m.Ts = t(lastOutside+1);
    end

    m.satTime = trapz(t, double(data.satFlag));
    m.satFraction = m.satTime/(t(end)-t(1));
end

function c = saveCandidate(Kp, Ki, J, data, metrics, components)
    c.Kp = Kp;
    c.Ki = Ki;
    c.cost = J;
    c.data = data;
    c.metrics = metrics;
    c.components = components;
end
