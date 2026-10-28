%% build_ace_speed_control.m
% ACE-Lab actuator: closed-loop speed control of a 298:1 DC gearmotor with a
% 12 CPR encoder on an Arduino Uno.
%
% 1. Builds ace_speed_sim.slx     - PI controller + speed estimator + motor
%    model; simulates and checks rise time < 1 s and zero steady-state error
%    at 30 rpm (nominal motor and +/-20 % motor spread).
% 2. Builds ace_speed_arduino.slx - the SAME controller and estimator wired
%    to the Arduino Uno Encoder, PWM and Digital Output blocks, configured
%    for the Arduino Uno and External mode. It is NOT deployed - use
%    Hardware tab > Monitor & Tune (or Build, Deploy & Start) yourself.
%
% Needs: Simulink Support Package for Arduino Hardware, ace_params.m.

clear; clc; close all;
ace_params;
simMdl = 'ace_speed_sim';
hwMdl  = 'ace_speed_arduino';

%% ======================= 1. Simulation model ============================
newModel(simMdl);
set_param(simMdl, 'PreLoadFcn', 'ace_params;', ...
    'InitFcn', 'if ~exist(''C'',''var''), ace_params; end', 'StopTime', '5', ...
    'SolverType', 'Variable-step', 'Solver', 'ode23tb', 'MaxStep', '1e-3');

addSpeedController(simMdl);
addSpeedEstimator(simMdl);
addMotorModel(simMdl);

add_block('simulink/Sources/Step', [simMdl '/Speed set-point (rpm)'], ...
    'Time', 'C.t_step', 'Before', '0', 'After', 'C.ref_rpm', 'SampleTime', 'C.Ts');
add_block('simulink/Signal Routing/Mux', [simMdl '/Mux'], 'Inputs', '3');
add_block('simulink/Sinks/Scope', [simMdl '/Speed scope']);
logBlk(simMdl, 'ref_log',  'ref_rpm');
logBlk(simMdl, 'true_log', 'rpm_true');
logBlk(simMdl, 'meas_log', 'rpm_meas');
logBlk(simMdl, 'duty_log', 'pwm_duty');

ln = @(a,b) add_line(simMdl, a, b, 'autorouting', 'smart');
ln('Speed set-point (rpm)/1', 'Speed Controller/1');
ln('Speed Estimator/1',       'Speed Controller/2');
ln('Speed Controller/1',      'Motor + Gearbox + Encoder/1');
ln('Motor + Gearbox + Encoder/1', 'Speed Estimator/1');
ln('Speed set-point (rpm)/1', 'Mux/1');
ln('Motor + Gearbox + Encoder/2', 'Mux/2');
ln('Speed Estimator/1',       'Mux/3');
ln('Mux/1', 'Speed scope/1');
ln('Speed set-point (rpm)/1', 'ref_log/1');
ln('Motor + Gearbox + Encoder/2', 'true_log/1');
ln('Speed Estimator/1',       'meas_log/1');
ln('Speed Controller/1',      'duty_log/1');
tidy(simMdl); save_system(simMdl);

%% ---- Simulate: nominal and +/-20 % motor spread ----
cases = [1.0 0.8 1.2];  names = {'Nominal motor','Motor -20 %','Motor +20 %'};
R = table('Size',[3 6],'VariableTypes',{'string','double','double','double','double','double'}, ...
    'VariableNames',{'Case','RiseTime_s','Overshoot_pct','SettlingTime_s','SSerror_rpm','SSduty'});
figure('Name','ACE-Lab speed control - simulation','Position',[80 80 1000 700]);
tl = tiledlayout(2,1,'TileSpacing','compact');
title(tl, sprintf('PI speed control, 298:1 gearmotor, %g rpm set-point (Kp=%g, Ki=%g, Ts=%g s)', ...
    C.ref_rpm, C.Kp, C.Ki, C.Ts));
ax1 = nexttile; hold on; grid on; ylabel('Output speed [rpm]');
ax2 = nexttile; hold on; grid on; ylabel('PWM duty [0-255]'); xlabel('Time [s]');
for k = 1:numel(cases)
    C.scale = cases(k);                    % motor constant spread
    out = sim(simMdl);
    t  = out.rpm_true.Time;  y = out.rpm_true.Data(:);
    ym = out.rpm_meas.Data(:); d = out.pwm_duty.Data(:); td = out.pwm_duty.Time;
    s  = t >= C.t_step;
    si = stepinfo(y(s), t(s) - C.t_step, C.ref_rpm, 0);
    last = t > t(end) - 1.5;
    R(k,:) = {names{k}, si.RiseTime, si.Overshoot, si.SettlingTime, ...
              mean(C.ref_rpm - y(last)), mean(d(td > td(end)-1.5))};
    plot(ax1, t, y, 'LineWidth', 1.4, 'DisplayName', [names{k} ' (true)']);
    if k == 1
        stairs(ax1, out.rpm_meas.Time, ym, ':', 'DisplayName', 'Nominal (encoder estimate)');
    end
    stairs(ax2, td, d, 'LineWidth', 1.1, 'DisplayName', names{k});
end
yline(ax1, C.ref_rpm, 'k--', 'DisplayName', 'Set-point');
yline(ax1, [0.1 0.9]*C.ref_rpm, ':', 'Color', [.6 .6 .6], 'HandleVisibility', 'off');
legend(ax1, 'Location', 'southeast'); legend(ax2, 'Location', 'southeast');
exportgraphics(gcf, 'ace_speed_sim_results.png', 'Resolution', 150);

R.Pass_rise = R.RiseTime_s < C.tr_max;
R.Pass_SSE  = abs(R.SSerror_rpm) < 0.05;          % < 0.05 rpm (0.17 %)
C.scale = 1;
disp(R);
writetable(R, 'ace_speed_sim_results.csv');
if all(R.Pass_rise & R.Pass_SSE)
    fprintf('\nALL CASES PASS: rise time < %.1f s and zero steady-state error.\n', C.tr_max);
else
    fprintf('\nSOME CASES FAIL - see table.\n');
end

%% ======================= 2. Arduino Uno hardware model ===================
newModel(hwMdl);
set_param(hwMdl, 'InitFcn', 'ace_params;', 'PreLoadFcn', 'ace_params;');
set_param(hwMdl, 'HardwareBoard', 'Arduino Uno');
set_param(hwMdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(C.Ts), 'StopTime', 'inf');
try, set_param(hwMdl, 'SimulationMode', 'external'); catch, end

% Identical controller + estimator, copied from the tested model
add_block([simMdl '/Speed Controller'], [hwMdl '/Speed Controller']);
add_block([simMdl '/Speed Estimator'],  [hwMdl '/Speed Estimator']);

load_system('arduinolib'); load_system('arduinosensorlib');
add_block('arduinosensorlib/Encoder', [hwMdl '/Encoder (D2,D3)'], ...
    'InputA', num2str(C.pin_encA), 'InputB', num2str(C.pin_encB), ...
    'SampleTime', num2str(C.Ts));
add_block('arduinolib/PWM', [hwMdl '/PWM (D5)'], 'pinNumber', num2str(C.pin_pwm));
add_block('arduinolib/Digital Output', [hwMdl '/Driver IN1 (D7)'], 'pinNumber', num2str(C.pin_in1));
add_block('arduinolib/Digital Output', [hwMdl '/Driver IN2 (D8)'], 'pinNumber', num2str(C.pin_in2));
add_block('simulink/Sources/Constant', [hwMdl '/HIGH'], 'Value', '1', 'SampleTime', 'C.Ts');
add_block('simulink/Sources/Constant', [hwMdl '/LOW'],  'Value', '0', 'SampleTime', 'C.Ts');
add_block('simulink/Sources/Step', [hwMdl '/Speed set-point (rpm)'], ...
    'Time', 'C.t_step_hw', 'Before', '0', 'After', 'C.ref_rpm', 'SampleTime', 'C.Ts');
add_block('simulink/Signal Routing/Mux', [hwMdl '/Mux'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [hwMdl '/Speed scope (rpm)']);
add_block('simulink/Sinks/Scope', [hwMdl '/PWM duty scope']);

lh = @(a,b) add_line(hwMdl, a, b, 'autorouting', 'smart');
lh('Speed set-point (rpm)/1', 'Speed Controller/1');
lh('Encoder (D2,D3)/1',       'Speed Estimator/1');
lh('Speed Estimator/1',       'Speed Controller/2');
lh('Speed Controller/1',      'PWM (D5)/1');
lh('HIGH/1', 'Driver IN1 (D7)/1');
lh('LOW/1',  'Driver IN2 (D8)/1');
lh('Speed set-point (rpm)/1', 'Mux/1');
lh('Speed Estimator/1',       'Mux/2');
lh('Mux/1', 'Speed scope (rpm)/1');
lh('Speed Controller/1', 'PWM duty scope/1');
tidy(hwMdl); save_system(hwMdl);

% Compile check only (no code generation, nothing sent to the board)
try
    set_param(hwMdl, 'SimulationCommand', 'update');
    fprintf('\n%s: model updates cleanly for Arduino Uno (not deployed).\n', hwMdl);
catch err
    fprintf('\n%s: update diagram reported: %s\n', hwMdl, err.message);
end
save_system(hwMdl);
open_system(hwMdl);

%% ======================= Local functions =================================
function newModel(m)
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
new_system(m); open_system(m);
end

function logBlk(m, name, var)
add_block('simulink/Sinks/To Workspace', [m '/' name], ...
    'VariableName', var, 'SaveFormat', 'Timeseries');
end

function tidy(m)
try, Simulink.BlockDiagram.arrangeSystem(m); catch, end
end

function sub = emptySubsystem(m, name, nIn, nOut)
sub = [m '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', sub);
delete_line(sub, 'In1/1', 'Out1/1');
delete_block([sub '/In1']); delete_block([sub '/Out1']);
for k = 1:nIn,  add_block('simulink/Sources/In1',  sprintf('%s/In%d', sub, k));  end
for k = 1:nOut, add_block('simulink/Sinks/Out1',   sprintf('%s/Out%d', sub, k)); end
end

function addSpeedController(m)
% In1 = set-point [rpm], In2 = measured speed [rpm]; Out1 = PWM duty 0-255
s = emptySubsystem(m, 'Speed Controller', 2, 1);
set_param([s '/In1'], 'Name', 'ref_rpm');  set_param([s '/In2'], 'Name', 'meas_rpm');
set_param([s '/Out1'], 'Name', 'pwm_duty');
add_block('simulink/Math Operations/Sum', [s '/Error'], 'Inputs', '+-');
add_block('simulink/Discrete/Discrete PID Controller', [s '/PI (anti-windup)'], ...
    'Controller', 'PI', 'P', 'C.Kp', 'I', 'C.Ki', 'SampleTime', 'C.Ts', ...
    'IntegratorMethod', 'Forward Euler', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'C.Vs', 'LowerSaturationLimit', '0', ...
    'AntiWindupMode', 'clamping');
add_block('simulink/Math Operations/Gain', [s '/Volts to duty'], 'Gain', '255/C.Vs');
add_block('simulink/Math Operations/Rounding Function', [s '/Round'], 'Operator', 'round');
add_block('simulink/Discontinuities/Saturation', [s '/0-255'], ...
    'UpperLimit', '255', 'LowerLimit', '0');
l = @(a,b) add_line(s, a, b, 'autorouting', 'smart');
l('ref_rpm/1', 'Error/1'); l('meas_rpm/1', 'Error/2');
l('Error/1', 'PI (anti-windup)/1'); l('PI (anti-windup)/1', 'Volts to duty/1');
l('Volts to duty/1', 'Round/1'); l('Round/1', '0-255/1'); l('0-255/1', 'pwm_duty/1');
tidy(s);
end

function addSpeedEstimator(m)
% In1 = encoder count (motor shaft, x4); Out1 = filtered output speed [rpm]
s = emptySubsystem(m, 'Speed Estimator', 1, 1);
set_param([s '/In1'], 'Name', 'enc_count'); set_param([s '/Out1'], 'Name', 'speed_rpm');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/to double'], ...
    'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [s '/Previous count'], ...
    'InitialCondition', '0', 'SampleTime', 'C.Ts');
add_block('simulink/Math Operations/Sum', [s '/Counts per sample'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [s '/Counts to rpm'], ...
    'Gain', '60/(C.CPR_out*C.Ts)');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Low-pass'], ...
    'Numerator', '[1-C.a]', 'Denominator', '[1 -C.a]', 'SampleTime', 'C.Ts');
l = @(a,b) add_line(s, a, b, 'autorouting', 'smart');
l('enc_count/1', 'to double/1'); l('to double/1', 'Counts per sample/1');
l('to double/1', 'Previous count/1'); l('Previous count/1', 'Counts per sample/2');
l('Counts per sample/1', 'Counts to rpm/1'); l('Counts to rpm/1', 'Low-pass/1');
l('Low-pass/1', 'speed_rpm/1');
tidy(s);
end

function addMotorModel(m)
% In1 = PWM duty; Out1 = encoder count (x4, motor shaft), Out2 = true output rpm
s = emptySubsystem(m, 'Motor + Gearbox + Encoder', 1, 2);
set_param([s '/In1'], 'Name', 'pwm_duty');
set_param([s '/Out1'], 'Name', 'enc_count'); set_param([s '/Out2'], 'Name', 'rpm_true');
g = @(n, k, varargin) add_block('simulink/Math Operations/Gain', [s '/' n], 'Gain', k, varargin{:});
g('Duty to volts', 'C.Vs/255');
add_block('simulink/Math Operations/Sum', [s '/Sum_V'], 'Inputs', '+--');
g('inv_L', '1/C.L');
add_block('simulink/Continuous/Integrator', [s '/current'], 'InitialCondition', '0');
g('R', 'C.R', 'Orientation', 'left');
g('Ke', 'C.Ke*C.scale', 'Orientation', 'left');
g('Kt', 'C.Kt*C.scale');
add_block('simulink/Math Operations/Sum', [s '/Sum_T'], 'Inputs', '+-');
add_block('simulink/User-Defined Functions/Fcn', [s '/tanh'], ...
    'Expression', 'tanh(u/10)', 'Orientation', 'left');
g('Friction', 'C.tau_c', 'Orientation', 'left');
g('inv_J', '1/(C.J*C.scale^2)');
add_block('simulink/Continuous/Integrator', [s '/motor speed'], 'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [s '/motor angle'], 'InitialCondition', '0');
g('rad to counts', 'C.cpr_motor/(2*pi)');
add_block('simulink/Math Operations/Rounding Function', [s '/Quantise'], 'Operator', 'floor');
add_block('simulink/Discrete/Zero-Order Hold', [s '/Encoder sample'], 'SampleTime', 'C.Ts');
g('to output rpm', '60/(2*pi*C.N)');
l = @(a,b) add_line(s, a, b, 'autorouting', 'smart');
l('pwm_duty/1', 'Duty to volts/1'); l('Duty to volts/1', 'Sum_V/1');
l('R/1', 'Sum_V/2'); l('Ke/1', 'Sum_V/3');
l('Sum_V/1', 'inv_L/1'); l('inv_L/1', 'current/1');
l('current/1', 'R/1'); l('current/1', 'Kt/1');
l('Kt/1', 'Sum_T/1'); l('Friction/1', 'Sum_T/2');
l('Sum_T/1', 'inv_J/1'); l('inv_J/1', 'motor speed/1');
l('motor speed/1', 'Ke/1'); l('motor speed/1', 'tanh/1'); l('tanh/1', 'Friction/1');
l('motor speed/1', 'motor angle/1'); l('motor speed/1', 'to output rpm/1');
l('motor angle/1', 'rad to counts/1'); l('rad to counts/1', 'Quantise/1');
l('Quantise/1', 'Encoder sample/1'); l('Encoder sample/1', 'enc_count/1');
l('to output rpm/1', 'rpm_true/1');
tidy(s);
end
