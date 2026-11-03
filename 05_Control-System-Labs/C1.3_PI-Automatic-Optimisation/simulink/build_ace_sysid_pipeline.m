%% build_ace_sysid_pipeline.m
% Builds the four models for the identify -> tune -> verify workflow:
%
%   ace_id_arduino.slx   STEP 1 (hardware): open-loop voltage excitation on
%                        the Uno, logs V_applied and rpm_raw (30 s).
%   ace_cl_arduino.slx   STEP 5 (hardware): closed-loop PI with the tuned
%                        gains, 30 rpm step at t = 2 s, logs speed (8 s).
%   ace_id_virtual.slx   Same as ace_id_arduino, motor model instead of I/O
%   ace_cl_virtual.slx   Same as ace_cl_arduino, motor model instead of I/O
%
% Nothing is deployed. Hardware run order (see README_ACE_pipeline.txt):
%   ace_id_arduino (Monitor & Tune) -> ace_step2_identify -> ace_step3_tune
%   -> ace_cl_arduino (Monitor & Tune) -> ace_step4_verify
%
% Needs: Simulink Support Package for Arduino Hardware, ace_params.m.

ace_params;
load_system('arduinolib'); load_system('arduinosensorlib');
buildModel('ace_id_arduino', 'hw',      'id');
buildModel('ace_cl_arduino', 'hw',      'cl');
buildModel('ace_id_virtual', 'virtual', 'id');
buildModel('ace_cl_virtual', 'virtual', 'cl');
fprintf('Built ace_id_arduino, ace_cl_arduino, ace_id_virtual, ace_cl_virtual.\n');

%% ========================================================================
function buildModel(m, io, kind)
C = evalin('base', 'C');
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
new_system(m);
set_param(m, 'PreLoadFcn', 'ace_params;', 'InitFcn', 'ace_params;');
if strcmp(kind, 'id'), T = 'C.id_T'; else, T = 'C.cl_T'; end
if strcmp(io, 'hw')
    set_param(m, 'HardwareBoard', 'Arduino Uno');
    set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', num2str(C.Ts), 'StopTime', T);
    try, set_param(m, 'SimulationMode', 'external'); catch, end
else
    set_param(m, 'SolverType', 'Variable-step', 'Solver', 'ode23tb', ...
        'MaxStep', '1e-3', 'StopTime', T);
end
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');

% ---------- speed measurement ----------
addSpeedEstimator(m);

% ---------- drive signal: 'Drive/1' = PWM duty 0-255 ----------
if strcmp(kind, 'id')
    s = emptySubsystem(m, 'Excitation (open loop)', 0, 2);
    set_param([s '/Out1'], 'Name', 'pwm_duty'); set_param([s '/Out2'], 'Name', 'V_applied');
    add_block('simulink/Sources/Constant', [s '/Offset'], 'Value', 'C.id_offset', 'SampleTime', 'C.Ts');
    add_block('simulink/Sources/Pulse Generator', [s '/Square 1'], 'PulseType', 'Sample based', ...
        'Amplitude', 'C.id_amp1', 'Period', 'C.id_per1', 'PulseWidth', 'C.id_per1/2', ...
        'PhaseDelay', '0', 'SampleTime', 'C.Ts');
    add_block('simulink/Sources/Pulse Generator', [s '/Square 2'], 'PulseType', 'Sample based', ...
        'Amplitude', 'C.id_amp2', 'Period', 'C.id_per2', 'PulseWidth', 'C.id_per2/2', ...
        'PhaseDelay', '0', 'SampleTime', 'C.Ts');
    add_block('simulink/Math Operations/Sum', [s '/Sum'], 'Inputs', '+++');
    add_block('simulink/Sources/Step', [s '/Start'], 'Time', 'C.id_start', 'Before', '0', ...
        'After', '1', 'SampleTime', 'C.Ts');
    add_block('simulink/Math Operations/Product', [s '/Gate']);
    add_block('simulink/Math Operations/Gain', [s '/Volts to duty'], 'Gain', '255/C.Vs');
    add_block('simulink/Math Operations/Rounding Function', [s '/Round'], 'Operator', 'round');
    add_block('simulink/Discontinuities/Saturation', [s '/0-255'], 'UpperLimit', '255', 'LowerLimit', '0');
    add_block('simulink/Math Operations/Gain', [s '/Duty to volts'], 'Gain', 'C.Vs/255');
    l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
    l('Offset/1', 'Sum/1'); l('Square 1/1', 'Sum/2'); l('Square 2/1', 'Sum/3');
    l('Sum/1', 'Gate/1'); l('Start/1', 'Gate/2'); l('Gate/1', 'Volts to duty/1');
    l('Volts to duty/1', 'Round/1'); l('Round/1', '0-255/1'); l('0-255/1', 'pwm_duty/1');
    l('0-255/1', 'Duty to volts/1'); l('Duty to volts/1', 'V_applied/1');
    tidy(s);
    drive = 'Excitation (open loop)/1';
else
    addSpeedController(m);
    add_block('simulink/Sources/Step', [m '/Speed set-point (rpm)'], 'Time', 'C.t_step_hw', ...
        'Before', '0', 'After', 'C.ref_rpm', 'SampleTime', 'C.Ts');
    L('Speed set-point (rpm)/1', 'Speed Controller/1');
    L('Speed Estimator/1', 'Speed Controller/2');
    drive = 'Speed Controller/1';
end

% ---------- plant / hardware I/O ----------
if strcmp(io, 'hw')
    add_block('arduinosensorlib/Encoder', [m '/Encoder (D2,D3)'], ...
        'InputA', num2str(C.pin_encA), 'InputB', num2str(C.pin_encB), 'SampleTime', num2str(C.Ts));
    add_block('arduinolib/PWM', [m '/PWM (D5)'], 'pinNumber', num2str(C.pin_pwm));
    add_block('arduinolib/Digital Output', [m '/Driver IN1 (D7)'], 'pinNumber', num2str(C.pin_in1));
    add_block('arduinolib/Digital Output', [m '/Driver IN2 (D8)'], 'pinNumber', num2str(C.pin_in2));
    add_block('simulink/Sources/Constant', [m '/HIGH'], 'Value', '1', 'SampleTime', 'C.Ts');
    add_block('simulink/Sources/Constant', [m '/LOW'],  'Value', '0', 'SampleTime', 'C.Ts');
    L('HIGH/1', 'Driver IN1 (D7)/1'); L('LOW/1', 'Driver IN2 (D8)/1');
    L(drive, 'PWM (D5)/1');
    L('Encoder (D2,D3)/1', 'Speed Estimator/1');
else
    addMotorModel(m);
    L(drive, 'Motor + Gearbox + Encoder/1');
    L('Motor + Gearbox + Encoder/1', 'Speed Estimator/1');
end

% ---------- logging (To Workspace + signal logging for Data Inspector) ----------
if strcmp(kind, 'id')
    logs = {'Excitation (open loop)/2', 'V_applied'; drive, 'pwm_duty'; ...
            'Speed Estimator/2', 'rpm_raw'; 'Speed Estimator/1', 'rpm_filt'};
else
    logs = {'Speed set-point (rpm)/1', 'ref_rpm'; drive, 'pwm_duty'; ...
            'Speed Estimator/2', 'rpm_raw'; 'Speed Estimator/1', 'rpm_filt'};
end
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [m '/log_' logs{k,2}], ...
        'VariableName', logs{k,2}, 'SaveFormat', 'Timeseries');
    L(logs{k,1}, ['log_' logs{k,2} '/1']);
    p = strsplit(logs{k,1}, '/');
    ph = get_param([m '/' strjoin(p(1:end-1), '/')], 'PortHandles');
    lh = get_param(ph.Outport(str2double(p{end})), 'Line');
    set_param(lh, 'Name', logs{k,2});
    set_param(ph.Outport(str2double(p{end})), 'DataLogging', 'on');
end
add_block('simulink/Signal Routing/Mux', [m '/Mux'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [m '/Speed scope (rpm)']);
add_block('simulink/Sinks/Scope', [m '/PWM duty scope']);
if strcmp(kind, 'id'), L('Excitation (open loop)/2', 'Mux/1'); else, L('Speed set-point (rpm)/1', 'Mux/1'); end
L('Speed Estimator/1', 'Mux/2');
L('Mux/1', 'Speed scope (rpm)/1');
L(drive, 'PWM duty scope/1');
tidy(m);
save_system(m);
if strcmp(io, 'hw')
    try
        set_param(m, 'SimulationCommand', 'update');
        fprintf('%s: updates cleanly for Arduino Uno (not deployed).\n', m);
    catch err
        fprintf('%s: update diagram reported: %s\n', m, err.message);
    end
    save_system(m);
end
close_system(m);
end

function tidy(m)
try, Simulink.BlockDiagram.arrangeSystem(m); catch, end
end

function sub = emptySubsystem(m, name, nIn, nOut)
sub = [m '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', sub);
delete_line(sub, 'In1/1', 'Out1/1');
delete_block([sub '/In1']); delete_block([sub '/Out1']);
for k = 1:nIn,  add_block('simulink/Sources/In1', sprintf('%s/In%d', sub, k));  end
for k = 1:nOut, add_block('simulink/Sinks/Out1',  sprintf('%s/Out%d', sub, k)); end
end

function addSpeedController(m)
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
add_block('simulink/Discontinuities/Saturation', [s '/0-255'], 'UpperLimit', '255', 'LowerLimit', '0');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('ref_rpm/1', 'Error/1'); l('meas_rpm/1', 'Error/2');
l('Error/1', 'PI (anti-windup)/1'); l('PI (anti-windup)/1', 'Volts to duty/1');
l('Volts to duty/1', 'Round/1'); l('Round/1', '0-255/1'); l('0-255/1', 'pwm_duty/1');
tidy(s);
end

function addSpeedEstimator(m)
% In1 = encoder count; Out1 = filtered rpm, Out2 = raw rpm (count difference)
s = emptySubsystem(m, 'Speed Estimator', 1, 2);
set_param([s '/In1'], 'Name', 'enc_count');
set_param([s '/Out1'], 'Name', 'rpm_filt'); set_param([s '/Out2'], 'Name', 'rpm_raw');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/to double'], 'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [s '/Previous count'], 'InitialCondition', '0', 'SampleTime', 'C.Ts');
add_block('simulink/Math Operations/Sum', [s '/Counts per sample'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [s '/Counts to rpm'], 'Gain', '60/(C.CPR_out*C.Ts)');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Low-pass'], ...
    'Numerator', '[1-C.a]', 'Denominator', '[1 -C.a]', 'SampleTime', 'C.Ts');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('enc_count/1', 'to double/1'); l('to double/1', 'Counts per sample/1');
l('to double/1', 'Previous count/1'); l('Previous count/1', 'Counts per sample/2');
l('Counts per sample/1', 'Counts to rpm/1'); l('Counts to rpm/1', 'Low-pass/1');
l('Low-pass/1', 'rpm_filt/1'); l('Counts to rpm/1', 'rpm_raw/1');
tidy(s);
end

function addMotorModel(m)
% Virtual rig: In1 = PWM duty; Out1 = encoder count (x4, motor shaft)
s = emptySubsystem(m, 'Motor + Gearbox + Encoder', 1, 1);
set_param([s '/In1'], 'Name', 'pwm_duty'); set_param([s '/Out1'], 'Name', 'enc_count');
g = @(n, k, varargin) add_block('simulink/Math Operations/Gain', [s '/' n], 'Gain', k, varargin{:});
g('Duty to volts', 'C.Vs/255');
add_block('simulink/Math Operations/Sum', [s '/Sum_V'], 'Inputs', '+--');
g('inv_L', '1/C.L');
add_block('simulink/Continuous/Integrator', [s '/current'], 'InitialCondition', '0');
g('R', 'C.R', 'Orientation', 'left');
g('Ke', 'C.Ke*C.scale', 'Orientation', 'left');
g('Kt', 'C.Kt*C.scale');
add_block('simulink/Math Operations/Sum', [s '/Sum_T'], 'Inputs', '+-');
add_block('simulink/User-Defined Functions/Fcn', [s '/tanh'], 'Expression', 'tanh(u/10)', 'Orientation', 'left');
g('Friction', 'C.tau_c', 'Orientation', 'left');
g('inv_J', '1/(C.J*C.scale^2)');
add_block('simulink/Continuous/Integrator', [s '/motor speed'], 'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [s '/motor angle'], 'InitialCondition', '0');
g('rad to counts', 'C.cpr_motor/(2*pi)');
add_block('simulink/Math Operations/Rounding Function', [s '/Quantise'], 'Operator', 'floor');
add_block('simulink/Discrete/Zero-Order Hold', [s '/Encoder sample'], 'SampleTime', 'C.Ts');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('pwm_duty/1', 'Duty to volts/1'); l('Duty to volts/1', 'Sum_V/1');
l('R/1', 'Sum_V/2'); l('Ke/1', 'Sum_V/3');
l('Sum_V/1', 'inv_L/1'); l('inv_L/1', 'current/1');
l('current/1', 'R/1'); l('current/1', 'Kt/1');
l('Kt/1', 'Sum_T/1'); l('Friction/1', 'Sum_T/2');
l('Sum_T/1', 'inv_J/1'); l('inv_J/1', 'motor speed/1');
l('motor speed/1', 'Ke/1'); l('motor speed/1', 'tanh/1'); l('tanh/1', 'Friction/1');
l('motor speed/1', 'motor angle/1');
l('motor angle/1', 'rad to counts/1'); l('rad to counts/1', 'Quantise/1');
l('Quantise/1', 'Encoder sample/1'); l('Encoder sample/1', 'enc_count/1');
tidy(s);
end
