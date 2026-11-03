%% build_rl_hw.m
% Builds dc_motor_rl.slx: the trained RL agent tuning the PI speed controller,
% running entirely on the Arduino Mega (Monitor & Tune).
%   Speed Reference -> PI Controller -> PWM (D5) -> motor -> Encoder (D2, D3)
%   -> Speed Measurement; every Ta = 3 s, Performance Features -> RL Policy
%   -> Action to Gains -> new Kp, Ki for the PI controller.
% The RL Policy block is the trained actor network written as plain MATLAB
% code (export_rl_policy.m), so it can be read and runs without the
% Reinforcement Learning Toolbox. Run export_rl_policy.m first.
% A software load (controller output x K_factor from t = 40 s) tests re-tuning.
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));  cd(here);
m = 'dc_motor_rl';
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
load_system('simulink');
new_system(m);
mws = get_param(m, 'ModelWorkspace');
mws.DataSource = 'MATLAB Code';
mws.MATLABCode = strjoin({
'% Parameters (edit here, then press Ctrl+D to update)'
'Ts = 0.01;              % controller / encoder sample time [s]'
'Ta = 3;                 % agent interval = one reference step [s] (as in training)'
'T_exp = 100;            % run length [s]'
'Vs = 12;                % controller output limit (as in training) [V]'
'V_motor = 6;            % maximum voltage reaching the motor [V]'
't_change = 40;  K_factor = 0.6;   % software load: controller output x K_factor from t_change (A: 0.6, B: 2.0)'
'V_supply = 9;           % driver supply voltage [V]: Cytron Maker Drive, 9 V battery'
'CPR = 12*298;           % encoder counts per output-shaft revolution'
'r_low = 70;  r_high = 90;  r_period = 6;   % speed reference [RPM], [s]'
'filt_num = 0.2;  filt_den = [1 -0.8];      % speed low-pass filter'
'logKp = [-2.5 0.3];     % Kp range, log10 (must match training)'
'logKi = [-1.3 0.9];     % Ki range, log10 (must match training)'
}, newline);
mws.reload;
try
    set_param(m, 'HardwareBoard', 'Arduino Mega 2560');
catch err
    warning('Could not set the hardware board: %s', err.message);
end
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'Ts', 'StopTime', 'T_exp');
try, set_param(m, 'SimulationMode', 'external'); catch, end
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');

% reference
s = [m '/Speed Reference'];
add_block('simulink/Ports & Subsystems/Subsystem', s);
delete_line(s, 'In1/1', 'Out1/1');  delete_block([s '/In1']);
add_block('simulink/Sources/Constant', [s '/Low'], 'Value', 'r_low', 'SampleTime', 'Ts');
add_block('simulink/Sources/Pulse Generator', [s '/Square Wave'], 'PulseType', 'Sample based', ...
    'Amplitude', 'r_high - r_low', 'Period', 'round(r_period/Ts)', ...
    'PulseWidth', 'round(r_period/Ts/2)', 'PhaseDelay', 'round(r_period/Ts/2)', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Sum'], 'Inputs', '++');
add_line(s, 'Low/1', 'Sum/1');  add_line(s, 'Square Wave/1', 'Sum/2');  add_line(s, 'Sum/1', 'Out1/1');

% MATLAB Function blocks
C = rl_block_code();
pol = fileread('rl_policy_code.txt');
addMF(m, 'PI Controller', C.pi, {'Ts', 'Vs'}, [0 0 160 110]);
addMF(m, 'Performance Features', C.feat, {'Ts', 'Ta', 'Vs'}, [0 0 180 130]);
addMF(m, 'RL Policy', pol, {}, [0 0 160 100]);
addMF(m, 'Action to Gains', C.gain, {'logKp', 'logKi'}, [0 0 140 80]);

% speed measurement
s = [m '/Speed Measurement'];
add_block('simulink/Ports & Subsystems/Subsystem', s);
delete_line(s, 'In1/1', 'Out1/1');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/To Double'], 'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [s '/Previous Count'], 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Counts per Sample'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [s '/Counts to RPM'], 'Gain', '60/(Ts*CPR)');
add_block('simulink/Discrete/Discrete Filter', [s '/Low-Pass Filter'], ...
    'Numerator', 'filt_num', 'Denominator', 'filt_den', 'SampleTime', 'Ts');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('In1/1', 'To Double/1'); l('To Double/1', 'Counts per Sample/1');
l('To Double/1', 'Previous Count/1'); l('Previous Count/1', 'Counts per Sample/2');
l('Counts per Sample/1', 'Counts to RPM/1'); l('Counts to RPM/1', 'Low-Pass Filter/1');
l('Low-Pass Filter/1', 'Out1/1');

% hardware I/O
add_block('simulink/Sources/Step', [m '/Load Change'], 'Time', 't_change', 'Before', '1', 'After', 'K_factor', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Product', [m '/Software Load']);
add_block('simulink/Discontinuities/Saturation', [m '/Motor Voltage Limit'], 'UpperLimit', 'V_motor', 'LowerLimit', '0');
add_block('simulink/Math Operations/Gain', [m '/Volts to PWM'], 'Gain', '255/V_supply');
add_block('simulink/Math Operations/Rounding Function', [m '/Round'], 'Operator', 'round');
add_block('simulink/Discontinuities/Saturation', [m '/PWM Range 0-255'], 'UpperLimit', '255', 'LowerLimit', '0');
add_block('arduinolib/PWM', [m '/PWM (D5)'], 'pinNumber', '5', 'Position', [0 0 140 70]);
add_block('arduinosensorlib/Encoder', [m '/Encoder (D2, D3)'], 'InputA', '2', 'InputB', '3', 'SampleTime', 'Ts');
add_block('simulink/Discrete/Unit Delay', [m '/Previous u'], 'SampleTime', 'Ts');
add_block('simulink/Discrete/Unit Delay', [m '/Previous Action'], 'InitialCondition', '[0; 0]', 'SampleTime', 'Ts');

L('Speed Reference/1', 'PI Controller/1');
L('Speed Measurement/1', 'PI Controller/2');
L('Action to Gains/1', 'PI Controller/3');
L('Action to Gains/2', 'PI Controller/4');
L('PI Controller/1', 'Software Load/1');  L('Load Change/1', 'Software Load/2');
L('Software Load/1', 'Motor Voltage Limit/1');  L('Motor Voltage Limit/1', 'Volts to PWM/1');
L('Volts to PWM/1', 'Round/1');
L('Round/1', 'PWM Range 0-255/1');
L('PWM Range 0-255/1', 'PWM (D5)/1');
L('Encoder (D2, D3)/1', 'Speed Measurement/1');
L('Speed Reference/1', 'Performance Features/1');
L('Speed Measurement/1', 'Performance Features/2');
L('PI Controller/1', 'Previous u/1');
L('Previous u/1', 'Performance Features/3');
L('Previous Action/1', 'Performance Features/4');
L('Performance Features/1', 'RL Policy/1');
L('RL Policy/1', 'Action to Gains/1');
L('RL Policy/1', 'Previous Action/1');

% scopes and logging
add_block('simulink/Signal Routing/Mux', [m '/Mux Speed'], 'Inputs', '2');
add_block('simulink/Signal Routing/Mux', [m '/Mux Gains'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [m '/Speed']);
add_block('simulink/Sinks/Scope', [m '/PI Gains']);
L('Speed Reference/1', 'Mux Speed/1');  L('Speed Measurement/1', 'Mux Speed/2');
L('Mux Speed/1', 'Speed/1');
L('Action to Gains/1', 'Mux Gains/1');  L('Action to Gains/2', 'Mux Gains/2');
L('Mux Gains/1', 'PI Gains/1');
logs = {'Speed Reference/1', 'r_data'; 'Speed Measurement/1', 'y_data'; ...
        'PI Controller/1', 'u_data'; 'Action to Gains/1', 'Kp_data'; 'Action to Gains/2', 'Ki_data'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [m '/log ' logs{k,2}], ...
        'VariableName', logs{k,2}, 'SaveFormat', 'Array', 'SampleTime', 'Ts');
    L(logs{k,1}, ['log ' logs{k,2} '/1']);
end
try, Simulink.BlockDiagram.arrangeSystem(m); catch, end
b = find_system(m, 'SearchDepth', 1, 'Type', 'Block');
pos = cell2mat(get_param(b(2:end), 'Position'));
note = Simulink.Annotation(m, sprintf(['RL AGENT TUNING A PI SPEED CONTROLLER, on the Arduino Mega (run with Monitor & Tune).\n' ...
    'Every Ta = 3 s the trained policy (RL Policy block: the actor network as MATLAB code) chooses Kp and Ki from the tracking performance.\n' ...
    'Software load: from t_change the controller output is multiplied by K_factor (A: 0.6, B: 2.0) before the 6 V motor limit. Double-click the MATLAB Function blocks to see their code.']));
x0 = min(pos(:,1));  y0 = max(pos(:,4)) + 30;
note.Position = [x0, y0, x0 + 900, y0 + 60];
save_system(m);
try
    set_param(m, 'SimulationCommand', 'update');
    fprintf('%s: built and updates OK for Arduino Mega 2560\n', m);
catch err
    fprintf(2, '%s: built, but update reported: %s\n', m, err.message);
end
clear note b pos C pol L l s

function addMF(m, name, code, params, pos)
blk = [m '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for k = 1:numel(params)
    d = find(ch, '-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end
