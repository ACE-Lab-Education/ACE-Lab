%% build_rl_env.m
% Builds rl_pi_tuner.slx: a reinforcement learning (RL) agent that tunes a PI
% speed controller for the DC motor (the virtual motor used in the
% self-tuning PI models).
%
%   Speed Reference  - square wave, 20 <-> 40 RPM, period 6 s
%   PI Controller    - velocity-form PI with anti-windup (MATLAB Function);
%                      Kp and Ki are inputs, set by the agent
%   Performance Features (MATLAB Function) - over each half-period of the
%                      reference (Ta = 3 s, one speed step) it calculates the
%                      observations and the reward for the agent:
%                        RMS tracking error, overshoot, mean voltage, mean
%                        speed, the speed change per volt (which reveals a
%                        change in the motor), the previous interval's
%                        values and the current (normalised) gains
%                      reward = -(RMS error/5)^2 - (overshoot)/2 - 0.15*(chatter)
%   RL Agent         - TD3 agent; every Ta it chooses the PI gains.
%                      Action a in [-1, 1]^2 is mapped to Kp and Ki on a
%                      logarithmic scale by the 'Action to Gains' block.
%   Virtual Motor    - first-order motor with dead-band and encoder; its gain,
%                      time constant and dead-band are randomised for every
%                      training episode (and the gain changes part-way through),
%                      so the agent learns to re-tune for different motors.
%
% Run train_rl_tuner.m after this script, then compare_controllers.m.
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));
cd(here);
m = 'rl_pi_tuner';
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
load_system('simulink');  load_system('rllib');
new_system(m);

mws = get_param(m, 'ModelWorkspace');
mws.DataSource = 'MATLAB Code';
mws.MATLABCode = strjoin({
'% Parameters (training randomises the virtual motor values in the reset function)'
'Ts = 0.01;              % controller / encoder sample time [s]'
'Ta = 3;                 % agent sample time = one reference step [s]'
'T_ep = 90;              % length of a training episode [s]'
'Vs = 12;                % controller output limit [V] (room for the x0.6 load, as on the hardware)'
'V_motor = 6;            % maximum voltage reaching the motor [V]'
'CPR = 12*298;           % encoder counts per output-shaft revolution'
'r_low = 70;  r_high = 90;  r_period = 6;   % speed reference [RPM], [s] (as on the hardware)'
'filt_num = 0.2;  filt_den = [1 -0.8];      % speed low-pass filter'
'logKp = [-2.5 0.3];     % Kp range, log10 (0.003 to 2 V/RPM)'
'logKi = [-1.3 0.9];     % Ki range, log10 (0.05 to 8 V/(RPM s))'
''
'% Virtual motor (nominal values; randomised during training)'
'K_true = 16;  tau_true = 0.054;  V_dead = 0.2;   % identified from the real motor (hw_id_noload2)'
't_change = 40;  K_factor = 0.6;            % gain change part-way through'
}, newline);
mws.reload;

set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'Ts', 'StopTime', 'T_ep', 'EnableMultiTasking', 'off');
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');

% ---------- reference ----------
s = emptySubsystem(m, 'Speed Reference', 0, 1);
add_block('simulink/Sources/Constant', [s '/Low'], 'Value', 'r_low', 'SampleTime', 'Ts');
add_block('simulink/Sources/Pulse Generator', [s '/Square Wave'], 'PulseType', 'Sample based', ...
    'Amplitude', 'r_high - r_low', 'Period', 'round(r_period/Ts)', ...
    'PulseWidth', 'round(r_period/Ts/2)', 'PhaseDelay', 'round(r_period/Ts/2)', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Sum'], 'Inputs', '++');
add_line(s, 'Low/1', 'Sum/1'); add_line(s, 'Square Wave/1', 'Sum/2'); add_line(s, 'Sum/1', 'Out1/1');
tidy(s);

% ---------- MATLAB Function blocks ----------
C = rl_block_code();
addMatlabFunction(m, 'PI Controller', C.pi, {'Ts', 'Vs'}, [0 0 160 110]);
addMatlabFunction(m, 'Performance Features', C.feat, {'Ts', 'Ta', 'Vs'}, [0 0 180 130]);
addMatlabFunction(m, 'Action to Gains', C.gain, {'logKp', 'logKi'}, [0 0 140 80]);

% ---------- RL agent ----------
add_block('rllib/RL Agent', [m '/RL Agent'], 'Position', [0 0 120 90]);
set_param([m '/RL Agent'], 'Agent', 'agent');
add_block('simulink/Sources/Constant', [m '/isdone'], 'Value', '0', 'SampleTime', 'Ta');

% ---------- virtual motor and speed measurement ----------
addVirtualMotor(m);
addSpeedMeasurement(m);
add_block('simulink/Math Operations/Gain', [m '/Volts to PWM'], 'Gain', '255/Vs');
add_block('simulink/Math Operations/Rounding Function', [m '/Round'], 'Operator', 'round');
add_block('simulink/Discontinuities/Saturation', [m '/PWM Range 0-255'], 'UpperLimit', '255', 'LowerLimit', '0');
add_block('simulink/Discrete/Unit Delay', [m '/Previous u'], 'SampleTime', 'Ts');
add_block('simulink/Discrete/Unit Delay', [m '/Previous Action'], 'InitialCondition', '[0; 0]', 'SampleTime', 'Ts');

L('Speed Reference/1', 'PI Controller/1');
L('Speed Measurement/1', 'PI Controller/2');
L('Action to Gains/1', 'PI Controller/3');
L('Action to Gains/2', 'PI Controller/4');
L('PI Controller/1', 'Volts to PWM/1');
L('Volts to PWM/1', 'Round/1');
L('Round/1', 'PWM Range 0-255/1');
L('PWM Range 0-255/1', 'Virtual Motor and Encoder/1');
L('Virtual Motor and Encoder/1', 'Speed Measurement/1');

L('Speed Reference/1', 'Performance Features/1');
L('Speed Measurement/1', 'Performance Features/2');
L('PI Controller/1', 'Previous u/1');
L('Previous u/1', 'Performance Features/3');
L('RL Agent/1', 'Previous Action/1');
L('Previous Action/1', 'Performance Features/4');
L('Performance Features/1', 'RL Agent/1');
L('Performance Features/2', 'RL Agent/2');
L('isdone/1', 'RL Agent/3');
L('RL Agent/1', 'Action to Gains/1');

% ---------- scopes and logging ----------
add_block('simulink/Signal Routing/Mux', [m '/Mux Speed'], 'Inputs', '2');
add_block('simulink/Signal Routing/Mux', [m '/Mux Gains'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [m '/Speed']);
add_block('simulink/Sinks/Scope', [m '/PI Gains']);
L('Speed Reference/1', 'Mux Speed/1');  L('Speed Measurement/1', 'Mux Speed/2');
L('Mux Speed/1', 'Speed/1');
L('Action to Gains/1', 'Mux Gains/1');  L('Action to Gains/2', 'Mux Gains/2');
L('Mux Gains/1', 'PI Gains/1');
logs = {'Speed Reference/1', 'r_data'; 'Speed Measurement/1', 'y_data'; ...
        'PI Controller/1', 'u_data'; 'Action to Gains/1', 'Kp_data'; ...
        'Action to Gains/2', 'Ki_data'; 'Performance Features/2', 'reward_data'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [m '/log ' logs{k,2}], ...
        'VariableName', logs{k,2}, 'SaveFormat', 'Array', 'SampleTime', 'Ts');
    L(logs{k,1}, ['log ' logs{k,2} '/1']);
end
ph = get_param([m '/Performance Features'], 'PortHandles');
set_param(get_param(ph.Outport(1), 'Line'), 'Name', 'observations');
set_param(get_param(ph.Outport(2), 'Line'), 'Name', 'reward');
ph = get_param([m '/RL Agent'], 'PortHandles');
set_param(get_param(ph.Outport(1), 'Line'), 'Name', 'action');

tidy(m);
b = find_system(m, 'SearchDepth', 1, 'Type', 'Block');
pos = cell2mat(get_param(b(2:end), 'Position'));
note = Simulink.Annotation(m, sprintf(['RL AGENT TUNING A PI SPEED CONTROLLER.  Every Ta = 3 s (one reference step) the TD3 agent ' ...
    'observes the tracking performance and how the motor responds (speed change per volt), and chooses Kp and Ki.\nReward = -(RMS error/5)^2 - (overshoot)/2 - 0.15 (PWM chatter).  ' ...
    'Train with train_rl_tuner.m, then compare with the fixed PI and the RLS self-tuning PI using compare_controllers.m.\n' ...
    'Double-click the MATLAB Function blocks to see their code. Parameters: Model Explorer > Model Workspace.']));
x0 = min(pos(:,1));  y0 = max(pos(:,4)) + 30;
note.Position = [x0, y0, x0 + 900, y0 + 70];
save_system(m);
fprintf('%s: built\n', m);
clear note b pos ph s L C   % (an annotation handle in the workspace cannot be sent to parallel workers)

%% ========================================================================



function addMatlabFunction(m, name, code, params, pos)
blk = [m '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for k = 1:numel(params)
    d = find(ch, '-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

function addSpeedMeasurement(m)
s = emptySubsystem(m, 'Speed Measurement', 1, 1);
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/To Double'], 'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [s '/Previous Count'], 'InitialCondition', '0', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Counts per Sample'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [s '/Counts to RPM'], 'Gain', '60/(Ts*CPR)');
add_block('simulink/Discrete/Discrete Filter', [s '/Low-Pass Filter'], ...
    'Numerator', 'filt_num', 'Denominator', 'filt_den', 'SampleTime', 'Ts');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('In1/1', 'To Double/1'); l('To Double/1', 'Counts per Sample/1');
l('To Double/1', 'Previous Count/1'); l('Previous Count/1', 'Counts per Sample/2');
l('Counts per Sample/1', 'Counts to RPM/1'); l('Counts to RPM/1', 'Low-Pass Filter/1');
l('Low-Pass Filter/1', 'Out1/1');
tidy(s);
end

function addVirtualMotor(m)
s = emptySubsystem(m, 'Virtual Motor and Encoder', 1, 1);
add_block('simulink/Math Operations/Gain', [s '/PWM to Volts'], 'Gain', 'Vs/255');
add_block('simulink/Discontinuities/Dead Zone', [s '/Dead-Band'], 'LowerValue', '-V_dead', 'UpperValue', 'V_dead');
add_block('simulink/Sources/Step', [s '/Gain Change'], 'Time', 't_change', 'Before', '1', ...
    'After', 'K_factor', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Product', [s '/Load Effect']);
add_block('simulink/Discontinuities/Saturation', [s '/Motor Voltage Limit'], 'UpperLimit', 'V_motor', 'LowerLimit', '0');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Motor (first order, ZOH)'], ...
    'Numerator', 'K_true*(1 - exp(-Ts/tau_true))', 'Denominator', '[1 -exp(-Ts/tau_true)]', ...
    'SampleTime', 'Ts');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Count Accumulator'], ...
    'Numerator', '[CPR*Ts/60 0]', 'Denominator', '[1 -1]', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Rounding Function', [s '/Whole Counts'], 'Operator', 'floor');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('In1/1', 'PWM to Volts/1'); l('PWM to Volts/1', 'Load Effect/1'); l('Gain Change/1', 'Load Effect/2');
l('Load Effect/1', 'Motor Voltage Limit/1'); l('Motor Voltage Limit/1', 'Dead-Band/1');
l('Dead-Band/1', 'Motor (first order, ZOH)/1');
l('Motor (first order, ZOH)/1', 'Count Accumulator/1');
l('Count Accumulator/1', 'Whole Counts/1'); l('Whole Counts/1', 'Out1/1');
tidy(s);
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
