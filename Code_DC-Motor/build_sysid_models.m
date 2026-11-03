%% build_sysid_models.m
% Builds the open-loop system identification models for the DC motor:
%
%   dc_motor_lls.slx           hardware (Arduino Mega 2560): open-loop excitation,
%                              linear least squares (LLS) estimator
%   dc_motor_rls.slx           hardware (Arduino Mega 2560): open-loop excitation,
%                              recursive least squares (RLS) estimator
%   dc_motor_lls_virtual.slx   as dc_motor_lls, with a virtual motor in place
%   dc_motor_rls_virtual.slx   as dc_motor_rls, with a virtual motor in place
%
% The estimators are MATLAB Function blocks: double-click a block to see and
% edit its code. The model parameters (sample time, excitation, estimator
% settings) are in each model's workspace (Modeling > Model Explorer >
% Model Workspace), so each .slx file is self-contained.
%
% Needs: Simulink Support Package for Arduino Hardware (hardware models).
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));
cd(here);
load_system('simulink');
for io = {'hw', 'virtual'}
    for est = {'lls', 'rls'}
        buildModel(est{1}, io{1});
    end
end
fprintf('Built dc_motor_lls, dc_motor_rls, dc_motor_lls_virtual and dc_motor_rls_virtual.\n');

%% ========================================================================
function buildModel(est, io)
m = ['dc_motor_' est];
if strcmp(io, 'virtual'), m = [m '_virtual']; end
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
new_system(m);

% ---------- parameters (model workspace) ----------
mws = get_param(m, 'ModelWorkspace');
mws.DataSource = 'MATLAB Code';
mws.MATLABCode = paramCode(est, io);
mws.reload;

if strcmp(io, 'hw')
    try
        set_param(m, 'HardwareBoard', 'Arduino Mega 2560');
    catch err
        warning('Could not set the hardware board: %s', err.message);
    end
end
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'Ts', 'StopTime', 'T_exp');
if strcmp(io, 'hw')
    try, set_param(m, 'SimulationMode', 'external'); catch, end
end

L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');

% ---------- open-loop excitation ----------
addExcitation(m);

% ---------- speed measurement ----------
addSpeedMeasurement(m);

% ---------- hardware I/O or virtual motor ----------
if strcmp(io, 'hw')
    add_block('arduinolib/PWM', [m '/PWM (D5)'], 'pinNumber', '5', 'Position', [0 0 140 70]);
    add_block('arduinosensorlib/Encoder', [m '/Encoder (D2, D3)'], ...
        'InputA', '2', 'InputB', '3', 'SampleTime', 'Ts');
    L('Open-Loop Excitation/1', 'PWM (D5)/1');
    L('Encoder (D2, D3)/1', 'Speed Measurement/1');
else
    addVirtualMotor(m);
    L('Open-Loop Excitation/1', 'Virtual Motor and Encoder/1');
    L('Virtual Motor and Encoder/1', 'Speed Measurement/1');
end
% ---------- estimator ----------
if strcmp(est, 'lls')
    name = 'LLS Estimator';
    addMatlabFunction(m, name, llsCode(), {'Ts', 'D', 't_start', 'NMin'});
else
    name = 'RLS Estimator';
    addMatlabFunction(m, name, rlsCode(), {'Ts', 'D', 't_start', 'lambda', 'P0', 'TraceMax'});
end
add_block('simulink/Discrete/Discrete Filter', [m '/Input Filter (same as speed filter)'], ...
    'Numerator', 'filt_num', 'Denominator', 'filt_den', 'SampleTime', 'Ts', 'Position', [0 0 90 50]);
L('Open-Loop Excitation/2', 'Input Filter (same as speed filter)/1');
L('Input Filter (same as speed filter)/1', [name '/1']);
L('Speed Measurement/1', [name '/2']);

% ---------- displays, scopes and logging ----------
add_block('simulink/Sinks/Display', [m '/theta = [a1; b0; c]'], 'Position', [0 0 110 70]);
add_block('simulink/Sinks/Display', [m '/K (RPM per V)'], 'Position', [0 0 110 30]);
add_block('simulink/Sinks/Display', [m '/tau (s)'], 'Position', [0 0 110 30]);
L([name '/1'], 'theta = [a1; b0; c]/1');
L([name '/2'], 'K (RPM per V)/1');
L([name '/3'], 'tau (s)/1');

add_block('simulink/Signal Routing/Mux', [m '/Mux'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [m '/Voltage and Speed']);
set_param([m '/Voltage and Speed'], 'NumInputPorts', '2');
L('Open-Loop Excitation/2', 'Voltage and Speed/1');
L('Speed Measurement/1', 'Mux/1');
L('Speed Measurement/2', 'Mux/2');
L('Mux/1', 'Voltage and Speed/2');
add_block('simulink/Sinks/Scope', [m '/Estimates']);
set_param([m '/Estimates'], 'NumInputPorts', '2');
L([name '/2'], 'Estimates/1');
L([name '/3'], 'Estimates/2');

logs = {'Open-Loop Excitation/2', 'u_data'; 'Speed Measurement/1', 'y_data'; ...
        [name '/1'], 'theta_est'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [m '/log ' logs{k,2}], ...
        'VariableName', logs{k,2}, 'SaveFormat', 'Array', 'SampleTime', '-1');
    L(logs{k,1}, ['log ' logs{k,2} '/1']);
end

% signal names
nameLine(m, 'Open-Loop Excitation', 1, 'PWM duty (0-255)');
nameLine(m, 'Open-Loop Excitation', 2, 'u: motor voltage (V)');
nameLine(m, 'Speed Measurement', 1, 'y: filtered speed (RPM)');
nameLine(m, 'Speed Measurement', 2, 'raw speed (RPM)');
nameLine(m, name, 1, 'theta');

tidy(m);
addNote(m, est, io);
save_system(m);
fprintf('%s: built\n', m);
close_system(m, 0);
end

%% ------------------------------------------------------------------------
function c = paramCode(est, io)
c = {
'% Parameters for this model (edit here, then press Ctrl+D to update)'
'Ts = 0.01;              % sample time [s]'
'T_exp = 60;             % length of the experiment [s]'
'Vs = 6;                 % maximum motor voltage (motor rating) [V]'
'CPR = 12*298;           % encoder counts per output-shaft revolution'
''
'% Open-loop excitation: offset + two square waves (PWM duty, 0-255)'
'% (duty varies between pwm_offset - pwm_amp1 - pwm_amp2 and'
'%  pwm_offset + pwm_amp1 + pwm_amp2; keep the minimum above the dead-band)'
'pwm_offset = 140;       % mean duty'
'pwm_amp1 = 50;  per1 = 3.0;   % square wave 1: amplitude, period [s]'
'pwm_amp2 = 30;  per2 = 7.4;   % square wave 2: amplitude, period [s]'
'V_supply = 6;            % driver supply voltage [V] (hardware value set below)'
't_start = 1;            % excitation starts at [s]'
''
'% Speed low-pass filter (also applied to the voltage for estimation)'
'filt_num = 0.2;  filt_den = [1 -0.8];'
''
'% Estimator'
'D = 5;                  % estimate every D samples (Te = D*Ts)'
};
if strcmp(est, 'lls')
    c = [c; {'NMin = 20;              % estimation steps before the first estimate'}];
else
    c = [c; {
    'lambda = 0.995;         % forgetting factor (0 < lambda <= 1)'
    'P0 = 1000;              % initial covariance, P = P0*I'
    'TraceMax = 1e4;         % trace(P) limit (prevents covariance wind-up)'}];
end
if strcmp(io, 'hw')
    c = [c; {
    ''
    '% Hardware: Cytron Maker Drive from a 9 V battery, 6 V motor'
    'V_supply = 9;           % driver supply voltage [V] (PWM duty 255 = V_supply)'
    'pwm_offset = 150;  pwm_amp1 = 15;  pwm_amp2 = 5;   % duty 130-170, i.e. 4.6-6 V (above the motor''s sticking region)'}];
end
if strcmp(io, 'virtual')
    c = [c; {
    ''
    '% Virtual motor (illustrative values, used only by the virtual model)'
    'K_true = 16;            % gain [RPM per V]'
    'tau_true = 0.25;        % time constant [s]'
    'V_dead = 0.5;           % dead-band (static friction) [V]'}];
end
c = strjoin(c, newline);
end

function addExcitation(m)
s = emptySubsystem(m, 'Open-Loop Excitation', 0, 2);
set_param([s '/Out1'], 'Name', 'pwm_duty');
set_param([s '/Out2'], 'Name', 'u_volts');
add_block('simulink/Sources/Constant', [s '/Offset'], 'Value', 'pwm_offset', 'SampleTime', 'Ts');
add_block('simulink/Sources/Pulse Generator', [s '/Square Wave 1'], 'PulseType', 'Sample based', ...
    'Amplitude', '2*pwm_amp1', 'Period', 'round(per1/Ts)', 'PulseWidth', 'round(per1/Ts/2)', ...
    'PhaseDelay', '0', 'SampleTime', 'Ts');
add_block('simulink/Sources/Pulse Generator', [s '/Square Wave 2'], 'PulseType', 'Sample based', ...
    'Amplitude', '2*pwm_amp2', 'Period', 'round(per2/Ts)', 'PulseWidth', 'round(per2/Ts/2)', ...
    'PhaseDelay', '0', 'SampleTime', 'Ts');
add_block('simulink/Sources/Constant', [s '/Centre'], 'Value', '-(pwm_amp1 + pwm_amp2)', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Sum'], 'Inputs', '++++');
add_block('simulink/Sources/Step', [s '/Start'], 'Time', 't_start', 'Before', '0', ...
    'After', '1', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Product', [s '/Gate']);
add_block('simulink/Math Operations/Rounding Function', [s '/Round'], 'Operator', 'round');
add_block('simulink/Discontinuities/Saturation', [s '/PWM Range 0-255'], 'UpperLimit', 'floor(255*Vs/V_supply)', 'LowerLimit', '0');
add_block('simulink/Math Operations/Gain', [s '/PWM to Volts'], 'Gain', 'V_supply/255');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('Offset/1', 'Sum/1'); l('Square Wave 1/1', 'Sum/2'); l('Square Wave 2/1', 'Sum/3');
l('Centre/1', 'Sum/4');
l('Sum/1', 'Gate/1'); l('Start/1', 'Gate/2'); l('Gate/1', 'Round/1');
l('Round/1', 'PWM Range 0-255/1'); l('PWM Range 0-255/1', 'pwm_duty/1');
l('PWM Range 0-255/1', 'PWM to Volts/1'); l('PWM to Volts/1', 'u_volts/1');
tidy(s);
end

function addSpeedMeasurement(m)
% In1 = encoder count; Out1 = filtered speed (RPM), Out2 = raw speed (RPM)
s = emptySubsystem(m, 'Speed Measurement', 1, 2);
set_param([s '/In1'], 'Name', 'encoder_count');
set_param([s '/Out1'], 'Name', 'speed_filtered');
set_param([s '/Out2'], 'Name', 'speed_raw');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/To Double'], 'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [s '/Previous Count'], 'InitialCondition', '0', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Counts per Sample'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [s '/Counts to RPM'], 'Gain', '60/(Ts*CPR)');
add_block('simulink/Discrete/Discrete Filter', [s '/Low-Pass Filter'], ...
    'Numerator', 'filt_num', 'Denominator', 'filt_den', 'SampleTime', 'Ts');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('encoder_count/1', 'To Double/1'); l('To Double/1', 'Counts per Sample/1');
l('To Double/1', 'Previous Count/1'); l('Previous Count/1', 'Counts per Sample/2');
l('Counts per Sample/1', 'Counts to RPM/1'); l('Counts to RPM/1', 'Low-Pass Filter/1');
l('Low-Pass Filter/1', 'speed_filtered/1'); l('Counts to RPM/1', 'speed_raw/1');
tidy(s);
end

function addVirtualMotor(m)
% In1 = PWM duty; Out1 = encoder count (int32), as from the Arduino Encoder block
s = emptySubsystem(m, 'Virtual Motor and Encoder', 1, 1);
set_param([s '/In1'], 'Name', 'pwm_duty'); set_param([s '/Out1'], 'Name', 'encoder_count');
add_block('simulink/Math Operations/Gain', [s '/PWM to Volts'], 'Gain', 'V_supply/255');
add_block('simulink/Discontinuities/Dead Zone', [s '/Dead-Band'], 'LowerValue', '-V_dead', 'UpperValue', 'V_dead');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Motor (first order, ZOH)'], ...
    'Numerator', 'K_true*(1 - exp(-Ts/tau_true))', 'Denominator', '[1 -exp(-Ts/tau_true)]', ...
    'SampleTime', 'Ts');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Count Accumulator'], ...
    'Numerator', '[CPR*Ts/60 0]', 'Denominator', '[1 -1]', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Rounding Function', [s '/Whole Counts'], 'Operator', 'floor');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/To int32'], 'OutDataTypeStr', 'int32');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('pwm_duty/1', 'PWM to Volts/1'); l('PWM to Volts/1', 'Dead-Band/1');
l('Dead-Band/1', 'Motor (first order, ZOH)/1'); l('Motor (first order, ZOH)/1', 'Count Accumulator/1');
l('Count Accumulator/1', 'Whole Counts/1'); l('Whole Counts/1', 'To int32/1');
l('To int32/1', 'encoder_count/1');
tidy(s);
end

function addMatlabFunction(m, name, code, params)
blk = [m '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', [0 0 200 150]);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for k = 1:numel(params)
    d = find(ch, '-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

function c = llsCode()
c = strjoin({
'function [theta, K, tau] = lls_estimator(u, y, Ts, D, t_start, NMin)'
'% Linear least squares (LLS) estimate of a first-order DC motor model'
'%'
'%   y(k) = -a1*y(k-1) + b0*u(k-1) + c,    theta = [a1; b0; c]'
'%'
'%   G(z) = b0/(z + a1),  K = b0/(1 + a1),  tau = -Te/ln(-a1)'
'%   c is a constant term that accounts for the motor dead-band (friction).'
'%'
'%   theta = (Phi''*Phi) \ (Phi''*Y), where each row of Phi is'
'%   phi'' = [-y(k-1), u(k-1), 1]. Phi''*Phi and Phi''*Y are accumulated one'
'%   step at a time, so theta is the batch LLS solution using ALL of the data'
'%   collected so far (every sample weighted equally).'
'%'
'%   u     motor voltage [V] (passed through the same filter as the speed)'
'%   y     filtered motor speed [RPM]'
'%   Ts    sample time [s];  D: estimate every D samples, Te = D*Ts'
'%   t_start  time the excitation starts [s] (earlier data are not used)'
'%   NMin  number of estimation steps before the first estimate'
''
'persistent PhiTPhi PhiTY N count uSum yPrev th k'
'if isempty(PhiTPhi)'
'    PhiTPhi = zeros(3, 3);     % running sum of phi*phi'''
'    PhiTY = zeros(3, 1);       % running sum of phi*y'
'    N = 0;                     % number of estimation steps'
'    count = D - 1;             % first estimation step at k = 0'
'    uSum = 0;                  % sum of u over the current interval'
'    yPrev = 0;                 % y at the previous estimation step'
'    th = zeros(3, 1);          % estimate [a1; b0; c]'
'    k = 0;                     % sample number'
'end'
'u = double(u);'
'y = double(y);'
'k = k + 1;'
''
'count = count + 1;'
'if count >= D'
'    uAvg = uSum/D;                       % average input over the interval'
'    phi = [-yPrev; uAvg; 1];             % regressor'
'    if (k - D)*Ts >= t_start             % only use data after the start'
'        PhiTPhi = PhiTPhi + phi*phi'';    % accumulate Phi''*Phi'
'        PhiTY = PhiTY + phi*y;           % accumulate Phi''*Y'
'        N = N + 1;'
'    end'
'    if N >= NMin && rcond(PhiTPhi) > 1e-12'
'        th = PhiTPhi \ PhiTY;            % solve the normal equations'
'    end'
'    yPrev = y;'
'    uSum = u;                            % start the next interval'
'    count = 0;'
'else'
'    uSum = uSum + u;'
'end'
''
'theta = th;'
'[K, tau] = firstOrderParams(th, D*Ts);'
'end'
''
'function [K, tau] = firstOrderParams(th, Te)'
'% Gain and time constant of G(z) = b0/(z + a1), sampled at Te'
'K = 0;'
'tau = 0;'
'if abs(1 + th(1)) > 1e-9'
'    K = th(2)/(1 + th(1));'
'end'
'p = -th(1);                              % pole of the discrete-time model'
'if p > 0 && p < 1'
'    tau = -Te/log(p);'
'end'
'end'
}, newline);
end

function c = rlsCode()
c = strjoin({
'function [theta, K, tau] = rls_estimator(u, y, Ts, D, t_start, lambda, P0, TraceMax)'
'% Recursive least squares (RLS) estimate of a first-order DC motor model'
'%'
'%   y(k) = -a1*y(k-1) + b0*u(k-1) + c,    theta = [a1; b0; c]'
'%'
'%   G(z) = b0/(z + a1),  K = b0/(1 + a1),  tau = -Te/ln(-a1)'
'%   c is a constant term that accounts for the motor dead-band (friction).'
'%'
'%   At each estimation step:'
'%     phi   = [-y(k-1); u(k-1); 1]              regressor'
'%     e     = y(k) - phi''*theta                  prediction error'
'%     L     = P*phi/(lambda + phi''*P*phi)        gain vector'
'%     theta = theta + L*e                        update the estimate'
'%     P     = (P - L*phi''*P)/lambda              update the covariance'
'%'
'%   No matrix inversion is needed. The forgetting factor lambda gives older'
'%   data less weight (memory of about 1/(1 - lambda) estimation steps);'
'%   lambda = 1 gives the same result as batch LLS. When the signals do not'
'%   change, dividing by lambda makes P grow without limit (wind-up), so'
'%   lambda is set to 1 whenever trace(P) exceeds TraceMax.'
'%'
'%   u     motor voltage [V] (passed through the same filter as the speed)'
'%   y     filtered motor speed [RPM]'
'%   Ts    sample time [s];  D: estimate every D samples, Te = D*Ts'
'%   t_start  time the excitation starts [s] (earlier data are not used)'
''
'persistent th P count uSum yPrev k'
'if isempty(th)'
'    th = zeros(3, 1);          % estimate [a1; b0; c]'
'    P = P0*eye(3);             % covariance matrix'
'    count = D - 1;             % first estimation step at k = 0'
'    uSum = 0;                  % sum of u over the current interval'
'    yPrev = 0;                 % y at the previous estimation step'
'    k = 0;                     % sample number'
'end'
'u = double(u);'
'y = double(y);'
'k = k + 1;'
''
'count = count + 1;'
'if count >= D'
'    uAvg = uSum/D;                       % average input over the interval'
'    phi = [-yPrev; uAvg; 1];             % regressor'
'    if (k - D)*Ts >= t_start             % only use data after the start'
'        e = y - phi''*th;                 % prediction error'
'        Pphi = P*phi;'
'        lam = lambda;'
'        if trace(P) > TraceMax           % anti wind-up'
'            lam = 1;'
'        end'
'        L = Pphi/(lam + phi''*Pphi);      % gain vector'
'        th = th + L*e;                   % update the estimate'
'        P = (P - L*Pphi'')/lam;           % update the covariance'
'        P = (P + P'')/2;                  % keep P symmetric'
'    end'
'    yPrev = y;'
'    uSum = u;                            % start the next interval'
'    count = 0;'
'else'
'    uSum = uSum + u;'
'end'
''
'theta = th;'
'[K, tau] = firstOrderParams(th, D*Ts);'
'end'
''
'function [K, tau] = firstOrderParams(th, Te)'
'% Gain and time constant of G(z) = b0/(z + a1), sampled at Te'
'K = 0;'
'tau = 0;'
'if abs(1 + th(1)) > 1e-9'
'    K = th(2)/(1 + th(1));'
'end'
'p = -th(1);                              % pole of the discrete-time model'
'if p > 0 && p < 1'
'    tau = -Te/log(p);'
'end'
'end'
}, newline);
end

function addNote(m, est, io)
if strcmp(est, 'lls')
    t1 = 'LINEAR LEAST SQUARES (open loop).';
    t2 = 'theta = (Phi''Phi)^-1 Phi''Y is recomputed from ALL the data so far. Run lls_offline.m afterwards for the batch estimate.';
else
    t1 = 'RECURSIVE LEAST SQUARES with forgetting factor (open loop).';
    t2 = 'theta and P are updated every estimation step, with no matrix inversion. Run plot_estimates.m afterwards.';
end
if strcmp(io, 'hw'), t0 = 'Hardware: run with Monitor & Tune.'; else, t0 = 'Virtual motor: press Run.'; end
txt = sprintf(['%s  %s\nModel: y(k) = -a1 y(k-1) + b0 u(k-1) + c,  G(z) = b0/(z + a1),  ' ...
    'K = b0/(1 + a1),  tau = -Te/ln(-a1).\n%s\n' ...
    'u = motor voltage, passed through the same low-pass filter as the speed; y = filtered speed (RPM). ' ...
    'Double-click the estimator to see its code. Parameters: Model Explorer > Model Workspace.'], t1, t0, t2);
b = find_system(m, 'SearchDepth', 1, 'Type', 'Block');
pos = cell2mat(get_param(b(2:end), 'Position'));
a = Simulink.Annotation(m, txt);
x0 = min(pos(:,1));  y0 = max(pos(:,4)) + 30;
a.Position = [x0, y0, x0 + 900, y0 + 70];
end

function nameLine(m, blk, port, txt)
ph = get_param([m '/' blk], 'PortHandles');
set_param(get_param(ph.Outport(port), 'Line'), 'Name', txt);
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
