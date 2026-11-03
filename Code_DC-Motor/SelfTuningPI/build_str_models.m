%% build_str_models.m
% Builds the self-tuning PI speed control models for the DC motor:
%
%   dc_motor_str.slx           hardware (Arduino Mega 2560, Monitor & Tune)
%   dc_motor_str_virtual.slx   the same, with a virtual motor in place of the
%                              Arduino I/O (the virtual motor's gain drops
%                              part-way through, to show the controller re-tune)
%
% Self-tuning regulator (indirect adaptive control):
%   1. RLS Estimator   - estimates a first-order model of the motor online,
%                        y(k) = -a1*y(k-1) + b0*u(k-1) + c
%   2. Self-Tuning PI  - recalculates the PI gains every sample from the
%                        latest model, by pole placement, so that the closed
%                        loop has the required damping ratio and natural frequency
%
% Both are MATLAB Function blocks: double-click a block to see its code.
% The parameters are in each model's workspace (Model Explorer > Model Workspace).
%
% Needs: Simulink Support Package for Arduino Hardware (hardware model).
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));
cd(here);
load_system('simulink');
buildModel('hw');
buildModel('virtual');
fprintf('Built dc_motor_str and dc_motor_str_virtual.\n');

%% ========================================================================
function buildModel(io)
m = 'dc_motor_str';
if strcmp(io, 'virtual'), m = [m '_virtual']; end
if bdIsLoaded(m), close_system(m, 0); end
if exist([m '.slx'], 'file'), delete([m '.slx']); end
new_system(m);

mws = get_param(m, 'ModelWorkspace');
mws.DataSource = 'MATLAB Code';
mws.MATLABCode = paramCode(io);
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

% ---------- reference ----------
addReference(m);

% ---------- controller and estimator ----------
addMatlabFunction(m, 'Self-Tuning PI', stpiCode(), ...
    {'Ts', 'D', 'zeta', 'wn', 'Vs', 'adapt_on', 'Kp0', 'Ki0', 'Kp_max', 'Ki_max'}, [0 0 200 150]);
addMatlabFunction(m, 'RLS Estimator', rlsCode(), ...
    {'Ts', 'D', 'lambda', 'P0', 'TraceMax', 'K0', 'tau0'}, [0 0 200 150]);
add_block('simulink/Discrete/Unit Delay', [m '/Previous Estimate'], ...
    'InitialCondition', '[-exp(-D*Ts/tau0); K0*(1 - exp(-D*Ts/tau0)); 0]', 'SampleTime', 'Ts');
add_block('simulink/Discrete/Discrete Filter', [m '/Input Filter (same as speed filter)'], ...
    'Numerator', 'filt_num', 'Denominator', 'filt_den', 'SampleTime', 'Ts', 'Position', [0 0 90 50]);

% voltage -> PWM duty
add_block('simulink/Math Operations/Gain', [m '/Volts to PWM'], 'Gain', '255/V_supply');
add_block('simulink/Math Operations/Rounding Function', [m '/Round'], 'Operator', 'round');
add_block('simulink/Discontinuities/Saturation', [m '/PWM Range 0-255'], 'UpperLimit', '255', 'LowerLimit', '0');

% ---------- speed measurement and hardware / virtual motor ----------
addSpeedMeasurement(m);
if strcmp(io, 'hw')
    add_block('arduinolib/PWM', [m '/PWM (D5)'], 'pinNumber', '5', 'Position', [0 0 140 70]);
    add_block('arduinosensorlib/Encoder', [m '/Encoder (D2, D3)'], ...
        'InputA', '2', 'InputB', '3', 'SampleTime', 'Ts');
    L('PWM Range 0-255/1', 'PWM (D5)/1');
    L('Encoder (D2, D3)/1', 'Speed Measurement/1');
else
    addVirtualMotor(m);
    L('PWM Range 0-255/1', 'Virtual Motor and Encoder/1');
    L('Virtual Motor and Encoder/1', 'Speed Measurement/1');
end

L('Speed Reference/1', 'Self-Tuning PI/1');
L('Speed Measurement/1', 'Self-Tuning PI/2');
L('Previous Estimate/1', 'Self-Tuning PI/3');
if strcmp(io, 'hw')
    % software load: at t_change the voltage reaching the motor is scaled by
    % K_factor (looks like a change in motor gain), then limited to V_motor
    add_block('simulink/Sources/Step', [m '/Software Load'], 'Time', 't_change', ...
        'Before', '1', 'After', 'K_factor', 'SampleTime', 'Ts');
    add_block('simulink/Math Operations/Product', [m '/Apply Load']);
    add_block('simulink/Discontinuities/Saturation', [m '/Motor Voltage Limit'], ...
        'UpperLimit', 'V_motor', 'LowerLimit', '0');
    L('Self-Tuning PI/1', 'Apply Load/1');
    L('Software Load/1', 'Apply Load/2');
    L('Apply Load/1', 'Motor Voltage Limit/1');
    L('Motor Voltage Limit/1', 'Volts to PWM/1');
else
    L('Self-Tuning PI/1', 'Volts to PWM/1');
end
L('Volts to PWM/1', 'Round/1');
L('Round/1', 'PWM Range 0-255/1');
L('Self-Tuning PI/1', 'Input Filter (same as speed filter)/1');
L('Input Filter (same as speed filter)/1', 'RLS Estimator/1');
L('Speed Measurement/1', 'RLS Estimator/2');
L('RLS Estimator/1', 'Previous Estimate/1');

% ---------- displays, scopes and logging ----------
add_block('simulink/Sinks/Display', [m '/theta = [a1; b0; c]'], 'Position', [0 0 110 70]);
add_block('simulink/Sinks/Display', [m '/K (RPM per V)'], 'Position', [0 0 110 30]);
add_block('simulink/Sinks/Display', [m '/tau (s)'], 'Position', [0 0 110 30]);
add_block('simulink/Sinks/Display', [m '/Kp (V per RPM)'], 'Position', [0 0 110 30]);
add_block('simulink/Sinks/Display', [m '/Ki (V per RPM s)'], 'Position', [0 0 110 30]);
L('RLS Estimator/1', 'theta = [a1; b0; c]/1');
L('RLS Estimator/2', 'K (RPM per V)/1');
L('RLS Estimator/3', 'tau (s)/1');
L('Self-Tuning PI/2', 'Kp (V per RPM)/1');
L('Self-Tuning PI/3', 'Ki (V per RPM s)/1');

add_block('simulink/Signal Routing/Mux', [m '/Mux Speed'], 'Inputs', '2');
add_block('simulink/Signal Routing/Mux', [m '/Mux Gains'], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [m '/Speed and Voltage']);
set_param([m '/Speed and Voltage'], 'NumInputPorts', '2');
add_block('simulink/Sinks/Scope', [m '/PI Gains']);
L('Speed Reference/1', 'Mux Speed/1');
L('Speed Measurement/1', 'Mux Speed/2');
L('Mux Speed/1', 'Speed and Voltage/1');
L('Self-Tuning PI/1', 'Speed and Voltage/2');
L('Self-Tuning PI/2', 'Mux Gains/1');
L('Self-Tuning PI/3', 'Mux Gains/2');
L('Mux Gains/1', 'PI Gains/1');

logs = {'Speed Reference/1', 'r_data'; 'Speed Measurement/1', 'y_data'; ...
        'Self-Tuning PI/1', 'u_data'; 'Self-Tuning PI/2', 'Kp_data'; ...
        'Self-Tuning PI/3', 'Ki_data'; 'RLS Estimator/1', 'theta_est'};
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [m '/log ' logs{k,2}], ...
        'VariableName', logs{k,2}, 'SaveFormat', 'Array', 'SampleTime', '-1');
    L(logs{k,1}, ['log ' logs{k,2} '/1']);
end

nameLine(m, 'Speed Reference', 1, 'r: reference (RPM)');
nameLine(m, 'Speed Measurement', 1, 'y: filtered speed (RPM)');
nameLine(m, 'Self-Tuning PI', 1, 'u: motor voltage (V)');
nameLine(m, 'RLS Estimator', 1, 'theta');
nameLine(m, 'PWM Range 0-255', 1, 'PWM duty');

tidy(m);
addNote(m, io);
save_system(m);
fprintf('%s: built\n', m);
close_system(m, 0);
end

%% ------------------------------------------------------------------------
function c = paramCode(io)
c = {
'% Parameters for this model (edit here, then press Ctrl+D to update)'
'Ts = 0.01;              % sample time [s]'
'T_exp = 80;             % length of the run [s] (100 s on the hardware, set below)'
'Vs = 6;                 % maximum motor voltage (motor rating, controller limit) [V]'
'V_supply = 6;           % driver supply voltage [V] (PWM duty 255 = V_supply)'
'CPR = 12*298;           % encoder counts per output-shaft revolution'
''
'% Speed reference: square wave between r_low and r_high'
'r_low = 20;  r_high = 40;   % [RPM]'
'r_period = 6;               % [s]'
''
'% Speed low-pass filter (also applied to the voltage for estimation)'
'filt_num = 0.2;  filt_den = [1 -0.8];'
''
'% Required closed-loop response (pole placement)'
'zeta = 1.0;             % damping ratio'
'wn = 6;                 % natural frequency [rad/s]'
''
'% Self-tuning PI'
'adapt_on = 1;           % 1 = self-tuning, 0 = fixed gains Kp0, Ki0'
'Kp0 = 0.1;  Ki0 = 0.5;  % gains used until the model is valid [V/RPM, V/(RPM s)]'
'Kp_max = 1;  Ki_max = 10;   % gain limits'
''
'% RLS estimator'
'D = 5;                  % estimate every D samples (Te = D*Ts)'
'lambda = 0.99;          % forgetting factor (0 < lambda <= 1)'
'P0 = 100;               % initial covariance, P = P0*I'
'TraceMax = 1e4;         % trace(P) limit (prevents covariance wind-up)'
'K0 = 10;  tau0 = 0.3;   % initial guess of the motor gain [RPM/V] and time constant [s]'
};
if strcmp(io, 'hw')
    c = [c; {
    ''
    '% ---- Hardware (Cytron Maker Drive, 9 V battery, 6 V motor) ----'
    'T_exp = 100;            % run length [s]'
    'V_supply = 9;           % driver supply voltage [V] (PWM duty 255 = V_supply)'
    'V_motor = 6;            % maximum voltage applied to the motor [V]'
    'r_low = 70;  r_high = 90;   % speed reference [RPM]: above the motor''s sticking region'
    '% Software load: at t_change the voltage reaching the motor is multiplied'
    '% by K_factor (0.6 = scenario A, load added; 2.0 = scenario B, load removed)'
    't_change = 40;  K_factor = 0.6;'
    'Vs = 10;                % controller output limit [V] (= V_motor/0.6, so the'
    '                        % motor can still reach 6 V after the load is added)'}];
end
if strcmp(io, 'virtual')
    c = [c; {
    ''
    '% Virtual motor (illustrative values, used only by the virtual model)'
    'K_true = 16;            % gain [RPM per V]'
    'tau_true = 0.25;        % time constant [s]'
    'V_dead = 0.5;           % dead-band (static friction) [V]'
    't_change = 40;          % time at which the motor gain changes [s]'
    'K_factor = 0.6;         % new gain = K_factor*K_true (e.g. extra load)'}];
end
c = strjoin(c, newline);
end

function addReference(m)
s = emptySubsystem(m, 'Speed Reference', 0, 1);
set_param([s '/Out1'], 'Name', 'r_rpm');
add_block('simulink/Sources/Constant', [s '/Low'], 'Value', 'r_low', 'SampleTime', 'Ts');
add_block('simulink/Sources/Pulse Generator', [s '/Square Wave'], 'PulseType', 'Sample based', ...
    'Amplitude', 'r_high - r_low', 'Period', 'round(r_period/Ts)', ...
    'PulseWidth', 'round(r_period/Ts/2)', 'PhaseDelay', 'round(r_period/Ts/2)', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Sum', [s '/Sum'], 'Inputs', '++');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('Low/1', 'Sum/1'); l('Square Wave/1', 'Sum/2'); l('Sum/1', 'r_rpm/1');
tidy(s);
end

function addSpeedMeasurement(m)
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
s = emptySubsystem(m, 'Virtual Motor and Encoder', 1, 1);
set_param([s '/In1'], 'Name', 'pwm_duty'); set_param([s '/Out1'], 'Name', 'encoder_count');
add_block('simulink/Math Operations/Gain', [s '/PWM to Volts'], 'Gain', 'V_supply/255');
add_block('simulink/Discontinuities/Dead Zone', [s '/Dead-Band'], 'LowerValue', '-V_dead', 'UpperValue', 'V_dead');
add_block('simulink/Sources/Step', [s '/Gain Change'], 'Time', 't_change', 'Before', '1', ...
    'After', 'K_factor', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Product', [s '/Load Effect']);
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Motor (first order, ZOH)'], ...
    'Numerator', 'K_true*(1 - exp(-Ts/tau_true))', 'Denominator', '[1 -exp(-Ts/tau_true)]', ...
    'SampleTime', 'Ts');
add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Count Accumulator'], ...
    'Numerator', '[CPR*Ts/60 0]', 'Denominator', '[1 -1]', 'SampleTime', 'Ts');
add_block('simulink/Math Operations/Rounding Function', [s '/Whole Counts'], 'Operator', 'floor');
add_block('simulink/Signal Attributes/Data Type Conversion', [s '/To int32'], 'OutDataTypeStr', 'int32');
l = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
l('pwm_duty/1', 'PWM to Volts/1'); l('PWM to Volts/1', 'Dead-Band/1');
l('Dead-Band/1', 'Load Effect/1'); l('Gain Change/1', 'Load Effect/2');
l('Load Effect/1', 'Motor (first order, ZOH)/1');
l('Motor (first order, ZOH)/1', 'Count Accumulator/1');
l('Count Accumulator/1', 'Whole Counts/1'); l('Whole Counts/1', 'To int32/1');
l('To int32/1', 'encoder_count/1');
tidy(s);
end

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

function c = stpiCode()
c = strjoin({
'function [u, Kp, Ki] = self_tuning_pi(r, y, theta, Ts, D, zeta, wn, Vs, adapt_on, Kp0, Ki0, Kp_max, Ki_max)'
'% Self-tuning PI speed controller (pole placement from the RLS model)'
'%'
'%   theta = [a1; b0; c] is the latest RLS model, estimated every D samples'
'%   (Te = D*Ts):  y(k) = -a1*y(k-1) + b0*u(k-1) + c'
'%'
'%   1. Convert the model to the control sample time Ts:'
'%        K = b0/(1 + a1),  p = (-a1)^(1/D),  G(z) = K*(1 - p)/(z - p)'
'%   2. PI controller:  C(z) = (q0*z + q1)/(z - 1),'
'%        q0 = Kp + Ki*Ts,  q1 = -Kp'
'%   3. Closed-loop characteristic equation:'
'%        (z - 1)(z - p) + K(1 - p)(q0*z + q1) = z^2 + p1*z + p2'
'%      where the required poles come from zeta and wn:'
'%        p1 = -2*exp(-zeta*wn*Ts)*cos(wn*Ts*sqrt(1 - zeta^2)),'
'%        p2 = exp(-2*zeta*wn*Ts)'
'%      Equating coefficients:'
'%        q0 = (p1 + 1 + p)/(K(1 - p)),   q1 = (p2 - p)/(K(1 - p))'
'%   4. Control law (velocity form, with anti-windup by clamping):'
'%        u(k) = u(k-1) + q0*e(k) + q1*e(k-1),   0 <= u <= Vs'
'%'
'%   The gains are only updated when the model is physically sensible'
'%   (0 < p < 1 and K > 0); otherwise the previous gains are kept.'
'%   adapt_on = 0 uses the fixed gains Kp0 and Ki0 (for comparison).'
''
'persistent uPrev ePrev KpS KiS'
'if isempty(uPrev)'
'    uPrev = 0;  ePrev = 0;  KpS = Kp0;  KiS = Ki0;'
'end'
''
'% ---- 1-3. recalculate the PI gains from the model ----'
'if adapt_on > 0'
'    a1 = theta(1);  b0 = theta(2);'
'    pe = -a1;                                % pole at the estimation rate'
'    if pe > 0 && pe < 1 && abs(1 + a1) > 1e-9'
'        K = b0/(1 + a1);                     % gain [RPM/V]'
'        p = pe^(1/D);                        % pole at the control rate'
'        if K > 0'
'            bs = K*(1 - p);'
'            if zeta < 1'
'                p1 = -2*exp(-zeta*wn*Ts)*cos(wn*Ts*sqrt(1 - zeta^2));'
'            else'
'                p1 = -2*exp(-zeta*wn*Ts)*cosh(wn*Ts*sqrt(zeta^2 - 1));'
'            end'
'            p2 = exp(-2*zeta*wn*Ts);'
'            q0 = (p1 + 1 + p)/bs;'
'            q1 = (p2 - p)/bs;'
'            KpS = min(max(-q1, 0), Kp_max);'
'            KiS = min(max((q0 + q1)/Ts, 0), Ki_max);'
'        end'
'    end'
'else'
'    KpS = Kp0;  KiS = Ki0;'
'end'
''
'% ---- 4. PI control law (velocity form) ----'
'e = r - double(y);'
'u = uPrev + (KpS + KiS*Ts)*e - KpS*ePrev;'
'u = min(max(u, 0), Vs);                     % actuator limits (anti-windup)'
'uPrev = u;'
'ePrev = e;'
'Kp = KpS;'
'Ki = KiS;'
'end'
}, newline);
end

function c = rlsCode()
c = strjoin({
'function [theta, K, tau] = rls_estimator(u, y, Ts, D, lambda, P0, TraceMax, K0, tau0)'
'% Recursive least squares (RLS) estimate of a first-order DC motor model'
'%'
'%   y(k) = -a1*y(k-1) + b0*u(k-1) + c,    theta = [a1; b0; c]'
'%   G(z) = b0/(z + a1),  K = b0/(1 + a1),  tau = -Te/ln(-a1),  Te = D*Ts'
'%   c is a constant term that accounts for the motor dead-band (friction).'
'%'
'%   At each estimation step:'
'%     phi   = [-y(k-1); u(k-1); 1]              regressor'
'%     e     = y(k) - phi''*theta                  prediction error'
'%     L     = P*phi/(lambda + phi''*P*phi)        gain vector'
'%     theta = theta + L*e                        update the estimate'
'%     P     = (P - L*phi''*P)/lambda              update the covariance'
'%'
'%   The estimate starts from an initial guess (K0, tau0). lambda < 1 lets the'
'%   estimate follow changes in the motor; lambda is set to 1 whenever'
'%   trace(P) exceeds TraceMax, to prevent covariance wind-up.'
'%'
'%   u     motor voltage [V] (passed through the same filter as the speed)'
'%   y     filtered motor speed [RPM]'
''
'persistent th P count uSum yPrev'
'if isempty(th)'
'    p0 = exp(-D*Ts/tau0);'
'    th = [-p0; K0*(1 - p0); 0];  % initial guess [a1; b0; c]'
'    P = P0*eye(3);             % covariance matrix'
'    count = D - 1;             % first estimation step at k = 0'
'    uSum = 0;                  % sum of u over the current interval'
'    yPrev = 0;                 % y at the previous estimation step'
'end'
'u = double(u);'
'y = double(y);'
''
'count = count + 1;'
'if count >= D'
'    uAvg = uSum/D;                       % average input over the interval'
'    phi = [-yPrev; uAvg; 1];             % regressor'
'    e = y - phi''*th;                     % prediction error'
'    Pphi = P*phi;'
'    lam = lambda;'
'    if trace(P) > TraceMax               % anti wind-up'
'        lam = 1;'
'    end'
'    L = Pphi/(lam + phi''*Pphi);          % gain vector'
'    th = th + L*e;                       % update the estimate'
'    P = (P - L*Pphi'')/lam;               % update the covariance'
'    P = (P + P'')/2;                      % keep P symmetric'
'    yPrev = y;'
'    uSum = u;                            % start the next interval'
'    count = 0;'
'else'
'    uSum = uSum + u;'
'end'
''
'theta = th;'
'K = 0;  tau = 0;'
'if abs(1 + th(1)) > 1e-9'
'    K = th(2)/(1 + th(1));'
'end'
'if -th(1) > 0 && -th(1) < 1'
'    tau = -D*Ts/log(-th(1));'
'end'
'end'
}, newline);
end

function addNote(m, io)
if strcmp(io, 'hw'), t0 = 'Hardware: run with Monitor & Tune.';
else, t0 = 'Virtual motor: press Run (the motor gain drops at t_change to show re-tuning).'; end
txt = sprintf(['SELF-TUNING PI SPEED CONTROL (RLS + pole placement).  %s\n' ...
    'RLS Estimator: y(k) = -a1 y(k-1) + b0 u(k-1) + c, estimated online.  ' ...
    'Self-Tuning PI: recalculates Kp and Ki every sample so the closed loop has damping ratio zeta and natural frequency wn.\n' ...
    'Double-click a block to see its code. Parameters: Model Explorer > Model Workspace (set adapt_on = 0 for fixed gains). ' ...
    'Run plot_str_results.m afterwards.'], t0);
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
