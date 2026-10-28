%% ace_params.m
% Parameters for the ACE-Lab actuator speed-control models
%   ace_speed_sim.slx      - closed loop with a motor model (desktop test)
%   ace_speed_arduino.slx  - same controller wired to Arduino Uno I/O
% Loaded automatically by each model's InitFcn.

C = struct();

%% ---- Requirement ----
C.ref_rpm  = 30;        % desired gearbox output speed      [rpm]
C.t_step   = 0.5;       % time the set-point is applied (sim)[s]
C.t_step_hw = 2;        % ... on hardware (time to open scope)[s]
C.tr_max   = 1.0;       % rise time requirement (10-90 %)    [s]

%% ---- Hardware ----
C.N        = 297.92;    % gearbox ratio (Pololu 298:1)        [-]
C.cpr_motor = 12;       % encoder counts per MOTOR rev (x4 quadrature)
C.CPR_out  = C.cpr_motor*C.N;   % counts per OUTPUT rev (= 3575)
C.Vs       = 6;         % motor driver supply voltage         [V]  <-- check
C.pin_encA = 2;         % encoder channel A (Uno interrupt pin)
C.pin_encB = 3;         % encoder channel B (Uno interrupt pin)
C.pin_pwm  = 5;         % PWM to driver enable / speed input
C.pin_in1  = 7;         % driver direction input 1 (held HIGH)
C.pin_in2  = 8;         % driver direction input 2 (held LOW)

%% ---- Motor model: Pololu 298:1 Micro Metal Gearmotor HP 6V (datasheet) ----
w_nl  = 100*C.N*2*pi/60;          % no-load motor speed @ 6 V  [rad/s]
I_nl  = 0.10;  I_st = 1.6;         % no-load / stall current    [A]
C.R   = 6/I_st;                    % armature resistance        [Ohm]
C.Ke  = (6 - I_nl*C.R)/w_nl;       % back-EMF constant          [V*s/rad]
C.Kt  = C.Ke;                      % torque constant            [N*m/A]
C.L   = 1.5e-3;                    % armature inductance (typ.) [H]
C.tm  = 0.025;                     % mechanical time constant   [s]
C.J   = C.tm*C.Kt*C.Ke/C.R;        % rotor + reflected inertia  [kg*m^2]
C.tau_c = C.Kt*I_nl;               % Coulomb friction (motor)   [N*m]
C.scale = 1.0;                     % +/- spread for robustness test

%% ---- Controller ----
C.Ts  = 0.02;           % control / encoder sample time (50 Hz) [s]
C.fc  = 8;              % speed-measurement low-pass cut-off    [Hz]
C.a   = exp(-2*pi*C.fc*C.Ts);      % filter pole
C.Kp  = 0.03;           % proportional gain  [V per rpm]
C.Ki  = 0.40;           % integral gain      [V per rpm*s]  -> zero SS error

%% ---- System-identification experiment (open loop, on the board) ----
C.id_T      = 30;       % experiment length                  [s]
C.id_start  = 1;        % excitation starts                  [s]
C.id_offset = 1.5;      % base voltage (keeps motor turning)  [V]
C.id_amp1   = 1.0;  C.id_per1 = 100;   % square wave 1: 1.0 V, 2.0 s period
C.id_amp2   = 0.6;  C.id_per2 = 270;   % square wave 2: 0.6 V, 5.4 s period
C.cl_T      = 8;        % closed-loop test length            [s]

%% ---- Use optimiser-tuned gains if they exist (ace_step3_tune) ----
C.gain_source = 'hand-tuned default';
if exist('ace_tuned_gains.mat', 'file')
    G_ = load('ace_tuned_gains.mat');
    C.Kp = G_.Kp;  C.Ki = G_.Ki;  C.gain_source = G_.source;
    clear G_
end

clear w_nl I_nl I_st
