function C = rl_block_code()
% rl_block_code  Code of the MATLAB Function blocks shared by the RL models
% (rl_pi_tuner.slx for training and dc_motor_rl.slx for the hardware).
C.pi = piCode();
C.feat = featCode();
C.gain = gainCode();
end

function c = piCode()
c = strjoin({
'function u = pi_controller(r, y, Kp, Ki, Ts, Vs)'
'% Velocity-form PI controller with anti-windup (clamping)'
'%   u(k) = u(k-1) + (Kp + Ki*Ts)*e(k) - Kp*e(k-1),   0 <= u <= Vs'
'% The velocity form changes smoothly when the gains change.'
'persistent uPrev ePrev'
'if isempty(uPrev)'
'    uPrev = 0;  ePrev = 0;'
'end'
'e = r - double(y);'
'u = uPrev + (Kp + Ki*Ts)*e - Kp*ePrev;'
'u = min(max(u, 0), Vs);'
'uPrev = u;'
'ePrev = e;'
'end'
}, newline);
end

function c = featCode()
c = strjoin({
'function [obs, reward] = performance_features(r, y, u, a, Ts, Ta, Vs)'
'% Observations and reward for the RL agent, over each agent interval Ta'
'% (one reference step).  At the end of each interval:'
'%'
'%   Khat = change in mean speed / change in mean voltage between this'
'%          interval and the last one.  This is how much the speed changes'
'%          per volt (an incremental gain, unaffected by the dead-band), so'
'%          it tells the agent when the motor has changed.'
'%   obs = [RMS error/10; overshoot/10; mean u/Vs; mean y/50; Khat/30;'
'%          previous Khat/30; previous RMS error/10; previous overshoot/10;'
'%          a(1); a(2)]'
'%   reward = -(RMS error/5)^2 - overshoot/2 - 0.15*chatter'
'%   chatter = mean |change in PWM duty| per sample (counts), which penalises'
'%   high gains that amplify encoder noise.'
'% where a is the agent''s current action (the normalised PI gains).'
'% The values are held until the end of the next interval.'
'persistent n cnt sE2 osMax sU sY sDU uLast rLast dir obsH rewH Khat uMeanPrev yMeanPrev rmsPrev osPrev'
'if isempty(n)'
'    n = round(Ta/Ts);  cnt = 0;  sE2 = 0;  osMax = 0;  sU = 0;  sY = 0;  sDU = 0;  uLast = u;'
'    rLast = r;  dir = 0;  obsH = zeros(10, 1);  rewH = 0;'
'    Khat = 15;  uMeanPrev = 0;  yMeanPrev = 0;  rmsPrev = 0;  osPrev = 0;'
'end'
'y = double(y);'
'if cnt == 0                          % start of an interval: step direction'
'    dir = sign(r - rLast);'
'    rLast = r;'
'end'
'e = r - y;'
'sE2 = sE2 + e^2;'
'osMax = max(osMax, dir*(y - r));     % overshoot past the new reference'
'sU = sU + u;'
'sDU = sDU + abs(u - uLast);  uLast = u;'
'sY = sY + y;'
'cnt = cnt + 1;'
'if cnt >= n                          % end of the interval'
'    rmsE = sqrt(sE2/n);'
'    uMean = sU/n;  yMean = sY/n;'
'    KhatPrev = Khat;'
'    if abs(uMean - uMeanPrev) > 0.1   % incremental gain estimate'
'        Khat = min(max((yMean - yMeanPrev)/(uMean - uMeanPrev), 0), 90);'
'    end'
'    obsH = [rmsE/10; osMax/10; uMean/Vs; yMean/50; Khat/30; KhatPrev/30; ...'
'            rmsPrev/10; osPrev/10; a(1); a(2)];'
'    chatter = sDU/n*255/Vs;          % mean |change in PWM duty| [counts]'
'    rewH = -(rmsE/5)^2 - osMax/2 - 0.15*chatter;'
'    uMeanPrev = uMean;  yMeanPrev = yMean;  rmsPrev = rmsE;  osPrev = osMax;'
'    cnt = 0;  sE2 = 0;  osMax = 0;  sU = 0;  sY = 0;  sDU = 0;'
'end'
'obs = obsH;'
'reward = rewH;'
'end'
}, newline);
end

function c = gainCode()
c = strjoin({
'function [Kp, Ki] = action_to_gains(a, logKp, logKi)'
'% Map the agent''s action a in [-1, 1] to the PI gains, on a log scale'
'a = min(max(double(a), -1), 1);'
'Kp = 10^(logKp(1) + (a(1) + 1)/2*(logKp(2) - logKp(1)));'
'Ki = 10^(logKi(1) + (a(2) + 1)/2*(logKi(2) - logKi(1)));'
'end'
}, newline);
end
