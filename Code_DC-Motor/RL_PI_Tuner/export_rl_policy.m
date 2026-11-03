%% export_rl_policy.m
% Exports the trained actor (policy) network in rl_pi_tuner_agent.mat as
% plain MATLAB code, so that it can run on the Arduino Mega inside a MATLAB
% Function block (dc_motor_rl.slx) and be read like any other code.
%   a = tanh(W3*relu(W2*relu(W1*obs + b1) + b2) + b3)
% Writes rl_policy_code.txt and checks it against the agent.
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));  cd(here);
S = load('rl_pi_tuner_agent.mat', 'agent');
net = getModel(getActor(S.agent));
Lr = net.Learnables;
val = @(layer, p) double(extractdata(Lr.Value{strcmp(Lr.Layer, layer) & strcmp(Lr.Parameter, p)}));
fcs = unique(Lr.Layer, 'stable');
assert(numel(fcs) == 3, 'Expected an actor with three fully connected layers.');
W = cell(3, 1);  b = cell(3, 1);
for k = 1:3
    W{k} = val(fcs{k}, 'Weights');  b{k} = val(fcs{k}, 'Bias');
end

fmt = @(M) mat2str(M, 6);
code = strjoin({
'function a = rl_policy(obs)'
'% Trained TD3 policy (actor network 10-16-16-2), exported from'
'% rl_pi_tuner_agent.mat by export_rl_policy.m.'
'%   obs = [RMS error/10; overshoot/10; mean u/Vs; mean y/50; Khat/30;'
'%          previous Khat/30; previous RMS error/10; previous overshoot/10;'
'%          a(1); a(2)]'
'%   a   = normalised PI gains in [-1, 1] (see Action to Gains)'
'%   a = tanh(W3*relu(W2*relu(W1*obs + b1) + b2) + b3)'
'% The network is only evaluated when new observations arrive (every Ta).'
'persistent obsLast aLast'
'if isempty(obsLast)'
'    obsLast = 1e9*ones(10, 1);  aLast = zeros(2, 1);'
'end'
'obs = double(obs(:));'
'if any(obs ~= obsLast)'
['    W1 = ' fmt(W{1}) ';']
['    b1 = ' fmt(b{1}) ';']
['    W2 = ' fmt(W{2}) ';']
['    b2 = ' fmt(b{2}) ';']
['    W3 = ' fmt(W{3}) ';']
['    b3 = ' fmt(b{3}) ';']
'    h1 = max(W1*obs + b1, 0);            % hidden layer 1 (ReLU)'
'    h2 = max(W2*h1 + b2, 0);             % hidden layer 2 (ReLU)'
'    aLast = tanh(W3*h2 + b3);            % output layer (tanh)'
'    obsLast = obs;'
'end'
'a = aLast;'
'end'
}, newline);
fid = fopen('rl_policy_code.txt', 'w');  fprintf(fid, '%s', code);  fclose(fid);

% check the exported code against the agent
maxErr = 0;
for i = 1:20
    o = [rand(4,1); 0.3 + rand(4,1); 2*rand(2,1) - 1];
    aA = getAction(S.agent, {o});  aA = double(aA{1});
    h1 = max(W{1}*o + b{1}, 0);  h2 = max(W{2}*h1 + b{2}, 0);  aC = tanh(W{3}*h2 + b{3});
    maxErr = max(maxErr, max(abs(aA - aC)));
end
fprintf('Policy exported (%d parameters); max difference from the agent = %.2g\n', ...
    sum(cellfun(@numel, [W; b])), maxErr);
