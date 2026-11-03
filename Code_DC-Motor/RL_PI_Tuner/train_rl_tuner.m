%% train_rl_tuner.m
% Trains a TD3 reinforcement learning agent to tune the PI speed controller
% in rl_pi_tuner.slx. Every training episode uses a different virtual motor
% (random gain, time constant and dead-band, and a random gain change part-way
% through), so the agent learns to choose good gains for whatever motor it is
% connected to, the same job as the RLS self-tuning PI.
%
% Training takes some time (about 20-30 minutes in parallel on a 16-core PC); the Reinforcement Learning Training Monitor
% shows the progress. The trained agent is saved to rl_pi_tuner_agent.mat.
% ACE-Lab, ACE-CORE.

here = fileparts(mfilename('fullpath'));  cd(here);
mdl = 'rl_pi_tuner';
if ~exist([mdl '.slx'], 'file'), build_rl_env; end
load_system(mdl);

Ta = 3;  T_ep = 90;
rng(0);

%% Observation and action specifications
obsInfo = rlNumericSpec([10 1]);
obsInfo.Name = 'observations';
obsInfo.Description = 'RMS error, overshoot, mean voltage, mean speed, speed change per volt, previous values, current action';
actInfo = rlNumericSpec([2 1], 'LowerLimit', -1, 'UpperLimit', 1);
actInfo.Name = 'normalised PI gains';

%% Environment, with a randomised virtual motor for every episode
env = rlSimulinkEnv(mdl, [mdl '/RL Agent'], obsInfo, actInfo);
env.ResetFcn = @(in) randomMotor(in, mdl);   % see randomMotor.m

%% TD3 agent (actor and critic networks created automatically)
initOpts = rlAgentInitializationOptions('NumHiddenUnit', 64);
agentOpts = rlTD3AgentOptions('SampleTime', Ta, 'DiscountFactor', 0.7, ...
    'MiniBatchSize', 128, 'ExperienceBufferLength', 1e5);
agentOpts.ExplorationModel.StandardDeviation = 0.4;
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4;
agentOpts.ExplorationModel.StandardDeviationMin = 0.05;
agent = rlTD3Agent(obsInfo, actInfo, initOpts, agentOpts);

% Small actor (policy) network, 10-16-16-2, so that the trained policy fits
% on the Arduino Mega (about 480 parameters, under 2 KB); the critics, which
% are only needed for training, keep 64 hidden units.
anet = dlnetwork([featureInputLayer(10)
                  fullyConnectedLayer(16)
                  reluLayer
                  fullyConnectedLayer(16)
                  reluLayer
                  fullyConnectedLayer(2)
                  tanhLayer]);
agent = setActor(agent, rlContinuousDeterministicActor(anet, obsInfo, actInfo));

%% Train
trainOpts = rlTrainingOptions('MaxEpisodes', 1000, 'MaxStepsPerEpisode', ceil(T_ep/Ta), ...
    'ScoreAveragingWindowLength', 20, 'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'EpisodeCount', 'StopTrainingValue', 1000, 'Verbose', false);
% Run the episodes in parallel if the Parallel Computing Toolbox is available
% (each episode simulates 60 s of the motor, so this saves a lot of time)
if license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'))
    trainOpts.UseParallel = true;
    trainOpts.ParallelizationOptions.Mode = 'async';
end
stats = train(agent, env, trainOpts);
save('rl_pi_tuner_agent.mat', 'agent', 'stats');
export_rl_policy;                    % write the policy as MATLAB code for the hardware model
fprintf('Training finished: average reward over the last 20 episodes = %.2f\n', ...
    stats.AverageReward(end));
