%% run_virtual_pipeline.m
% Dry run of the whole workflow on the virtual rig (motor model in place of
% the Arduino I/O). Same models, same scripts, same signal names as the
% hardware run - only the I/O differs.

clear out id_source cl_source
if exist('ace_tuned_gains.mat', 'file'), delete('ace_tuned_gains.mat'); end
ace_params;

% STEP 1  open-loop excitation + logging
out = sim('ace_id_virtual');
% STEP 2  least-squares identification
id_source = 'virtual rig';
ace_step2_identify;
% STEP 3  gain optimisation on the identified model
ace_step3_tune;
% STEP 4  closed loop with the tuned gains (InitFcn reloads ace_params)
out = sim('ace_cl_virtual');
cl_source = 'virtual rig';
ace_step4_verify;
