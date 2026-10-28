<p align="center"><a href="https://www.ace-lab.co.uk"><b>🌐 www.ace-lab.co.uk</b></a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/practical-exercises">Practical Exercises</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/ace-core">ACE-CORE</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/build-you-own-ace-lab">Get an ACE-Lab</a></p>

# Reinforcement learning PI tuner (C1.6)

| File | What it does |
|---|---|
| `rl_pi_tuner.slx` | Training environment: virtual DC motor, PI controller, and the RL agent choosing Kp and Ki every 3 s |
| `build_rl_env.m` | Rebuilds `rl_pi_tuner.slx` |
| `randomMotor.m` | Gives every training episode a different virtual motor |
| `train_rl_tuner.m` | Trains the TD3 agent and saves `rl_pi_tuner_agent.mat` (about 20 to 30 minutes in parallel on a 16-core PC) |
| `export_rl_policy.m`, `rl_block_code.m` | Write the trained policy as plain MATLAB code for the Arduino |
| `build_rl_hw.m` | Builds `dc_motor_rl.slx`: the trained agent tuning the PI controller on the Arduino Mega (Monitor & Tune) |
| `compare_controllers.m`, `compare_all.m` | Compare fixed PI, LLS + PI, RLS self-tuning PI and RL tuning |
| `verify_requirements.m`, `analyse_hw_run.m` | Check the results against the requirements |

**The trained agent is not included** because the file is over GitHub's 100 MB limit. Run `train_rl_tuner.m` to create `rl_pi_tuner_agent.mat`, then `export_rl_policy.m` and `build_rl_hw.m`.

Needs the Reinforcement Learning Toolbox and Deep Learning Toolbox for training. The exported hardware model doesn't need them.

[← Back to the DC motor code](../README.md)

---
<p align="center"><b>Visit <a href="https://www.ace-lab.co.uk">www.ace-lab.co.uk</a></b> for the full ACE-Lab learning resources.<br>Developed by Dr James E. Pickering · Harper Adams University &amp; Aston University · Project funding received from MathWorks and EUCA</p>
