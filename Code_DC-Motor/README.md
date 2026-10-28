<p align="center"><a href="https://www.ace-lab.co.uk"><b>🌐 www.ace-lab.co.uk</b></a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/practical-exercises">Practical Exercises</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/ace-core">ACE-CORE</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/build-you-own-ace-lab">Get an ACE-Lab</a></p>

# DC motor code: system identification and PI control

MATLAB and Simulink files for the ACE-Lab **Actuate** DC motor exercises. Keep this folder together: some scripts use files in the neighbouring folders.

| Folder / file | Used in | What it does |
|---|---|---|
| `dc_motor_lls.slx` | [A1.4](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.4_LLS-Parameter-Estimation) | Hardware model: open-loop excitation and linear least squares (LLS) estimator |
| `dc_motor_lls_virtual.slx` | A1.4 | The same with a virtual motor, so you can try it without hardware |
| `dc_motor_rls.slx` | [A1.5](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.5_RLS-Online-Estimation) | Hardware model: recursive least squares (RLS) online estimator |
| `dc_motor_rls_virtual.slx` | A1.5 | The same with a virtual motor |
| `lls_offline.m` | A1.4, A1.5 | Batch LLS estimate from the logged data (run after the model) |
| `plot_estimates.m`, `test_sysid.m` | A1.4, A1.5 | Plot the estimates; quick test on the virtual models |
| `build_sysid_models.m` | A1.4, A1.5 | Rebuilds the four models above from scratch |
| `hw_id_noload2.mat` | C1.4 | Example open-loop identification data from a real ACE-Lab motor |
| [`SelfTuningPI/`](SelfTuningPI) | [C1.4](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.4_PI-Pole-Placement), [C1.5](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.5_Self-Tuning-PI) | Fixed PI, LLS + pole-placement PI, and RLS self-tuning PI |
| [`RL_PI_Tuner/`](RL_PI_Tuner) | [C1.6](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.6_Reinforcement-Learning-PI) | Reinforcement learning (TD3) agent that tunes the PI gains |

## Requirements
- MATLAB and Simulink (the models were built with a recent release)
- **Simulink Support Package for Arduino Hardware** (hardware models). The hardware models target an **Arduino Mega 2560**.
- System identification: no extra toolboxes. C1.6 also needs the **Reinforcement Learning Toolbox** and **Deep Learning Toolbox** for training.

## How to use
1. Open MATLAB in this folder.
2. Try the `_virtual` model first (press **Run**), then the hardware model (**Hardware > Monitor & Tune**).
3. Stop the model and run `lls_offline.m` to compute the model parameters.

The parameters of each model are in its model workspace (**Modeling > Model Explorer > Model Workspace**). The estimators and controllers are MATLAB Function blocks: double-click one to read its code.

---
<p align="center"><b>Visit <a href="https://www.ace-lab.co.uk">www.ace-lab.co.uk</a></b> for the full ACE-Lab learning resources.<br>Developed by Dr James E. Pickering · Harper Adams University &amp; Aston University · Project funding received from MathWorks and EUCA</p>
