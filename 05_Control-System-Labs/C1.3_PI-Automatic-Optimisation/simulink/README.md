# C1.3 · simulink: identify → tune → verify

A complete pipeline for the Arduino Uno. Open MATLAB in this folder first.

| Step | File | What it does |
|---|---|---|
| 1 | `ace_id_arduino.slx` | Runs the motor open loop for 30 s with voltage steps (**Monitor & Tune**). |
| 2 | `ace_step2_identify.m` | Least-squares model of the motor → `ace_id_model.mat` |
| 3 | `ace_step3_tune.m` | Optimises the PI gains (`fminsearch`) → `ace_tuned_gains.mat` |
| 4 | `ace_cl_arduino.slx` | Closed-loop test with the tuned gains (30 rpm step) |
| 5 | `ace_step4_verify.m` | Checks rise time < 1 s and zero steady-state error: PASS/FAIL |

**No hardware?** Run `run_virtual_pipeline`, which uses the `_virtual` models.

Other files: `ace_params.m` (pins, supply and sample time), `ace_speed_sim.slx` and `ace_speed_arduino.slx` (basic speed control), `build_ace_speed_control.m` and `build_ace_sysid_pipeline.m` (rebuild the models), `aceGetSignal.m` (helper). `ace_tuned_gains.mat` starts with gains tuned on the virtual rig; step 3 overwrites it with gains for your motor.

[← Back to C1.3](../README.md) · [www.ace-lab.co.uk](https://www.ace-lab.co.uk)
