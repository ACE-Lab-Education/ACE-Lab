<p align="center"><a href="https://www.ace-lab.co.uk"><b>🌐 www.ace-lab.co.uk</b></a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/practical-exercises">Practical Exercises</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/ace-core">ACE-CORE</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/build-you-own-ace-lab">Get an ACE-Lab</a></p>

# Self-tuning PI speed control (C1.4, C1.5)

| File | What it does |
|---|---|
| `dc_motor_str.slx` | Hardware model (Arduino Mega 2560, Monitor & Tune): RLS estimator + self-tuning PI by pole placement |
| `dc_motor_str_virtual.slx` | The same with a virtual motor whose gain drops part-way through, to show the controller re-tune |
| `hw_setup.m` | Sets up one test: `hw_setup('Fixed PI','A')`, `hw_setup('LLS + PI','A')` (C1.4) or `hw_setup('RLS self-tuning PI','A')` (C1.5); scenario A or B |
| `hw_collect.m` | Saves and plots a hardware run |
| `verify_requirements_hw.m` | Checks each run against the speed-control requirements |
| `test_str.m`, `plot_str_results.m` | Run and plot the virtual model |
| `build_str_models.m` | Rebuilds both models from scratch |

**LLS + PI (C1.4)** uses `../hw_id_noload2.mat`, the open-loop LLS identification of a real motor. Replace it with your own data from A1.4.

[← Back to the DC motor code](../README.md)

---
<p align="center"><b>Visit <a href="https://www.ace-lab.co.uk">www.ace-lab.co.uk</a></b> for the full ACE-Lab learning resources.<br>Developed by Dr James E. Pickering · Harper Adams University &amp; Aston University · Project funding received from MathWorks and EUCA</p>
