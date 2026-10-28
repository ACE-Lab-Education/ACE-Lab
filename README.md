<p align="center"><a href="https://www.ace-lab.co.uk"><b>🌐 www.ace-lab.co.uk</b></a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/practical-exercises">Practical Exercises</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/ace-core">ACE-CORE</a> &nbsp;·&nbsp; <a href="https://www.ace-lab.co.uk/build-you-own-ace-lab">Get an ACE-Lab</a></p>

<h1 align="center">ACE-Lab Practical Exercises</h1>
<p align="center"><b>Practical First. Theory Follows.</b><br>Every ACE-Lab exercise sheet, learning guide, and MATLAB &amp; Simulink file in one place.</p>

<p align="center"><img src="https://custom-images.strikinglycdn.com/res/hrscywv4p/image/upload/f_auto,q_auto/22329992/543744_727789.png" alt="ACE-Lab Base + Sense" height="180">&nbsp;&nbsp;<img src="https://custom-images.strikinglycdn.com/res/hrscywv4p/image/upload/f_auto,q_auto/22329992/895721_522242.png" alt="ACE-Lab Actuate" height="180"></p>

> 🌐 **The full ACE-Lab learning resources are at [www.ace-lab.co.uk](https://www.ace-lab.co.uk).** This repository holds all the files: exercise sheets, ACE-CORE learning guides, and MATLAB &amp; Simulink files.

## How ACE-Lab works
| 1 · Get the hardware | 2 · Do the practical exercise | 3 · Learn the theory |
|---|---|---|
| **Base + Sense**: Arduino, breadboard, LEDs, switches and sensors. **Actuate**: DC motor with encoder and cooling fan. | Each exercise has its PDF sheet, DOI, and MATLAB & Simulink files here on GitHub. Run the model on real hardware and see the result. | Each exercise links to the **ACE-CORE** guides that explain why it works, from first principles to adaptive control. |
| [Get an ACE-Lab →](https://www.ace-lab.co.uk/build-you-own-ace-lab) | [Browse the exercises →](https://github.com/ACE-Lab-Education/ACE-Lab) | [ACE-CORE →](https://www.ace-lab.co.uk/ace-core) |


## Getting started
1. **Download this repository:** click the green **Code** button, then **Download ZIP** (or `git clone https://github.com/ACE-Lab-Education/ACE-Lab.git`).
2. Complete **G1** to install MATLAB, Simulink and the Arduino support package.
3. Open an exercise folder, follow its PDF sheet, and run the model from `simulink/`.
4. Read the linked **ACE-CORE** guide to understand the theory.

## Repository structure
```
ACE-Lab/
├── 01_Getting-Started/        G1–G3   (software setup, MATLAB & Simulink Onramps)
├── 02_Preliminary-Labs/       P1–P4   (Base + Sense)
├── 03_Measurement-Labs/       M1–M4   (Base + Sense)
├── 04_Actuator-Labs/          A1.1–A2 (Base + Sense + Actuate)
├── 05_Control-System-Labs/    C1.1–C1.6 (Base + Sense + Actuate)
├── ACE-CORE/                  all ACE-CORE learning guide PDFs, by course level
└── Code_DC-Motor/             MATLAB & Simulink files for the DC motor labs (A1.4–A1.5, C1.4–C1.6)
```
Every exercise folder contains a `README.md`, the exercise sheet PDF, a `matlab/` folder and a `simulink/` folder.

## Practical exercises
### [Getting Started](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/01_Getting-Started)
*Hardware: Base + Sense*

| Code | Exercise | Time | Sheet | DOI |
|---|---|---|---|---|
| **G1** | [Install MATLAB, Simulink and Code Generation](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/01_Getting-Started/G1_Install-MATLAB-Simulink) | 1 hour | — | — |
| **G2** | [MATLAB Onramp](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/01_Getting-Started/G2_MATLAB-Onramp) | 2 hours | [Course](https://matlabacademy.mathworks.com/details/matlab-onramp/gettingstarted) | — |
| **G3** | [Simulink Onramp](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/01_Getting-Started/G3_Simulink-Onramp) | 2 hours | [Course](https://matlabacademy.mathworks.com/details/simulink-onramp/simulink) | — |

### [Preliminary Labs](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/02_Preliminary-Labs)
*Hardware: Base + Sense*

| Code | Exercise | Time | Sheet | DOI |
|---|---|---|---|---|
| **P1** | [LED (Digital)](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/02_Preliminary-Labs/P1_LED-Digital) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/02_Preliminary-Labs/P1_LED-Digital/P1_LED_Digital_Exercise.pdf) | — |
| **P2** | [LED (PWM)](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/02_Preliminary-Labs/P2_LED-PWM) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/02_Preliminary-Labs/P2_LED-PWM/P2_LED_PWM_Exercise.pdf) | — |
| **P3** | [On/Off Power Switch](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/02_Preliminary-Labs/P3_On-Off-Power-Switch) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/02_Preliminary-Labs/P3_On-Off-Power-Switch/P3_On_Off_Power_Switch.pdf) | — |
| **P4** | [Tactile Button Switch](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/02_Preliminary-Labs/P4_Tactile-Button-Switch) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/02_Preliminary-Labs/P4_Tactile-Button-Switch/P4_Tactile_Button_Switch.pdf) | — |

### [Measurement Labs](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/03_Measurement-Labs)
*Hardware: Base + Sense*

| Code | Exercise | Time | Sheet | DOI |
|---|---|---|---|---|
| **M1** | [LDR (Light Dependent Resistor)](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/03_Measurement-Labs/M1_LDR) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/03_Measurement-Labs/M1_LDR/M1_LDR_Light_Dependent_Resistor.pdf) | — |
| **M2** | [Slide Potentiometer](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/03_Measurement-Labs/M2_Slide-Potentiometer) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/03_Measurement-Labs/M2_Slide-Potentiometer/M2_Slide_Potentiometer.pdf) | — |
| **M3** | [Ultrasonic Sensor](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/03_Measurement-Labs/M3_Ultrasonic-Sensor) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/03_Measurement-Labs/M3_Ultrasonic-Sensor/M3_Ultrasonic_Sensor.pdf) | — |
| **M4** | [Temperature Sensor](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/03_Measurement-Labs/M4_Temperature-Sensor) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/03_Measurement-Labs/M4_Temperature-Sensor/M4_Temperature_Sensor.pdf) | — |

### [Actuator Labs](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs)
*Hardware: Base + Sense + Actuate*

| Code | Exercise | Time | Sheet | DOI |
|---|---|---|---|---|
| **A1.1** | [DC Motor Polarity Control](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.1_DC-Motor-Polarity) | 0.5 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/04_Actuator-Labs/A1.1_DC-Motor-Polarity/A1.1_DC_motor_polarity_control.pdf) | — |
| **A1.2** | [DC Motor Speed Measurement using an Encoder](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.2_DC-Motor-Speed-Encoder) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/04_Actuator-Labs/A1.2_DC-Motor-Speed-Encoder/A1.2_DC_motor_speed_measurement.pdf) | [23019221](https://doi.org/10.5281/zenodo.23019221) |
| **A1.3** | [Discrete-Time Low-Pass Filter Design for DC Motor Speed Measurement](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.3_Low-Pass-Filter) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/04_Actuator-Labs/A1.3_Low-Pass-Filter/A1.3_Low_pass_filter_dc_motor.pdf) | [23244891](https://doi.org/10.5281/zenodo.23244891) |
| **A1.4** | [Parameter Estimation of a DC Motor using Linear Least Squares](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.4_LLS-Parameter-Estimation) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/04_Actuator-Labs/A1.4_LLS-Parameter-Estimation/A1.4_Parameter_Estimation_of_a_DC_Motor_using_LLS.pdf) | [23568474](https://doi.org/10.5281/zenodo.23568474) |
| **A1.5** | [Online Parameter Estimation of a DC Motor using Recursive Least Squares](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A1.5_RLS-Online-Estimation) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/04_Actuator-Labs/A1.5_RLS-Online-Estimation/A1.5_Online_Parameter_Estimation_of_a_DC_Motor_using_RLS.pdf) | [23671549](https://doi.org/10.5281/zenodo.23671549) |
| **A2** | [DC Motor Cooling Fan](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/04_Actuator-Labs/A2_Cooling-Fan) | 1 hour | — | — |

### [DC Motor Control System Labs](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs)
*Hardware: Base + Sense + Actuate*

| Code | Exercise | Time | Sheet | DOI |
|---|---|---|---|---|
| **C1.1** | [DC Motor Speed Control Using a PI Controller](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.1_PI-Speed-Control) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/05_Control-System-Labs/C1.1_PI-Speed-Control/C1.1_DC_Motor_Speed_Control_Using_a_PI_Controller.pdf) | [23043611](https://doi.org/10.5281/zenodo.23043611) |
| **C1.2** | [DC Motor PI Control: Manual Tuning](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.2_PI-Manual-Tuning) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/05_Control-System-Labs/C1.2_PI-Manual-Tuning/C1.2_Manual_Tuning_Discrete_Time_PID_Controller.pdf) | [23245132](https://doi.org/10.5281/zenodo.23245132) |
| **C1.3** | [DC Motor PI Control: Automatic Optimisation](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.3_PI-Automatic-Optimisation) | 1 hour | — | — |
| **C1.4** | [DC Motor PI Control Design by Pole Placement from an LLS Model](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.4_PI-Pole-Placement) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/05_Control-System-Labs/C1.4_PI-Pole-Placement/C1.4_DC_Motor_PI_Control_Design_by_Pole_Placement_from_an_LLS_Model.pdf) | [23671552](https://doi.org/10.5281/zenodo.23671552) |
| **C1.5** | [DC Motor Self-Tuning PI Control using Recursive Least Squares](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.5_Self-Tuning-PI) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/05_Control-System-Labs/C1.5_Self-Tuning-PI/C1.5_DC_Motor_Self_Tuning_PI_Control_using_RLS.pdf) | [23671554](https://doi.org/10.5281/zenodo.23671554) |
| **C1.6** | [DC Motor PI Tuning using Reinforcement Learning](https://github.com/ACE-Lab-Education/ACE-Lab/tree/main/05_Control-System-Labs/C1.6_Reinforcement-Learning-PI) | 1 hour | [PDF](https://github.com/ACE-Lab-Education/ACE-Lab/blob/main/05_Control-System-Labs/C1.6_Reinforcement-Learning-PI/C1.6_DC_Motor_PI_Tuning_using_Reinforcement_Learning.pdf) | [23671556](https://doi.org/10.5281/zenodo.23671556) |

## ACE-CORE learning guides
All learning guide PDFs are in the **[ACE-CORE folder](ACE-CORE)**, sorted by course level, with a table showing which guide goes with which exercise.

## Citing
Each exercise and learning guide has its own DOI. Please cite the specific resource you use.

---
<p align="center"><b>Visit <a href="https://www.ace-lab.co.uk">www.ace-lab.co.uk</a></b> for the full ACE-Lab learning resources.<br>Developed by Dr James E. Pickering · Harper Adams University &amp; Aston University · Project funding received from MathWorks and EUCA</p>
