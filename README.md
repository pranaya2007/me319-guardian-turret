# Guardian: predictive fire control of a laser turret

ME319 Control Systems (Autumn 2026), Assignment 2, IIT Bombay.
Author: Pranaya Singla (24B2297)

This repository contains the MATLAB code, figures and animation behind the IEEE-style paper
"The Servo as the Delay Line: Predictive Fire Control of a Laser Turret".

## What the project does

A laser turret must point at a maneuvering drone that is seen only through a noisy radar
(range and elevation, 20 Hz, sometimes blind for up to 0.5 s). The controller

1. estimates the target motion with a constant-acceleration Kalman filter (Cartesian states),
2. predicts the target position 0.5 s ahead and converts it to an elevation reference,
3. tracks that reference with an LTI compensator designed so that the closed loop behaves like a 0.5 s delay,
4. fires a pulse only when the current estimate says the beam is on the target and the estimate is reliable.

One fixed design is used for every scenario. Nothing is retuned per case.

## Requirements

* MATLAB R2025a (written and tested with it).
* Control System Toolbox: needed only by `design_analysis.m` and `robust_check.m`.
  Everything else runs on base MATLAB.

## Folder layout

```
matlab/        all source code (run everything from this folder)
paper/         main.tex and figures/ (scripts write their PDFs to ../paper/figures)
media/         scenC.gif and scenC.mp4 (animation of Scenario C)
docs/          MATLAB_code_guide.md, assignment_understanding.md, session log
results.txt    output of run_all.m
```

## The three functions required by the assignment

```matlab
[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
[r_meas, theta_meas, meas_valid, truth] = target_motion_generator(t, scenario)
```

`turret_controller.m` and `turret_plant.m` read all their numbers from `guardian_params.m`.
They keep their own state in `persistent` variables and reset when `t <= 0`.

## How to reproduce the results

```matlab
cd "Matlab Codes"
design_analysis            % plant, compensator, margins, step response   (Control System Toolbox)
run_all                    % scenarios A, B, C and the 200-case Monte-Carlo study -> results.txt
make_frames                % four snapshots of Scenario C
make_estimation_fig('C')   % estimation and prediction diagnostics, prints numbers
robust_check               % plant-parameter perturbations with the compensator fixed
tune_study                 % sweep over q and K (slow, about 20-30 minutes)
animate_video('C','scenC.mp4')
```

Figures are written to `../paper/figures` by `save_fig.m`.

## Main results (from `results.txt`)

| Scenario | First hit [s] | Pulses / hits | RMS miss [m] | Max miss [m] |
|---|---|---|---|---|
| A (nominal) | 1.113 | 9 / 9 | 0.30 | 0.82 |
| B (maneuver) | 1.116 | 9 / 9 | 0.80 | 1.89 |
| C (maneuver + radar loss 1.0-1.5 s) | 2.372 | 8 / 8 | 0.81 | 1.48 |

Monte-Carlo, 200 seeded random cases: 100 % intercepted, mean interception time 1.116 s (max 2.385 s),
1615 pulses of which 1614 hit (99.9 %), median miss 0.56 m, 95th percentile 1.25 m, maximum 2.81 m.

Design numbers: Gp(s) = 3.125 / (s (s^2 + 10.47 s + 3129.7)); L(s) = 200 / (s (s+5) (s+20));
gain margin 21.9 dB, phase margin 64.2 deg, closed-loop poles -20.6 and -2.19 +/- 2.21j.

## Animation

![Scenario C](media/scenC.gif)

The video version is `media/scenC.mp4`.

## Known limitations

* The controller depends on how often it is called. At 1 ms and 10 ms every scenario behaves the same;
  at 50 ms every pulse still hits but the first hit is later (2.3-2.4 s) and fewer pulses are released.
* No voltage limit, backlash, gravity or structural flexibility is modeled (assignment assumptions).
* One of 1615 Monte-Carlo pulses missed (case 17, 2.81 m), during a hard weave near the acceleration limit.

## Documents

* `docs/MATLAB_code_guide.md`: every file explained in plain language, with the full code.
* `docs/assignment_understanding.md`: the mathematics and design reasoning, including material that did not fit in the paper.
