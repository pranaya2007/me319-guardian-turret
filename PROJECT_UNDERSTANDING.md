# My Understanding of the Guardian Turret Project

**Author:** Pranaya Singla (24B2297)  
**Course:** ME319 Control Systems, Assignment 2

This document summarises my understanding of the Guardian Turret project, including the problem being addressed, the approach used to develop the solution, and the reasoning behind the major design decisions.

The aim is to explain the project in a simple and structured manner, focusing on how the different components work together. I have also included the reasoning behind conducting simulations, as they helped me understand the actual behaviour of the system and verify whether the theoretical design worked as expected.

A more detailed mathematical explanation is provided in `assignment_understanding.md`.

---

## 1. Understanding the Problem

The objective of this project is to design a laser turret that can accurately track and engage a moving drone.

The turret receives information about the drone through a radar system. However, the radar does not provide continuous information. It gives noisy measurements of the range and elevation angle every 50 ms, and there can also be periods during which the radar cannot detect the drone for up to 0.5 seconds.

The laser operates by releasing individual pulses, with a maximum firing frequency of one pulse per second. A pulse is considered successful if the laser beam passes within 2 m of the drone.

At distances between 800 and 2000 m, this 2 m engagement radius corresponds to an angular accuracy of only 1.0 to 2.5 milliradians. Additionally, the line of sight of the drone can change by as much as 94 milliradians in just 0.5 seconds.

This makes the problem challenging because the turret cannot simply point towards the drone's current measured position. By the time the turret responds, the drone would already have moved.

Therefore, the system needs to estimate the drone's motion, predict its future position and move the turret towards that predicted position. The timing of the movement is equally important because reaching the predicted position too early or too late would result in an inaccurate engagement.

---

## 2. Overall Working of the System

The solution consists of multiple components that work together to estimate the target's motion, predict its future position, control the turret and decide when to release a pulse.

The overall sequence is:

```text
Radar measurements (range, elevation)
                 |
                 v
            Kalman Filter
                 |
                 v
       Predict 0.5 s into the future
                 |
                 v
        Calculate elevation reference
                 |
                 v
          Compensator Gc(s)
                 |
                 v
             Motor Voltage
                 |
                 v
             Turret Motion
                 |
                 v
             Beam Angle

       Fire Gate controls pulse release
```

The working of each stage is explained below.

1. **State Estimation:** The radar measurements are noisy, so they cannot be used directly for controlling the turret. A Kalman filter processes these measurements and estimates the drone's position, velocity and acceleration.

2. **Future Position Prediction:** Using the estimated position, velocity and acceleration, the system predicts where the drone will be after 0.5 seconds. This is done using the constant-acceleration motion equation.

3. **Reference Generation:** The predicted Cartesian coordinates are converted into an elevation angle using `atan2(y,x)`. This angle becomes the reference that the turret needs to follow.

4. **Turret Control:** A linear compensator is used to control the motor voltage and move the turret towards the reference angle.

5. **Pulse Release:** A separate firing condition checks whether the beam is sufficiently close to the estimated target position and whether the uncertainty in the estimate is within acceptable limits. A pulse is released only when all the conditions are satisfied.

The important point is that estimation, prediction, control and firing are handled as separate stages, but they work together to achieve accurate engagement.

---

## 3. The Main Design Idea: Using the Servo as a Delay

One of the most important decisions in this project was how to make the turret respond to the predicted position.

The system predicts the drone's position 0.5 seconds into the future. Since the laser pulse is treated as instantaneous, the turret must reach this predicted position after exactly 0.5 seconds.

In other words, the turret should not reach the reference immediately. It should follow a response that introduces the required delay.

If the turret responds too quickly, the beam will point ahead of the drone. If it responds too slowly, the drone will have already moved beyond the predicted position.

Therefore, instead of designing the controller simply to achieve the highest possible speed, I designed the closed-loop response to approximate a pure time delay of 0.5 seconds.

### How this was converted into mathematical conditions

The desired closed-loop transfer function was matched with the Taylor series approximation of a pure time delay:

\[
e^{-st_p}\approx 1-t_ps+\frac{t_p^2s^2}{2}-\cdots
\]

Using this approximation, the controller parameters were related as follows:

\[
\beta=Kt_p
\]

This gives:

\[
K_v=\frac{K}{\beta}=\frac{1}{t_p}=2\text{ s}^{-1}
\]

Similarly,

\[
\alpha=\frac{Kt_p^2}{2}
\]

The value of \(K_v\) is important because it determines the ramp tracking behaviour of a type-1 control system. The ramp tracking error is proportional to the reference slope and inversely proportional to \(K_v\).

Since the predicted reference leads the actual target by 0.5 seconds, the servo must introduce an equivalent lag. Setting \(K_v=1/t_p\) makes the two effects cancel under the assumed tracking conditions.

I also checked the stability conditions using the Routh-Hurwitz criterion. The resulting condition was:

\[
\alpha\beta>K
\]

Substituting the expressions for alpha and beta gives:

\[
K>\frac{2}{t_p^3}
\]

For \(t_p=0.5\) seconds:

\[
K>16
\]

I selected:

\[
K=200
\]

This resulted in the desired open-loop transfer function:

\[
L(s)=\frac{200}{s(s+5)(s+20)}
\]

The compensator was then obtained by dividing the desired open-loop transfer function by the plant transfer function:

\[
G_c(s)=
\frac{64(s^2+10.47s+3129.7)}
{(s+5)(s+20)}
\]

This design was important because it allowed the turret to provide the required delayed response instead of trying to make it follow the reference as quickly as possible.

---

## 4. Understanding the Turret Plant

The plant represents the actual physical system consisting of the motor, armature circuit and turret mechanics.

The transfer function obtained from the system model is:

\[
G_p(s)=
\frac{3.125}
{s(s^2+10.47s+3129.7)}
\]

The model also accounts for the 100:1 gear ratio, which affects the effective rotor inertia through the square of the gear ratio.

A few important observations from the plant are:

- **Pole at the origin:** The plant contains an integrator, making it a type-1 system. This means that a constant motor voltage produces a constant angular velocity under the model.

- **Lightly damped resonance:** The quadratic term represents a relatively lightly damped resonant mode. Its natural frequency is approximately 55.9 rad/s, with a damping ratio of 0.094.

- **Back EMF:** The motor's back electromotive force has a significant effect on the system and opposes the applied voltage as the motor rotates.

- **Resonance cancellation:** The compensator contains zeros that cancel the resonance represented by the quadratic term in the plant transfer function.

Since exact pole-zero cancellation can be sensitive to changes in physical parameters, I also performed a robustness check. In this check, the compensator was kept fixed while the plant parameters were varied.

The tested cases remained stable, with the slowest pole reaching -2.11 in the worst case when Ke was reduced by 20%.

This check helped me understand why it is important to examine the behaviour of the system even when the actual plant does not perfectly match the mathematical model.

### 4.1 Why Does the First Hit Occur at Approximately 1.1 Seconds?

The closed-loop system has a step response with a rise time of approximately 1.12 seconds.

Since the system is linear, its response time is independent of the magnitude of the step input. Therefore, for the tested initial conditions, the beam reaches the required target position and produces its first crossing at approximately 1.1 seconds.

This explains why the first successful hits in Scenarios A and B occur at around 1.1 seconds.

Scenario C is different because the firing gate remains closed during the relevant radar loss period. Therefore, even though the beam crosses the target earlier, the system does not release a pulse at that time.

---

## 5. Why I Used a Kalman Filter in Cartesian Coordinates

The Kalman filter is responsible for estimating the drone's position and motion from noisy radar measurements.

One important decision was to perform the estimation in Cartesian coordinates instead of directly working with range and elevation angle.

There were several reasons for this choice:

- **Simpler motion equations:** The drone's position, velocity and acceleration can be represented directly in the x and y directions. This makes it easier to apply the constant-acceleration model.

- **Linear estimation model:** The motion model is approximately linear in Cartesian coordinates. Therefore, a standard linear Kalman filter is sufficient for the estimation process. Working directly with polar measurements would require a nonlinear estimation approach, such as an Extended Kalman Filter.

- **Handling radar noise:** The radar provides measurements in range and elevation angle. These measurements are converted into Cartesian coordinates, and the associated measurement noise is transformed using a Jacobian matrix.

- **Measurement accuracy:** The radar provides better accuracy across the line of sight, with an error of approximately 0.28 to 0.70 m, compared with approximately 1 m along the line of sight. This difference is important because errors in the direction affecting the elevation angle directly influence the turret's pointing accuracy.

### Why Constant Acceleration Instead of Constant Velocity?

I chose a constant-acceleration model because assuming that the drone maintains a constant velocity would not be sufficiently accurate during manoeuvres.

For example, if an acceleration of 20 m/s² remains unaccounted for over a period of 0.5 seconds, the resulting position error would be:

\[
\Delta x=\frac{1}{2}at^2
\]

\[
\Delta x=\frac{1}{2}(20)(0.5)^2=2.5\text{ m}
\]

This is already greater than the allowed engagement radius of 2 m.

Therefore, including acceleration in the estimation model is important for maintaining sufficient prediction accuracy.

### Selection of Process Noise Parameter

The process noise parameter was selected as:

\[
q=1000
\]

This value was estimated using the expected jerk of the target during a manoeuvre.

For a 2 rad/s weaving motion at the acceleration limit, the expected jerk is approximately 40 m/s³. Using the relationship:

\[
q\approx\frac{40^2}{2}=800
\]

I selected 1000 as the design value. I also performed a parameter sweep to study how different values of q affected the RMS miss distance. This showed that the selected value was within a reasonably good operating range.

During radar loss, the Kalman filter continues predicting the target's motion using its previous estimates. However, because no new measurements are available to correct the prediction, the uncertainty gradually increases.

---

## 6. Understanding the Firing Conditions

The firing mechanism is controlled by a separate set of conditions, which I refer to as the fire gate.

The system does not release a pulse every time the beam appears to be aligned with the target. It also checks whether the estimated position is reliable enough to make that decision.

The three main conditions are:

1. The estimated distance between the beam and the target must be less than or equal to 0.8 m.
2. The uncertainty in this distance must be less than or equal to 0.6 m.
3. At least one second must have passed since the previous pulse.

The first two conditions are related to the engagement radius:

\[
0.8+2(0.6)=2.0\text{ m}
\]

This ensures that the estimated alignment and its uncertainty remain within the specified 2 m engagement radius, using a two-standard-deviation uncertainty allowance.

The uncertainty condition is particularly important during radar loss. As the filter continues predicting without receiving new measurements, its uncertainty increases. Once it exceeds 0.6 m, the firing gate prevents the system from releasing a pulse.

This explains the delayed first pulse in Scenario C. Although the beam crosses the target at approximately 1.1 seconds, the system does not fire because the uncertainty is too high during the radar loss period. The first successful pulse is therefore released at approximately 2.37 seconds, once the firing conditions are satisfied again.

---

## 7. Why I Performed Simulations

Although the mathematical model and controller design provided the theoretical basis for the project, I also needed to understand how the complete system would behave when all the components worked together.

For this reason, I developed and ran simulations in MATLAB.

The simulations were important because they allowed me to observe the actual response of the turret, the movement of the target, the effect of noisy radar measurements and the behaviour of the firing mechanism over time.

They also helped me understand aspects of the system that were difficult to interpret from the transfer functions and equations alone.

I used the simulations for the following purposes:

- **Understanding the complete system:** By running the engagement scenarios, I could observe how the radar measurements passed through the estimation and prediction stages before affecting the turret movement.

- **Testing different target trajectories:** Scenarios A, B and C helped me understand how the system responded under different target motion conditions.

- **Studying radar loss:** I could observe how the Kalman filter continued predicting the target's position during periods without radar measurements and how the increasing uncertainty affected the firing decision.

- **Verifying the controller:** The simulation allowed me to check whether the turret reached the predicted reference at the intended time and whether the beam was sufficiently close to the target when a pulse was released.

- **Testing reliability:** I also ran 200 random test cases to check whether the system continued to perform well under different target trajectories and motion conditions.

- **Identifying limitations:** The simulations helped identify situations where the system did not perform as expected, such as the single missed pulse in the Monte Carlo tests and the changes observed when the simulation time step was increased.

The simulation was therefore not just used to generate the final numbers and figures. It was also an important part of understanding the project and checking whether the theoretical design decisions translated into the expected system behaviour.

---

## 8. Results Obtained from the Simulations

The main results obtained from the MATLAB simulations are summarised below.

| Performance Measure | Result |
|---|---|
| First hit in Scenario A | 1.113 s |
| First hit in Scenario B | 1.116 s |
| First hit in Scenario C | 2.372 s |
| Successful pulses in Scenario A | 9/9 |
| Successful pulses in Scenario B | 9/9 |
| Successful pulses in Scenario C | 8/8 |
| RMS miss distance in Scenario A | 0.30 m |
| RMS miss distance in Scenario B | 0.80 m |
| RMS miss distance in Scenario C | 0.81 m |
| Monte Carlo interception rate | 100% |
| Total successful pulses in 200 random cases | 1614 out of 1615 |
| Median pulse miss distance | 0.56 m |
| 95th percentile pulse miss distance | 1.25 m |
| Maximum pulse miss distance | 2.81 m |

One interesting observation was that the raw prediction error was considerably higher than the final beam miss distance.

After 3 seconds, the prediction error had an RMS value of 2.42 m and reached a maximum of 8.21 m. However, the RMS beam miss distance in Scenario C was only 0.81 m.

This difference can be explained by the relatively slow response of the servo, which has a bandwidth of approximately 3.1 rad/s. The servo does not immediately follow every small fluctuation in the predicted reference. Instead, it smooths out rapid changes, resulting in a more stable beam trajectory.

Another important observation was the single missed pulse in the 200 random test cases. This occurred in case 17 during a relatively aggressive target manoeuvre, where the acceleration was between 17 and 19 m/s². There was no radar loss immediately before the missed pulse.

This showed that although the system performed well across the tested cases, it was not completely immune to sudden target manoeuvres.

---

## 9. Limitations of the Current Model

While the simulation results were encouraging, there are certain limitations that need to be considered.

- **Limited test cases:** The instructor's evaluation cases were not available during the development of the project. Therefore, the system was tested using the predefined scenarios and my own 200 random test cases.

- **Effect of simulation time step:** When the controller call interval was increased to 50 ms, all released pulses still resulted in hits. However, the first successful hit occurred later and the number of pulses decreased. This indicates that the system's performance is sensitive to the controller update interval.

- **Idealised physical model:** The current model does not include practical effects such as voltage saturation, mechanical backlash, gravity and structural flexibility.

- **Motor voltage requirements:** The maximum voltage reached during the first turret movement was approximately 476 V. This would exceed the limits of a typical real-world amplifier, indicating that the current simulation does not yet account for all practical actuator constraints.

- **Prediction limitations:** The prediction horizon is fixed at 0.5 seconds. If the drone suddenly changes its acceleration or reverses its direction of motion, the prediction may become inaccurate.

These limitations are important because they distinguish the performance of the mathematical simulation from what could be expected from an actual physical system.

---

## 10. Important Questions I Should Be Able to Answer in a Viva

**1. Why did you introduce a delay in the turret response?**

The system predicts the target's position 0.5 seconds into the future. Therefore, the turret should reach that position after 0.5 seconds. The delay helps synchronise the turret movement with the future target position.

**2. How do you know that the system is stable?**

The Routh-Hurwitz criterion gives the condition \(K>16\). The selected value is \(K=200\), which satisfies this condition. I also checked the closed-loop poles, gain margin and phase margin. The gain margin is 21.9 dB and the phase margin is 64.2 degrees.

**3. Why does the system achieve a small beam miss distance even when the prediction error is large?**

The servo has a relatively slow response and does not immediately follow every fluctuation in the predicted reference. This smooths the turret movement. Additionally, the firing gate releases pulses only when the estimated alignment and uncertainty satisfy the required conditions.

**4. What happens when the radar loses the target?**

The Kalman filter continues predicting the target's position using its previous state estimates. However, its uncertainty increases because no new measurements are available. Once the uncertainty exceeds the specified limit, the firing gate prevents the release of a pulse.

**5. Why did you use a constant-acceleration model?**

A constant-velocity model would not account for target manoeuvres. Even an acceleration of 20 m/s² over 0.5 seconds can introduce a position error of 2.5 m, which is greater than the allowed engagement radius.

**6. Why did you perform simulations if the mathematical model was already available?**

The mathematical model helped me derive the controller and understand the theoretical behaviour of the system. However, it was important to see how estimation, prediction, turret movement and firing conditions worked together. The simulations helped me understand the complete system, test different scenarios, verify the theoretical results and identify limitations that were not immediately obvious from the equations alone.

**7. What was the role of AI in this project, and what did you verify yourself?**

AI tools were used to help propose the initial system architecture and assist with drafting parts of the code and documentation. I then implemented and ran the scripts in MATLAB, checked the outputs, replaced provisional values with the results obtained from my own simulations, and removed or corrected claims that were not supported by the results. I also used an independent verification script to check the important calculations.

---

## 11. Final Understanding

Through this project, I understood how different concepts from control systems can be combined to solve a practical tracking and control problem.

The most important part of the design was understanding that accurate tracking does not necessarily mean making the turret respond as quickly as possible. Since the system works with predicted future positions, the turret response must be properly timed to match the prediction horizon.

I also understood the importance of state estimation when working with noisy measurements and how uncertainty becomes particularly important when sensor data is unavailable.

Finally, performing simulations helped me move beyond the mathematical derivations and understand how the complete system behaves under different operating conditions. It allowed me to verify the design, investigate unexpected results and identify areas where the model could be improved further.

Overall, the project helped me connect the theoretical concepts studied in ME319 with their practical implementation in MATLAB and gave me a better understanding of how a complete feedback control system is designed, tested and evaluated.
