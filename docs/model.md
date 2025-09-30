# Model and assumptions

## Robot and map

The robot is a circular footprint of radius 0.20 m, with a 0.36 m wheel track,
0.06 m wheel radius, and 2,048 encoder counts per revolution. The map is
12 × 10 m. Start pose is (1, 1, 0) and the goal is (11, 9).

For wheel speeds `left` and `right` in m/s:

```
v = (right + left) / 2
w = (right - left) / track
```

The plant integrates constant wheel speeds along a circular arc at a 0.1 s
time step. There is no motor lag, wheel inertia, terrain model, or lateral
dynamics. Commands can change instantaneously.

A* uses a 0.25 m grid with eight neighbors and a Euclidean heuristic. Diagonal
moves cannot pass between blocked cells. Collision checks during planning use
a 0.45 m radius, including a 0.25 m clearance margin. A line-of-sight pass
removes unnecessary waypoints. Segments are checked at intervals no longer
than 0.04 m. Physical collision checks use the 0.20 m footprint along the
chord between successive true positions. This is a sampled sweep, not a
continuous contact solver.

## Sensors

Encoder scale factors are drawn once per trial. Gaussian distance errors are
added to cumulative wheel travel before converting it to integer encoder
counts. The difference between successive counts gives the measured wheel
speeds. Distance noise is specified per sample at the stated 10 Hz rate.

The gyro measures true yaw rate plus a drifting bias and white noise:

```
bias_next = bias + Normal(0, bias_walk * sqrt(dt))
gyro = true_yaw_rate + bias_next + Normal(0, gyro_noise)
```

| Parameter | Nominal | Slip | Drift | Combined |
| --- | ---: | ---: | ---: | ---: |
| Encoder scale standard deviation | 0.0005 | 0.0005 | 0.0005 | 0.0005 |
| Encoder distance noise (m/sample) | 0.0002 | 0.0002 | 0.0002 | 0.0002 |
| Gyro noise (rad/s) | 0.015 | 0.015 | 0.015 | 0.015 |
| Initial gyro bias standard deviation (rad/s) | 0.015 | 0.015 | 0.060 | 0.060 |
| Bias random walk (rad/s/sqrt(s)) | 0.001 | 0.001 | 0.008 | 0.008 |
| Slip arrival rate while inactive (1/s) | 0 | 0.18 | 0 | 0.18 |

Each slip event affects one randomly selected wheel for 0.4–1.6 s, reducing
traction by 32.5–65%. Events do not overlap. The noise and slip schedule use a
fixed number of random draws per time step, so two policies with the same seed
encounter the same disturbances at the same simulated times.

## EKF

The state is `[x, y, heading, v, w, gyro_bias]`. A constant-velocity model predicts
the next pose using midpoint heading. The transition Jacobian propagates the
6 × 6 covariance. Process noise includes speed and yaw-rate changes, small
position and heading disturbances, and gyro-bias drift. Speed and yaw-rate
process noise are also propagated into pose for the current interval.

Measurements are encoder forward speed, encoder yaw rate, and gyro yaw rate:

```
h(state) = [v, w, w + gyro_bias]
R = diag([0.003^2, 0.012^2, 0.015^2])
```

The normalized difference between the gyro and encoder yaw rates, after
subtracting estimated bias, is a slip indicator. Above three standard
deviations the filter increases encoder measurement variance. A detected
disagreement also increases the next position process covariance. The update
uses the Joseph covariance form and wraps heading to [-pi, pi).

These noise assumptions are heuristic. The initial pose is known. There are
no GPS, camera, magnetometer, landmarks, or ground-truth corrections. Encoder
scale error and undetected common-wheel slip can make position error grow
beyond the reported uncertainty. Persistent wheel error can also be mistaken
for gyro bias. The plotted two-sigma curve is an uncertainty indicator, not
a calibrated bound for every disturbance.

## Control and recovery

Heading uses proportional control with gain 2.5 and a 1.6 rad/s limit. Forward
speed is capped at 0.65 m/s, reduced near the next waypoint, and multiplied by
the cube of the positive heading-error cosine. Both policies use those rules.

The adaptive policy additionally multiplies speed by
`max(0.30, 1 / (1 + 3 * position_sigma))`, where position sigma is the square
root of the largest eigenvalue of the position covariance. A slip indication
reduces speed further. Disagreement lasting at least 0.3 s triggers a one-second
stop and replanning, with a four-second cooldown between recovery starts.
Eight recoveries are allowed per trial. Replanning failure ends the trial.

This recovery can settle the bias estimate while stationary and redirect the
planned route. It cannot recover an absolute position fix from these sensors.
Lower speed also increases exposure time to random disturbances, so the
adaptive policy is not guaranteed to improve success rate.

## Scoring

The controller stops when its estimate is within 0.18 m of the goal. Scoring
then requires true goal distance below 0.35 m for success; otherwise the trial
is `missed_goal`. Collision, timeout, failed replanning, and exhausting the
recovery allowance are separate failure outcomes. A collision ends a trial,
so collision count is zero or one.

Position and heading RMSE use every recorded sample up to termination, including
failed trials. The encoder-only trajectory provides a localization baseline
along the same physical trial; it does not control a second robot. Final error
means true-to-estimated position distance. Mean time to goal includes successful
trials only and is null when none succeed. Group RMSE is the arithmetic mean
of per-trial RMSE, not an average weighted by trial duration. Success intervals
use the 95% Wilson binomial interval.

The fixed and adaptive policies use the same EKF. Their comparison isolates
controller behavior; it is not an ablation of the filter's slip handling.
