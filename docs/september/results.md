# Benchmark results

These results were measured on this reconstruction using Python 3.12.11 and NumPy 2.2.6. There are 400 trials: 50 seeds (100–149), four disturbance scenarios, and two controllers. Each trial has a 120 s limit at a 0.1 s time step. Development checks used separate seeds.

```sh
python -m navigation benchmark --trials 50 --seed 100 --output results/benchmark
```

![Policy comparison](comparison.png)

| Scenario | Policy | Success | 95% interval | Collisions | Position RMSE (m) | Recoveries | Time to goal (s) |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: |
| nominal | fixed | 37/50 | 60%–84% | 0 | 0.096 | 0.00 | 31.2 |
| nominal | adaptive | 37/50 | 60%–84% | 0 | 0.105 | 0.00 | 42.8 |
| slip | fixed | 2/50 | 1%–13% | 35 | 0.366 | 0.00 | 31.4 |
| slip | adaptive | 12/50 | 14%–37% | 6 | 0.182 | 4.84 | 53.0 |
| drift | fixed | 38/50 | 63%–86% | 0 | 0.095 | 0.00 | 31.2 |
| drift | adaptive | 36/50 | 58%–83% | 0 | 0.106 | 0.00 | 42.8 |
| combined | fixed | 2/50 | 1%–13% | 36 | 0.368 | 0.00 | 31.4 |
| combined | adaptive | 11/50 | 13%–35% | 7 | 0.177 | 4.76 | 52.7 |

RMSE and recovery counts are means across all trials. Time to goal averages successful trials only, so it compares different surviving subsets. Error bars show uncertainty in the estimated success rate; they are not a test of statistical significance between the paired policies.

Slowing and recovery reduced collisions under slip: 35 to 6 in the slip scenario and 36 to 7 with slip plus gyro drift. Success rose from 2 to 12 trials and from 2 to 11 trials respectively. Most disturbed trials still failed. The controller is a useful safety improvement in this model, but it does not solve localization under wheel slip.

Without slip, adaptive speed gave no success improvement and increased travel time. With a long dead-reckoning route and a strict final-goal tolerance, some robots stopped at the estimated goal while the true pose was outside tolerance. There is no absolute position sensor to correct that error.

## Failure counts

| Scenario | Policy | Collision | Missed goal | Replan failed | Recovery limit | Timeout |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| nominal | fixed | 0 | 13 | 0 | 0 | 0 |
| nominal | adaptive | 0 | 13 | 0 | 0 | 0 |
| slip | fixed | 35 | 13 | 0 | 0 | 0 |
| slip | adaptive | 6 | 17 | 13 | 2 | 0 |
| drift | fixed | 0 | 12 | 0 | 0 | 0 |
| drift | adaptive | 0 | 14 | 0 | 0 | 0 |
| combined | fixed | 36 | 12 | 0 | 0 | 0 |
| combined | adaptive | 7 | 18 | 12 | 2 | 0 |

Raw records: [trials.csv](benchmark/trials.csv), [summary.json](benchmark/summary.json). The README example is nominal/adaptive seed 0; it reaches the goal in 44.2 s with 0.071 m position RMSE. These measurements describe this simulator, not physical hardware or the lost original implementation.
