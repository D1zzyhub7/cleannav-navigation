# Dynamic obstacle stress results (2026-10-07)

## Matrix

Each configuration has five valid headless runs with different deterministic
static-obstacle seeds. A run is successful only when all of these conditions
hold:

1. the navigation action succeeds;
2. the command/odometry log contains a real stop followed by motion recovery;
3. the final Gazebo world pose is within 0.35 m of the requested goal.

| Group | Dynamic setup | Scan obstacle / raytrace | Timing and direction | Valid runs | Result |
| --- | --- | --- | --- | ---: | --- |
| A | one pedestrian at 2.4 m/s, X=-0.55 | 5.0 / 5.5 m | 3 s hold, forward crossing | 5 | 5/5 succeeded |
| B | one pedestrian at 2.7 m/s, X=-0.30 | 6.0 / 6.5 m | 7 s hold, reverse crossing | 5 | 5/5 succeeded |
| C | 3.0 m/s main pedestrian plus 2.55 m/s second pedestrian | 7.0 / 7.5 m | 7 s and 9 s holds, opposite crossings at X=-0.55 and X=0.10 | 5 | 5/5 succeeded |

## Aggregate measurements

| Group | Mean stop duration | Stop duration range | Mean odometry displacement | Mean final Gazebo error | Maximum final Gazebo error |
| --- | ---: | ---: | ---: | ---: | ---: |
| A | 2.689 s | 1.312–3.829 s | 2.196 m | 0.181 m | 0.221 m |
| B | 5.212 s | 4.383–6.309 s | 2.194 m | 0.165 m | 0.209 m |
| C | 5.209 s | 1.201–7.110 s | 2.191 m | 0.168 m | 0.191 m |

## Bugs found while expanding the matrix

- One A run recovered from the pedestrian but then drifted laterally and
  timed out. Its Gazebo pose had not been reset immediately before the goal.
  Re-running the same seed after deterministic pose reset succeeded.
- One B run completed without stopping because the 2.7 m/s pedestrian left
  the crossing before the vehicle arrived. Increasing the hold from 5 s to
  7 s produced a repeatable stop/recovery and succeeded with the same seed.
- Repeated Gazebo launches sometimes lost the response to the second-model
  spawn request even though the model was created. The script now retries and
  accepts both a confirmed spawn and an already-existing entity.
- A controller action can report success from TF/wheel odometry while the
  Gazebo world pose is still short of the goal. The batch now independently
  checks the final Gazebo pose and rejects this as `WORLD_GOAL_MISS`.
- The experiment originally used the wrong Gazebo service and model-state
  topic names. Vehicle reset now uses `/set_entity_state` and verifies through
  `/model_states` before every goal.

## Result directories

- Initial A/B matrix: `batch-headless-cruise-20261007-002109`
- Corrected A seed 7105: `batch-headless-cruise-20261007-095617`
- Corrected B seed 7204: `batch-headless-cruise-20261007-095916`
- Final C matrix: `batch-headless-cruise-20261007-094122`

The valid result set is A runs 1–4 plus corrected seed 7105, B runs 1–3 and 5
plus corrected seed 7204, and all five runs from the final C matrix.
