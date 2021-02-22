# Contents

This is the Swarmbug project page.
Contents that we was not able to cover in the paper due to the paper limit are placed here.

## Supplementary materials for Finding algorithm's defect

We observed algorithm's defect by fuzzing.

![](https://github.com/swarmbug/swarmbug/blob/main/main_1_fig1_crash.png)

Leader (red drone) does not consider other drones.
As a result, it crashes with follower (blue drone) after it avoids obstacle (red polygon).
This case that follower drone is placed in front of leader drone is rare in normal running but we observe while fuzzing.

More details in **[Finding_algorithms_defect](https://github.com/swarmbug/src/tree/main/Finding_algorithms_defect)**.

## Supplementary materials for Exposing buggy logic via Dcc

Using Dcc value, we can trace back to code and improve swarm's behavior by modifying code recognized.

![](https://github.com/swarmbug/swarmbug/blob/main/main_2_fig1_dcc.png)

only `obst_pot_c` (the right most figure) shows meaningful change of dcc value while the others don't.

![](https://github.com/swarmbug/swarmbug/blob/main/Exposing_buggy_logic_via_Dcc/videos/no_fix.gif)

After modifying the code related to `obst_pot_c`, we observe drones (blue sphere) avoid the obstacle (red sphere).

More details in **[Exposing_buggy_logic_via_Dcc](https://github.com/swarmbug/src/tree/main/Exposing_buggy_logic_via_Dcc)**.

## Supplementary materials for Physical experiments.

We checked proposed patch by Swarmbug performed well in real would experiment while naive patch did not.

![](https://github.com/swarmbug/swarmbug/blob/main/main_3_fig1_phy.png)

Left figure shows drones are crash with each other (naive patch) and the right figure shows drones are flying without crash.

More details in **[Physical_experiments](https://github.com/swarmbug/src/tree/main/Physical_experiments)**.
