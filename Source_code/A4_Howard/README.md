# SWARMBUG with Howard's

This page explains how we implement the core component (i.e., computing the delta value) for the algorithm, including code snippets.

- The [code subfolder](./code) contains the source code of the modified swarm system, including
  - (1) the implementation of Swarmbug,
  - (2) modifications we made on the simulator/emulator to integrate Swarmbug into the algorithm.

## Quick start

- **Dependency issues:** Swarmbug does not require additional dependencies to the original algorithm. If you encounter dependency issues, please find relevant information (e.g., trouble-shooting regarding the dependencies) from the original Github repository (see the references in the paper).
- **Command line:** The following process will execute our Swarmbug prototype.

  1. Run the matlab
  2. Run `ccc_fct_dcc_Test_Swarm_3d_indepWaypoints.m`

- **Output:** Dcc values will be stored in `dcc.csv`under the root folder (where Matlab runs). Also, other outputs for analysis are the same.

## Summary of changes

- Creating alternative executions to compute Dcc

In `ccc_fct_dcc_Test_Swarm_3d_indepWaypoints.m`, `GradientDescentUpdate()` is called every tick in the main loop.

- `drones(i).pos` is the result of the original execution (Figure 5-(a)).
- `fake_coor_o` is the result of the counterfactual execution (Figure 5-(b)) without obstacle.

`Delta_o` is delta (Figure 5-(c)) for computing `dcc_o` (Dcc value).

```matlab
...
while( done == 0 ) %main loop
...
% the original execution
drones(i).pos = drones(i).pos + GradientDescentUpdate(drones(i).pos, i, drones, obst, pt );
...
  drones_fake = drones;
  obst_fake = obst;
  temp_o_pos = obst(1).pos;
   obst_fake(1).pos = [999; 999; 999]; % removing the obstacle

  % the counterfactual execution
  fake_coor_o = previous_pos + GradientDescentUpdate(drones_fake(1).pos, 1, drones_fake, obst_fake, pt_1);
  obst_fake(1).pos = temp_o_pos;
   drones_fake(1).pos = temp_1_pos;
...
delta_o = norm(drones(1).pos - fake_coor_o);
...
dcc_o = delta_o / sum_delta;
...

```
