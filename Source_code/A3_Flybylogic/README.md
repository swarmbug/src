# SWARMBUG with Fly-by-Logic

This page explains how we implement the core component (i.e., computing the delta value) for the algorithm, including code snippets.

- The [code subfolder](./code) contains the source code of the modified swarm system, including
  - (1) the implementation of Swarmbug,
  - (2) modifications we made on the simulator/emulator to integrate Swarmbug into the algorithm.

## Quick start

- **Dependency issues:** Swarmbug does not require additional dependencies to the original algorithm. If you encounter dependency issues, please find relevant information (e.g., trouble-shooting regarding the dependencies) from the original Github repository (see the references in the paper).
  - Note that Fly-by-Logic requires **additional optimization libraries** for Matlab.
- **Command line:** The following process will execute our Swarmbug prototype.

  1.  Run the matlab and make sure essential packages (e.g., tbxmanager, casadi, etc.) are included for execution.
  2.  Run `bbb_Dcc_reach_avoid_Ndrones_varvel.m`

- **Output:** Dcc values will be stored in `delta.csv`under the root folder (where Matlab runs). Also, other outputs for analysis are the same.

## Summary of changes

- Creating alternative executions to compute Dcc

In `bbb_Dcc_reach_avoid_Ndrones_varvel.m`, `bbb_fct_Dcc_reach_avoid_Ndrones_varvel()` is called every tick in the main loop. This function computes the counterfactual execution by removing the objects:

- `[xx_1, yy_1, zz_1]` is the result of the original execution.
- `[xx_2, yy_2, zz_2]` and `[xx_o, yy_o, zz_o]` are the result of the counterfactual execution without 2nd agent and obstacle, respectively.

Then, they are computed together to get delta (`delta_about_a2` and `delta_about_o`).

In `bbb_Dcc_reach_avoid_Ndrones_varvel.m`,

```matlab
...
for current_t = 1:100 %main loop
...
[xx_1, yy_1, zz_1] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, current_xx, current_yy, current_zz, does_obstacle_exist); % result of original execution
...
modified_current_xx_2 = xx(current_t,:);
modified_current_yy_2 = yy(current_t,:);
modified_current_zz_2 = zz(current_t,:);
modified_current_xx_2(2) = 0; % removing the target object: 2nd agent
modified_current_yy_2(2) = 0;
modified_current_zz_2(2) = 0;
...
% the result of counterfactual execution (without 2nd agent)
[xx_2, yy_2, zz_2] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, modified_current_xx_2, modified_current_yy_2, modified_current_zz_2, does_obstacle_exist);
…
% the result of counterfactual execution (without obstacle)
[xx_o, yy_o, zz_o] = bbb_fct_Dcc_reach_avoid_Ndrones_varvel(N_drones, current_xx, current_yy, current_zz, 0);
...
delta_about_a2 = sqrt( (xx_1(desired_time,1) - xx_2(desired_time,1))^2 + (yy_1(desired_time,1) - yy_2(desired_time,1))^2 + (zz_1(desired_time,1) - zz_2(desired_time,1))^2 );
...
delta_about_o = sqrt( (xx_1(desired_time,1) - xx_o(desired_time,1))^2 + (yy_1(desired_time,1) - yy_o(desired_time,1))^2 + (zz_1(desired_time,1) - zz_o(desired_time,1))^2 );
...
```

In `bbb_fct_Dcc_reach_avoid_Ndrones_varvel.m`, we modified the original map data into an empty set (since it has only one ‘unsafe’ zone here) for removing the obstacle,

```matlab
if(obstacle_arg)
   map_name = 'Maps_mrsl/map_test_2.txt';
else
   map_name = 'Maps_mrsl/map_test_empty.txt';
end
...
```
