# Swarmbug Implementation: Adaptive Swarm

This page explains how we implement the core component (i.e., computing the delta value) for the algorithm, including code snippets.

- The [code subfolder](./code) contains the source code of the modified swarm system, including
  - (1) the implementation of Swarmbug,
  - (2) modifications we made on the simulator/emulator to integrate Swarmbug into the algorithm.

## Quick start

- **Dependency issues:** Swarmbug does not require additional dependencies to the original algorithm. If you encounter dependency issues, please find relevant information (e.g., trouble-shooting regarding the dependencies) from the original Github repository (see the references in the paper).
- **Command line:** The following command line will execute our Swarmbug prototype.

```bash
$ python src/layered_planner_long_mission/planner.py test -k long
```

- **Output:** Dcc values will be stored in `ref_x.csv` under `seed_pool` folder and other outputs for analysis under `output` folder.

## Summary of changes

- Creating alternative executions to compute Dcc

In `planner.py`, This `contribution_other()` is called every tick in the main loop.

```python
for x in tqdm(range(MAX_TICK)):
...
   contribution_others(robots, rbt, 'for_followers', arg_target_index=1, simulation_tick=simul_tick, followers_sp=followers_sp, params=params, OBSTACLES=OBSTACLES, target_obs=target_obs)
...
```

In `new_tools.py`, we added an additional calculation part, `contribution_others()` to create the alternative executions without the objects using `delta_distance`.

The result of counterfactual execution is calculated by `m_original_new_local_planner()` and it is used for the calculation of delta with the result from `after_normal_cal_sp`.

```python
def contribution_others(robots, robot1, leader_follower, arg_target_index, simulation_tick, followers_sp, params, OBSTACLES, target_obs):
...
'''For other drones 1 -> 2 -> 3 '''
...

'''For obstacle 1'''
followers_sp_prime = followers_sp
self_sp_global_prime = robots[target_index].sp_global
obstacles_prime = []
obs_onlyprime = copy.deepcopy(OBSTACLES)

for ext_p_idx in range(0, 4):
    # Remove the object
    obs_onlyprime[-obs_idx][ext_p_idx][0] = obs_onlyprime[-obs_idx][ext_p_idx][0] + delta_distance

robots_obstacles_sp_prime = [x for i, x in enumerate(
    followers_sp_prime + [robot1.sp]) if i != (target_index - 1)]
robots_obstacles_prime = poses2polygons(robots_obstacles_sp_prime)

obstacles_prime = np.array(obs_onlyprime + robots_obstacles_prime)
temp_sp.append(m_original_new_local_planner(
    self_sp, self_sp_global_prime, obstacles_prime, params)) # Counterfactual execution

'''For obstacle 2'''
...
# for obstacle 1, getting delta: after_normal_cal_sp is the normal execution
temp_max_obs_btt = np.linalg.norm(after_normal_cal_sp - temp_sp[4])
...
# for dcc value
temp_sum = temp_max_des + temp_max_1st + temp_max_2nd + temp_max_3rd + \
temp_max_obs_btt + temp_max_obs_dia + temp_max_obs_ltr + temp_wall
...
temp_dcc_obs_btt = temp_max_obs_btt / temp_sum
...

```
