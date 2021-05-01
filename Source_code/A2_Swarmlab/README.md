# SWARMBUG with Swarmlab

This page explains how we implement the core component (i.e., computing the delta value) for the algorithm, including code snippets.

- The [code subfolder](./code) contains the source code of the modified swarm system, including
  - (1) the implementation of Swarmbug,
  - (2) modifications we made on the simulator/emulator to integrate Swarmbug into the algorithm.

## Quick start

- **Dependency issues:** Swarmbug does not require additional dependencies to the original algorithm. If you encounter dependency issues, please find relevant information (e.g., trouble-shooting regarding the dependencies) from the original Github repository (see the references in the paper).
- **Command line:** The following process will execute our Swarmbug prototype.

  1. Run the matlab
  2. Type the below command in command window

  ```
  >> GUI_swarm
  ```

  3. Select `Olfati-Saber` in `Swarming Algorithm` and run the simulation.

- **Output:** Dcc values will be stored in `dcc.csv`under the root folder (where Matlab runs). Also, other outputs for analysis are the same.

## Summary of changes

- Creating alternative executions to compute Dcc

In `example_olfati_saber.m`, `swarm.fake_get_pos_ned()` is called every tick in the main loop. `N` is the result from original execution, and `fake_N` is the result from the counterfactual execution without the objects. `temp_norm` is the delta calculated by using the two values (`N` and `fake_N`).

```matlab
...
for time = p_sim.start_time:p_sim.dt:p_sim.end_time
...
   N = swarm.get_pos_ned(); %original execution
   fake_N = swarm.fake_get_pos_ned(); %counterfactual execution
   fake_d_1_pos_ned = swarm.fake_drones(1).pos_ned;

   main_agent = 2;
...
   % calculation delta for dcc
   temp_norm = sqrt((N(1:1, main_agent) - fake_N(1:1, main_agent))^2 ...
       + (N(2:2, main_agent) - fake_N(2:2, main_agent))^2 ...
       + (N(3:3, main_agent) - fake_N(3:3, main_agent))^2 );
...
```

In `Swarm.m`, `fake_get_pos_ned` refers `fake_drone.pos_ned` that is calculated in `Drone.m`. `Drone.m` requires `vel_command` to compute `fake_drone.pos_ned` and `vel_command` is decided in `compute_vel_olfati_saber.m`.
We made the modified version of `compute_vel_olfati_saber.m`: `fake_compute_vel_olfati_saber.m` that calculate the counterfactual execution by removing the objects.

In `fake_compute_vel_olfati_saber.m`, by making `dist_ab` big, following predicate is deactivated so the obstacle is not considered as a close one (hence, ignored), which functions as an essential part to affect the `command_vel`.

```matlab
% process of other agents
...
% process of obstacles
if p_swarm.is_active_cyl

   for obs = 1:p_swarm.n_cyl
       c_obs = p_swarm.cylinders(1:2, obs);
       r_obs = p_swarm.cylinders(3, obs);

       x_alpha = X(1:2, agent);
       v_alpha = V(1:2, agent);

       % Compute distance agent(a)-obstacle(b)
       dist_ab = sqrt(sum((X(1:2, agent) - c_obs).^2)) - r_obs;
       nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

       if dist_ab < min_dist_obs
           min_dist_obs = dist_ab;
       end

       if agent == main_agent
           if dist_ab < p_swarm.r0 && dist_ab > (p_swarm.r0 - 1) && obs == 6
...
           end
           if contribution_mode == 3
               if obs == target_obst
                   dist_ab = 9999; % so that next if predicate is not activated.
                   %disp('target_obst has gone far away');
               end
           end
       end

       if (dist_ab < p_swarm.r0)

           s = r_obs / (r_obs + dist_ab);
           x_beta = s * x_alpha + (1 - s) * c_obs;

           s_dot = r_obs * (v_alpha' * (x_beta - x_alpha) / dist_ab) / (r_obs + dist_ab)^2;
           v_beta = s * v_alpha - r_obs * (s_dot / s) * (x_beta - x_alpha) / dist_ab;
           x_gamma = c_obs + p_swarm.lambda * fake_u_ref(1:2);
           d_ag = norm(x_gamma - X(1:2, agent));

           a_obs(1:2, agent) = a_obs(1:2, agent) + ...
               +p_swarm.c_pm_obs * rho(dist_ab / p_swarm.r0, p_swarm.delta, p_swarm.r, p_swarm.k) * ...
               (phi(dist_ab - p_swarm.d_ref) * (x_beta - x_alpha) / dist_ab + ...
               phi(d_ag - p_swarm.d_ref) * (x_gamma - x_alpha) / (norm(x_gamma - x_alpha))) + ...
               p_swarm.c_vm_obs * (v_beta - v_alpha);
       end
   end
end

```
