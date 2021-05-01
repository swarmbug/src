# Finding algorithm's defect

During the fuzz-testing on the Adaptive swarm, we find a rare bug in the algorithm.
That is, when a follower drone is placed between the leader and the obstacle, it leads to a crash to the leader. Our manual analysis (with domain knowledge) shows that the leader drone's algorithm does not consider follower drones as an object to avoid. This is odd because follower drones do have the logic to avoid the leader drone if it gets too close.
We suspect that the developer may not consider a scenario where a leader drone is behind other drones. This is because, in part, such a scenario rarely happens during simulations. However, Swarmbug's extensive fuzz-testing identified such a scenario and exposed the defect.

Later, we carefully introduce some randomness during the testing and find out that such a scenario is possible in the real-world (e.g., when a strong wind holds the leader drone back).
More details are as follows.

## Rare case found while fuzzing

### Case 1

This is the crash case of patched version of Adaptive Swarm by Swarmbug (with _Infl_radius_ \* 2.0)

- red: leader drone
- blue, brown, and yellow: follower 1,2, and 3, respectively
- black square: moving obstacle
- green big circle: goal

In this situation, follower 1 crashes with the leader drone.

1. Leader drone moves down to avoid the obstacle, but follower 1 is already there because it decided to avoid the leader drone.
2. Follower 1 is placed between the leader drone and the wall. Both are objectives to avoid for follower 1.
3. Leader drone goes to its way: to its goal avoiding obstacle without considering follower 1.
4. Follower 1 tries to avoid both but leader drone push its way, then they crash.

![](https://github.com/swarmbug/src/blob/main/Finding_algorithms_defect/video/case1/case_1.gif)

![](https://github.com/swarmbug/src/blob/main/Finding_algorithms_defect/video/case1/sc_1.png)

In above figure, transparent images reflect the real size of drones and obstacles.
Leader (red drone) crashes with follower 1 (blue drone) avoiding an obstacle (red polygon).

### Case 2

Below screenshots are from the similar case.

In this situation, follower 1 is placed between leader and obstacle, and leader does not consider follower 1 pushing its way. As a result, follower 1 and obstacle crash.
Note that they are spawned with enough distance between each other (bigger than safety distance).
Figures are placed in order of time from the left.
![](https://github.com/swarmbug/src/blob/main/Finding_algorithms_defect/video/case2/combined.png)
(This situation is too short to be shown, screenshot is used.)

![](https://github.com/swarmbug/src/blob/main/Finding_algorithms_defect/video/case2/sc_2.png)

In above figure, transparent images reflect the real size of drones and obstacles. Follower 1 (blue drone) crashes with an obstacle (red polygon) avoiding leader (red drone). Follower 1 is also close to leader because leader is approaching toward follower 1.

There are more cases (similar ones) that can be found while fuzzing.

## The reason that can be found in code

### code for followers

This code is the part that how followers use `local_planner()`.
`robot1.sp` is the coordinates of the leader.
We can find that followers include leader as obstacles in second line: `robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp]) if i!=p]`.
Then, it is merged with original obstacle (`obstacles`) and becomes `obstacles1` that is input for `local_planner()`.

```python
for p in range(len(followers_sp)): # formation poses correction with local planner
        # robots repel from each other inside the formation
    robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp]) if i!=p] # all poses except the robot[p]
    robots_obstacles = poses2polygons( robots_obstacles_sp ) # each drone is defined as a small cube for inter-robots collision avoidance
    obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
    # follower robot's position correction with local planner
    robots[p+1].local_planner(obstacles1, params)
    followers_sp[p] = robots[p+1].sp
```

### code for leader

However, below code, in case of leader, it has only one line for `local_planner()`.

```python=
robot1.local_planner(obstacles, params)
```

Here, `obstacles` does not include any other drones.
