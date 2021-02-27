# Profiling for the configuration definitions

This page explains how to profile **[configuration definitions](https://github.com/swarmbug/src/tree/main/Input_Swarmbug)**, especially an approach to identify `varnothing` for environment configuration (4.1.1 Configuration variables).

We present how we find a value for an environment configuration variable that eliminates the causal impact of the variable. We observe that environment configuration variables have stable delta values on the lower and higher sides of mutated values.

Intuitively, this is because the lower and higher sides values of the configuration variables essentially move the object associated with the variable far away from the swarm.

![](https://github.com/swarmbug/src/blob/main/Profiling_conf_def/fig/profiling.PNG)

- Finding a concrete value of `varnothing`.
  Moving an object to the yellow area near the drone cause a crash (i.e., an invalid test). We remove them from our analysis.

(b) shows computed delta values at each coordinate (x and y), and (a) presents two examples of tested values (_T<sub>1</sub>_ and _T<sub>2</sub>_) for an obstacle. Note that _O<sub>org</sub>_ is the obstacle in the original execution which does not have an impact on the drone under test because it exists far from the drone. The silver arrow essentially represents the flight path in the original execution.

We run a number of tests to cover most of the coordinates. _T<sub>1</sub>_ and _T<sub>2</sub>_ show two representative cases. If we mutate the obstacle's coordinate to be close to the drone _T<sub>1</sub>_, it changes the drone's flight significantly, leading to a large delta value (_Delta<sub>1</sub>_). On the other hand, if we mutate the coordinate to be far from the drone _T<sub>2</sub>_, it does not change the drone's flight at all, leading to a "zero" delta (_Delta<sub>2</sub>_).

(b) presents the delta values on each coordinate.
Observe that the area near the _Delta<sub>2</sub>_ is all having the same delta values, which are 0, forming a flat area.
The coordinate value from such a flat area is essentially a value that can eliminate the impact of the environment configuration variable.
We call this value `varnothing`. Note that different configuration variable may have different `varnothing` values. Hence, we repeat the above process for each variable.
