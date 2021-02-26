# Input example

This page explains the input of Swarmbug.
Swarmbug expects a user to provide the configuration definitions: A list of configuration variables with each variable's type (environment or swarm configuration) and the value specification.

## Environment configuration variables

In the case of environment configuration variables, the value to eliminate the corresponding impact of the variable (i.e., `varnothing`) is required.

Below data is the real input when Swarmbug is applied to Adaptive Swarm. For example, `obstacles[8].sp` is the coordinates of the first moving obstacle. As the coordinate that every object in the world can have is from -2.5 to 2.5 for x-axis and from -2.5 to 2.5 for y-axis, when `obstacles[8]` has a bigger value than [3.0, 3.0], it cannot affect any object in the world.

```json
{
    "Environment configuration"   : {

        "robot1.sp"        : {
            "Varnothing"   : [3.0, 3.0]
        },
        "followers[0].sp"  : {
            "Varnothing"   : [3.0, 3.0]
        },
        "followers[1].sp"  : {
            "Varnothing"   : [3.0, 3.0]
        },
        "followers[2].sp"  : {
            "Varnothing"   : [3.0, 3.0]
        },
        "obstacles[8].sp"  : {
            "Varnothing"   : [3.0, 3.0]
        },
        "obstacles[9].sp"  : {
            "Varnothing"   : [3.0, 3.0]
        },
        "obstacles[10].sp" : {
            "Varnothing"   : [3.0, 3.0]
        }
    },
},
...
```

## Swarm configuration variables

Below data is the real input as swarm configuration variables. In this type of variable, unlikely environment configuration variables, "Default", "Min", and "Max" are required. Note that "Varnothing" is not needed for this type of variable (See 4.1. Behavior causal analysis).

"Default" means the original value of the swarm configuration variable, which becomes a starting point of mutation. "Min" and "Max" are the minimum and maximum values, respectively. Generally, as swarm configuration variables have the real physical meaning, setting for the feasible range is important.

For example, `drone_vel` means the velocity of the drone. The physical drone cannot move with negative velocity, and it cannot fly with a bigger velocity than its maximum velocity (given by physical restriction).

```json
{
  "Swarm configuration": {
    "w": {
      "Default": 20.0,
      "Min": 0.0,
      "Max": 40.0
    },
    "xi": {
      "Default": 400.0,
      "Min": 0.0,
      "Max": 800.0
    },
    "nu": {
      "Default": 0.0014,
      "Min": 0.0,
      "Max": 0.0028
    },
    "int_dist": {
      "Default": 0.7,
      "Min": 0.0,
      "Max": 1.4
    },
    "infl_radius": {
      "Default": 0.3,
      "Min": 0.0,
      "Max": 0.6
    },
    "drone_vel": {
      "Default": 4.0,
      "Min": 0.0,
      "Max": 8.0
    }
  }
}
```

Above `json` file can be found in **[Sample Input](https://github.com/swarmbug/src/tree/main/Input_example/Sample_data)**.
