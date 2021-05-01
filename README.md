# Contents

This is the Swarmbug project page.
Contents that we was not able to cover in the paper due to the paper limit are placed here.

## Source code

This page contains Swarmbug's source code and additional profiling tools mentioned in the paper (with example inputs/data).

More details in **[Source code](https://github.com/swarmbug/src/tree/main/Source_code)**

## Profiling tools and supporting approaches

This folder contains tools we created to assist data analysis and processing tasks.

More details in **[Profiling tools and supporting approaches](https://github.com/swarmbug/src/tree/main/Source_code/Tools)**

## Criteria for selecting algorithms

This page contains our swarm algorithm selection criteria in detail, which is mentioned in Section 5. Evaluation, Swarm Algorithms.

More details in **[Criteria for selecting algorithms](https://github.com/swarmbug/src/tree/main/Criteria_for_selecting_algorithms)**

## Input data of Swarmbug.

We explains configuration variables with the real input data of Swarmbug when we apply Swarmbug to Adaptive Swarm. Below data is a part of input file.

```json
{
    "Environment configuration"   : {

        "robot1.sp"               : {
            "Varnothing"          : [3.0, 3.0]
        },
        ...
    },

    "Swarm configuration"         : {
        "w"                       : {
            "Default"             : 20.0,
            "Min"                 : 0.0,
            "Max"                 : 40.0
        },
        ...
    }
}
```

More details in **[Input_Swarmbug](https://github.com/swarmbug/src/tree/main/Input_Swarmbug)**.

## Profiling the configuration definitions

We present how to profile configuration definitions, especially an approach to identify `varnothing` for environment configuration (4.1.1 Configuration variables) here.

![](https://github.com/swarmbug/src/blob/main/main_4_fig1_pro.png)

By mutating variables we eliminate impacts of configuration variables: from the environment (left figure) we extract the delta values (right figure), the coordinates (e.g., `x` and `y` values) that reach the plateua becomes `varnothing`.

More details in **[Profiling_conf_def](https://github.com/swarmbug/src/tree/main/Profiling_conf_def)**.

## Identifying the fixed point in computing spacial variation.

In the paper (4.2.1 Profiling sptial variations), we reach the fixed point with 10 test sets (100 tests). We explain how to get this fixed point here.

![](https://github.com/swarmbug/src/blob/main/main_5_fig1_converged.png)

When we observe _SV<sub>i-1</sub>_ and _SV<sub>i</sub>_ do not differ more than 5%, we stop repeating the process.

More details in **[Fixed_pt_SVMap](https://github.com/swarmbug/src/tree/main/Fixed_pt_SVMap)**.

## Profiling the threshold for the time window

We explain how to profile the threshold for the time window (4.1.3 Temporal analysis).

![](https://github.com/swarmbug/src/blob/main/main_6_fig1_whole.png)

![](https://github.com/swarmbug/src/blob/main/main_6_fig2_zoom.png)

These figures are the whole dcc value of follower 1 (_m<sub>2</sub>_) and the enlarged part for _T<sub>win<sub>3</sub></sub>_ and _T<sub>win<sub>4</sub></sub>_.

We observe _T<sub>win<sub>3</sub></sub>_ and _T<sub>win<sub>4</sub></sub>_ are time delay between stable flight status in above figure.

More details in **[Profiling_thr_time_win](https://github.com/swarmbug/src/tree/main/Profiling_thr_time_win)**.

## Physical experiments.

We checked proposed patch by Swarmbug performed well in real would experiment while naive patch did not.

![](https://github.com/swarmbug/src/blob/main/main_3_fig1_phy.png)

Left figure shows drones are crash with each other (naive patch) and the right figure shows drones are flying without crash.

More details in **[Physical_experiments](https://github.com/swarmbug/src/tree/main/Physical_experiments)**.

## Finding algorithm's defect

We observed algorithm's defect by fuzzing.

![](https://github.com/swarmbug/src/blob/main/main_1_fig1_crash.png)

Leader (red drone) does not consider other drones.
As a result, it crashes with follower (blue drone) after it avoids obstacle (red polygon).
This case that follower drone is placed in front of leader drone is rare in normal running but we observe while fuzzing.

More details in **[Finding_algorithms_defect](https://github.com/swarmbug/src/tree/main/Finding_algorithms_defect)**.

## Exposing buggy logic via Dcc

Using Dcc value, we can trace back to code and improve swarm's behavior by modifying code recognized.

![](https://github.com/swarmbug/src/blob/main/main_2_fig1_dcc.png)

only `obst_pot_c` (the right most figure) shows meaningful change of dcc value while the others don't.

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/videos/no_fix.gif)

After modifying the code related to `obst_pot_c`, we observe drones (blue sphere) avoid the obstacle (red sphere).

More details in **[Exposing_buggy_logic_via_Dcc](https://github.com/swarmbug/src/tree/main/Exposing_buggy_logic_via_Dcc)**.
