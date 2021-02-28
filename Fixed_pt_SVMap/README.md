# Identifying the fixed point in computing spacial variation

This page explains how to get the fixed when we repeat the process to define SVMap.

As mentioned in 4.2.1 Profiling sptial variations, On the _i<sup>th</sup>_ test set, we measure the spatial variation of the drones’ poses (_SV<sub>i</sub>_) from all the test runs executed at this point (_i_ ∗ 10 tests).
We repeat the process until we observe _SV<sub>i-1</sub>_ and _SV<sub>i</sub>_ do not differ more than 5%. In general, we reach the fixed point with 10 test sets, meaning that we run 100 tests in total.

![](https://github.com/swarmbug/src/blob/main/Fixed_pt_SVMap/fig/converged.PNG)

- Converged norm value of centroid and radius of 90% area.

(a) is the trend of norm value of centroid of 90% area of SVMap and (b) is for radius.
We observe it is converged to fixed point after 9 test sets (90 tests). Note that this threshold can be differ across algorithms.
