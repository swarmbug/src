# Profiling the threshold for the time window

This page explains how to profile the threshold for the time window.
<sub></sub>

In the paper, we identify when the current Dcc value is changed more than 10% than its previous tick's Dcc value (i.e., Dcc value is rapidly changing). 10 % threshold is configurable and we explain how we get a time window by using this threshold.

In _M<sub>k</sub>_ (k <sup>th</sup> mission), for _m<sub>r</sub>_, we collect _T<sub>win<sub>i</sub></sub>_ then take an average of them.

- Definitions of terms are described in 4.1.3 Temporal analysis.

![](https://github.com/swarmbug/src/blob/main/Profiling_thr_time_win/fig/dcc_whole.PNG)

- This is the whole dcc value of follower 1 (_m<sub>r</sub>_), which is one of the real data.
  - Legend for this figure is the same as figure 6 in the paper.

In this data, we can get 6.67 tick as a time window.
When we aggregate all drones (_m<sub>r</sub>_) and consider more missions (_M<sub>k</sub>_), this is converged into 7.6 tick.

![](https://github.com/swarmbug/src/blob/main/Profiling_thr_time_win/fig/zoom.PNG)

- This is partial figure for _T<sub>win<sub>3</sub></sub>_ and _T<sub>win<sub>4</sub></sub>_.

![](https://github.com/swarmbug/src/blob/main/Profiling_thr_time_win/fig/traj.PNG)

- This is swarm's flight snapshot that corresponds to the 2nd figure.

With domain knowledge, we observe ① follower 1 flies (blue drone in above figure, before _T<sub>win<sub>3</sub></sub>_) and ② approaches to wall (obstacle 3). Then ③ it flies next to wall (between _T<sub>win<sub>3</sub></sub>_ and _T<sub>win<sub>4</sub></sub>_) and ④ tries to turn around the corner of wall (_T<sub>win<sub>4</sub></sub>_). At last, ⑤ it flies away (after _T<sub>win<sub>4</sub></sub>_).
So, we observe _T<sub>win<sub>3</sub></sub>_ and _T<sub>win<sub>4</sub></sub>_ are time delay between stable flight status.

In this way, we can measure the _T<sub>win</sub>_. Note that 10% threshold can be tuned for each algorithm (10% works fine for four algorithms we used in this paper.)
