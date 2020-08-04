---
layout: page
title: Papers
permalink: /papers/
---

# Current Work
- [Fast and Guaranteed Safe Controller Synthesis]({{ site.baseurl }}{% link files/cav2020.pdf %}) -
 This paper is accepted to the International Conference on Computer-Aided Verification (CAV) 2020. In this paper, we use Lyapunov functions of the tracking error between the actual trajectory and the reference to bound the value of such tracking error. For reference controllers, we use SMT-encoding and SMT solvers to find the piece-wise linear reference trajectories.
 [[BibTex]]({{ site.baseurl }}{% link files/cav2020.txt %})

# Previous Work
- [Controller Synthesis Made Real: Reach-Avoid Specifications and Linear Dynamics](http://cfan10.web.engr.illinois.edu/wp-content/uploads/2018/08/CAV2018.pdf) (CAV2018) [[BibTex]]({{ site.baseurl }}{% link files/cav2018.txt %})

<!-- - [Fast Nonlinear Controller Synthesis Using Reachability Analysis]({{ site.baseurl }}{% link files/reachtube.pdf %}) -
This paper is submitted to Conference on Decision and Control [(CDC)](https://cdc2020.ieeecss.org/) 2020 and is currently under review. The major differences between this submission with the CAV 2020 version are 1) we use reachability analysis to find a much less conservative bound for the tracking error, and 2) we use MILP encoding and MILP solvers to find the piece-wise linear reference trajectories. When we submitted this paper to CDC 2020, the CAV version was still under review. Therefore, we did not explicitly compare the two versions in the paper. We plan to include more comparison results on this webpage. -->
