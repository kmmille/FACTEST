---
layout: page
---

## Results Gallery
# Example reference and actual trajectories using controllers found by FACTEST

FACTEST can find controllers different vehicle models (3-6 dimensional state space and 2-4 dimensional input space), including ground, aerial, and underwater vehicles, across eight scenarios (with up to 22 obstacles), all with running time at the sub-second range.

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/car_zigzag1.png"></td>
    <td><img src="{{site.baseurl}}/figs/car_zigzag2.png"></td>
    <td><img src="{{site.baseurl}}/figs/car_zigzag3.png"></td>
  </tr>
  <tr>
    <td>Example solution to zigzag1</td>
    <td>Example solution to zigzag2</td>
    <td>Example solution to zigzag3</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/car_maze.png"></td>
    <td><img src="{{site.baseurl}}/figs/robot_barrier.png"></td>
  </tr>
  <tr>
    <td>Example solution to maze</td>
    <td>Example solution to barrier</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/auv_ltunnel.png"></td>
    <td><img src="{{site.baseurl}}/figs/auv_ztunnel.png"></td>
  </tr>
  <tr>
    <td>Example solution to L-tunnel</td>
    <td>Example solution to Z-tunnel</td>
  </tr>
</table>

| Model      | Env      | Initial Set Size | # Obstacles | Time  | # Partitions |
|:----------:|:--------:|:----------------:|:-----------:|:-----:|:------------:|
| car        | zigzag1  | 0.200            | 9           | 0.028 | 1            |
| car        | zigzag2  | 0.400            | 9           | 0.144 | 4            |
| car        | zigzag3  | 0.600            | 9           | 0.605 | 16           |
| car        | maze     | 0.200            | 22          | 0.078 | 1            |
| car        | barrier  | 0.707            | 6           | 0.415 | 22           |
| car        | SCOTS    | 0.070            | 19          | 0.667 | 1            |
| robot      | zigzag1  | 0.200            | 9           | 0.025 | 1            |
| robot      | zigzag2  | 0.400            | 9           | 0.196 | 4            |
| robot      | zigzag3  | 0.600            | 9           | 0.612 | 16           |
| robot      | maze     | 0.200            | 22          | 0.079 | 1            |
| robot      | barrier  | 0.707            | 6           | 0.498 | 22           |
| robot      | SCOTS    | 0.070            | 19          | 0.635 | 1            |
| auv        | Z-tunnel | 0.866            | 6           | 0.372 | 1            |
| auv        | L-tunnel | 0.866            | 8           | 0.317 | 1            |
| hovercraft | Z-tunnel | 0.866            | 6           | 0.472 | 1            |
| hovercraft | L-tunnel | 0.866            | 8           | 0.140 | 1            |

# Reference trajectory found by FACEST vs. RRT

To demonstrate the speed of our SMT-based reference trajectory planner, we compare it with [Rapidly-exploring Random Trees (RRT)](http://msl.cs.uiuc.edu/rrt/about.html). The running time, number of line segments, and number of iterations needed to find a path were compared. RRT was run using the [Python Robotics library](https://github.com/AtsushiSakai/PythonRobotics), which is not necessarily an optimized implementation. SAT-Plan was run using [Yices SMT Solver](https://yices.csl.sri.com/).

Each planner was run 100 times. The colored bars represent the average runtime and average number of iterations. The error bars represent the range of minimum and maximum. RRT timed out for the SCOTS scenario, unable to find a trajectory within 5000 iterations. The maze scenario timed out about 10% of the time. Overall SAT-Plan scales in time much better as the size of the unsafe set increases. Additionally, the maximum number of iterations that RRT had to perform was far greater than the average number of line segments needed to find a safe path.

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_easy.png"></td>
    <td><img src="{{site.baseurl}}/figs/rrt_easy.png"></td>
  </tr>
  <tr>
    <td>FACTEST on easy scenario</td>
    <td>RRT on easy scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_hard.png"></td>
    <td><img src="{{site.baseurl}}/figs/rrt_hard.png"></td>
  </tr>
  <tr>
    <td>FACTEST on hard scenario</td>
    <td>RRT on hard scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_zigzag.png"></td>
    <td><img src="{{site.baseurl}}/figs/rrt_zigzag.png"></td>
  </tr>
  <tr>
    <td>FACTEST on zigzag scenario</td>
    <td>RRT on zigzag scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_part2.png"></td>
    <td><img src="{{site.baseurl}}/figs/rrt_part2.png"></td>
  </tr>
  <tr>
    <td>FACTEST on maze scenario</td>
    <td>RRT on maze scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_maze.png"></td>
    <td><img src="{{site.baseurl}}/figs/rrt_maze.png"></td>
  </tr>
  <tr>
    <td>FACTEST on maze scenario</td>
    <td>RRT on maze scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/smt_SCOTS.png" width="367px"></td>
    <td>TIME OUT</td>
  </tr>
  <tr>
    <td>FACTEST on SCOTS scenario</td>
    <td>RRT timed out on SCOTS scenario</td>
  </tr>
</table>

<table style="text-align:center; width:100%">
  <tr>
    <td><img src="{{site.baseurl}}/figs/all_time_comparison.png"></td>
    <td><img src="{{site.baseurl}}/figs/all_iter_comparison.png"></td>
  </tr>
  <tr>
    <td>RRT vs. SAT-plan time comparison</td>
    <td>RRT vs. SAT-plan iteration comparison</td>
  </tr>
</table>
