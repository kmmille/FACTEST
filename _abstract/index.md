---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

## Abstract

FACTEST is tool for synthesizing controllers for nonlinear systems with reach-avoid requirements. The controllers found by FACTEST consists of a reference trajectory and a tracking controller which drives the actual trajectory to follow the reference trajectory.

In our [papers]({{site.baseurl}}{% link papers/index.html %}), we identify a type of reference trajectory such that the tracking error between the actual trajectory of the closed-loop system and the reference trajectory can be bounded and pre-computed. Moreover, such a bound on the tracking error is independent of the reference trajectory. Using such bounds on the tracking error, we propose a method that can find a reference trajectory by solving a satisfiability problem over linear constraints. Our overall algorithm guarantees that the resulting controller can make sure every trajectory from the initial set of the system satisfies the given reach-avoid requirement.
