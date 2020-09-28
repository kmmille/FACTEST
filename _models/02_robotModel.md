---
layout: page
---

# Appendix A from <i>Fast and Guaranteed Safe Controller Synthesis</i>
# A.1 Robot Model

The robot is one of the models used in testing the 2D scenarios. The kinematics for a robot are given by

<div style="text-align:center"> \[ \dot{q} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & 0 \\ \sin(\theta) & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix} \] </div>

<p> The model can be made bijective by using the states \( s = \sin(\theta) \) and \( c = \cos(\theta) \) in place of \( \theta \). The kinematic equation becomes </p>

<div style="text-align:center"> \[ \dot{q} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{s} \\ \dot{c} \end{bmatrix} = \begin{bmatrix} c & 0 \\ s & 0 \\ 0 & c \\ 0 & s \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix} \] </div>

When a reference trajectory is introduced, the error states are given by

<div style="text-align:center">
\[ e_x = c(x_{ref} - x) + s(y_{ref} - y) \]
\[ e_y = -s(x_{ref} - x) + c(y_{ref} - y) \]
\[ e_s = \sin(\theta_{ref} - \theta) = s_{ref}c - c_{ref}s \]
\[ e_c = \cos(\theta_{ref} - \theta) = c_{ref}c + s_{ref}s -1 \]
</div>

From [5](https://www.sciencedirect.com/science/article/pii/S0921889011001023?casa_token=QRzosI8VUdoAAAAA:0C4GTrNm3QpxABHRRQ3ZGcEL8ClJ4J98cOLzTaryxA1CVYR85Y1OF2MH7TGNc7gdSgcmniO1QiE), the following Lyapunov function is proposed:

<div style="text-align:center"> \[ V = \frac{k}{2} (e_x^2 + e_y^2) + \frac{1}{2(1 + \frac{e_c}{a})} (e_s^2 + e_c^2) \] </div>

<p> where \( k > 0 \) and \( a > 0 \) are constants. The range of \( e_c \) is \( [-2, 0] \) and therefore \( 0 < \frac{a - 2}{a} \leq 1 + \frac{e_c}{a} \leq 1 \) and \( 1 \leq \frac{1}{1 + \frac{e_c}{a}} \leq \frac{a}{a-2} \) . The Lyapunov function has the derivative </p>

<div style="text-align:center"> \[ \dot{V} = -ke_x v_b + e_s \left( k v_{ref} e_y - \frac{\omega_b}{(1 + \frac{e_c}{a})^2} \right) \] </div>

which is negative semi-definite with the control law

<div style="text-align:center">
\[ v_b = k_x e_x \]
\[ \omega_b = k v_{ref} e_y (1 + \frac{e_c}{a})^2 + k_s e_s \left[ \left( 1 + \frac{e_c}{a} \right)^2 \right]^n \]
</div>

<p> It can be checked that \( e_s^2 + e_c^2 = -2 e_c \in [0, 4] \). The term \( \frac{1}{2(1 + \frac{e_c}{a})}(e_s^2 + e_c^2) \) in \( V \) can also be bounded with \( \frac{1}{2(1 + \frac{e_c}{a})}(e_s^2 + e_c^2) \in [2, \frac{2a}{a-2}] \). </p>
