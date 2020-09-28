---
layout: page
---

# A.3 hovercraft Model

<p> The hovercraft is one of the models used in testing the 3D scenarios. The \( x \), \( y \), \( z \) and \( \theta \) states are the same as the car. A fourth state is added to allow the car to hover. The kinematics for the hovercraft are given by  </p>

<div style="text-align:center"> \[ \dot{q} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & 0 & 0 \\ \sin(\theta) & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1  \end{bmatrix} \begin{bmatrix} v \\ v_z \\ \omega \end{bmatrix} \] </div>

<p> where \( v \) is the velocity in the xy-plane, \( v_z \) is the velocity along the z-axis, and \( \omega \) is the rate of turning. When a reference trajectory is introduced, the error states are given by </p>

<div style="text-align:center">
\[ e_x = \cos(\theta) (x_{ref} - x) + \sin(\theta) (y_{ref} - y) \]
\[ e_y = -\sin(\theta) (x_{ref} - x) + \cos(\theta) (y_{ref} - y) \]
\[ e_z = z_{ref} - z \]
\[ e_{\theta} = \theta_{ref} - \theta \]
</div>

The following Lyapunov function is proposed:

<div style="text-align:center"> \[ V = \frac{1}{2}(e_x^2 + e_y^2 + e_z^2) + \frac{1 - \cos(e_{\theta})}{k_2} \] </div>

with the time derivative

<div style="text-align:center"> \[ \dot{V} = -k_1 e_x^2 - k_4 e_z^2 - \frac{v_{ref} k_3 \sin^2(e_{\theta})}{k_2} \] </div>

<p> This time derivative is negative semi-definite when \( k_1, k_2, k_3, k_4 > 0 \) and the control law is given by: </p>

<div style="text-align:center">
\[ v = v_{ref} \cos(e_{\theta}) + k_1 e_x \]
\[ \omega = \omega_{ref} + v_{ref} (k_2 e_y + k_3 \sin(e_{\theta})) \]
\[ v_z = v_{z,{ref}} + k_4 e_z \]
</div>

<p> We can also see that the term \( \frac{1 - \cos(e_{\theta})}{k_2} \in [0, \frac{2}{k_2}] \) for any \( e_{\theta} \). </p>
