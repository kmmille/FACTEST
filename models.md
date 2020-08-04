---
layout: page
title: Vehicle Models
permalink: /models/
---
<script type="text/javascript"
        src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.0/MathJax.js?config=TeX-AMS_CHTML"></script>

The files for all the vehicle models can be found in the models folder on the [GitHub repository](https://github.com/kmmille/FACTEST).

# Car Model
The rearwheel kinematic car is one of the models used in testing the 2D scenarios. The dynamics for the car are given by the following.

<div style="text-align:center"> \[ \dot{q} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & 0 \\ \sin(\theta) & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} v \\ w \end{bmatrix} \] </div>

The error for the car are given by the following.

<div style="text-align:center"> \[ \begin{bmatrix} e_x \\ e_y \\ e_{\theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & \sin(\theta) & 0 \\ -\sin(\theta) & \cos(\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{ref} - x \\ y_{ref} - y \\ \theta_{ref} - \theta \end{bmatrix} \] </div>

The error dynamics are given by the following.

<div style="text-align:center"> \[ \begin{bmatrix} \dot{e}_x \\ \dot{e}_y \\ \dot{e}_{\theta} \end{bmatrix} = \begin{bmatrix} \omega e_y - v + v_{ref} \cos(e_{\theta}) \\ -\omega e_x + v_{ref} \sin(e_{\theta}) \\ \omega_{ref} - \omega \end{bmatrix} \] </div>

The following control law is proposed:

<div style="text-align:center"> \[ v = v_{ref} \cos(e_{\theta}) + k_1 e_x \]
\[ \omega = \omega_{ref} + v_{ref}(k_2 e_y + k_3 \sin(e_{\theta})) \] </div>

Substituting this control law, the error dynamics become:

<div style="text-align:center"> \[ \begin{bmatrix} \dot{e}_x \\ \dot{e}_y \\ \dot{e}_{\theta} \end{bmatrix} = \begin{bmatrix} (\omega_{ref} + v_{ref}(k_2 e_y + k_3 \sin(e_{\theta}))) e_y - k_1 e_x \\ -(\omega_{ref} + v_{ref}(k_2 e_y + k_3 \sin(e_{\theta}))) e_x + v_{ref} \sin(e_{\theta}) \\ \omega_{ref} - \omega \end{bmatrix} \] </div>

A candidate Lyapunov function for the system is given below.

<div style="text-align:center"> \[ V = \frac{1}{2} (e_x^2 + e_y^2) + \frac{1 - \cos(e_{\theta})}{k_2} \] </div>

The time derivative of this Lyapunov function is:

<div style="text-align:center"> \[ \dot{V} = -k_1 e_x^2 - \frac{v_{ref} k_3 \sin^2(e_{\theta})}{k_2} \] </div>

<p> The Lyapunov function is positive semi-definite and its time derivative is negative semi-definite, meaning it is a valid Lyapunov function. The \( \frac{1 - \cos(e_{\theta})}{k_2} \) term can be upper bounded by \( \frac{2}{k_2} \). Initially \( V(e(0)) \leq \frac{\ell^2}{2} + \frac{2}{k_2} \) where \( \ell^2 = e_x^2 + e_y^2 \). Then, for all time \( V(e(t)) \leq \frac{\ell^2}{2} + \frac{2i}{k_2} \) for the \( i^{th} \) segment. Therefore, the error of the car is upper bounded by \( \sqrt{( \ell^2 + \frac{4i}{k_2})} \) .</p>

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

# A.2 Autonomous Underwater Vehicle (AUV) Model

<p> The autonomous underwater vehicle (AUV) is one of the models used for testing the 3D scenarios. The position of the AUV is \( \textbf{x} = \begin{bmatrix} x & y & z \end{bmatrix}^T \), and the Euler angles (roll, pitch, and yaw respectively) are \( \boldsymbol{\theta} = \begin{bmatrix} \phi & \theta & \psi \end{bmatrix}^T \). The equations of motion for the position and Euler angles are given by </p>

<div style="text-align:center">
\[ \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix} = \begin{bmatrix} \cos{\psi} \cos{\theta} \\ \sin{\psi} \cos{\theta} \\ \sin{\theta} \end{bmatrix} v \]
\[ \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} =
   \begin{bmatrix} 1 & \sin{\phi} \tan{\theta} & \cos{\phi} \tan{\theta} \\
                   0 & \cos{\phi} & -\sin{\phi} \\
                   0 & \sin{\phi} \sec{\theta} & \cos{\phi} \sec{\theta} \end{bmatrix}
   \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix} \]
</div>

<p> The angular acceleration \( \boldsymbol{\omega} = \begin{bmatrix} \omega_x & \omega_y & \omega_z \end{bmatrix} \) is given in the local frame. By combining the equations of motion for the position and Euler angles, the total kinematis are given by  </p>

<div style="text-align:center">
\[ \begin{bmatrix} \dot{\bf x} \\ \dot{\boldsymbol{\theta}} \end{bmatrix} =
   \begin{bmatrix} {\bf b_1} & 0_{3 \times 3} \\ 0_{1 \times 3} & {\bf B_2} \end{bmatrix}
   \begin{bmatrix} u_1 \\ {\bf u_2} \end{bmatrix} \] </div>

<p> where the \( m \times n\) zero matrix is denoted by \( 0_{m \times n} \) and  </p>

<div style="text-align:center">
\[ \begin{gathered}
      {\bf b_1} = \begin{bmatrix} \cos{\psi} \cos{\theta} \\ \sin{\psi} \cos{\theta} \\ -\sin{\theta} \end{bmatrix} \\
      {\bf B_2} = \begin{bmatrix} 1 & \sin{\phi} \tan{\theta} & \cos{\phi} \tan{\theta} \\ 0 & \cos{\phi} & -\sin{\phi} \\ 0 & \sin{\phi} \sec{\theta} & \cos{\phi} \sec{\theta} \end{bmatrix}
    \end{gathered} \]
</div>

The error is defined as

<div style="text-align:center">
\[ {\bf x_e} = {\bf R}^\top ({\bf x_{ref}} - {\bf x}) \]
\[ \boldsymbol{\theta}_e = \boldsymbol{\theta}_{ref} - \boldsymbol{\theta} \]
</div>

<p> where \( {\bf R} \) is the rotation matrix </p>

<div style="text-align:center">
\[ {\bf R} = \begin{bmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\ s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\ -s\theta & c\theta s\phi & c\theta c\phi \end{bmatrix} \]
</div>

<p> Note that \( c_{\theta} \) and \( s_{\theta} \) denote \( \cos(\theta) \) and \( \sin(\theta) \) respectively. Consider the Lyapunov function: </p>

<div style="text-align:center"> \[ V = \dfrac{1}{2} {\bf x_e}^T {\bf x_e} + {\bf k}^T {\bf f}(\boldsymbol{\theta}_e) \] </div>

<p>where \( {\bf k} = \begin{bmatrix} k_1 & k_2 & k_3 \end{bmatrix}^T \) is the controller gains vector and \( {\bf f}(\boldsymbol{\theta}_e) = \begin{bmatrix} 1-\cos{\phi_e} & 1-\cos{\theta_e} & 1-\cos{\psi_e} \end{bmatrix}^T \)
is a vector-valued function. The time derivative of the Lyapunov function is
</p>

<div style="text-align:center"> \[ \dot{V} = {\bf x_e}^T \dot{\bf x}_e + {\bf k}^T \dfrac{d{\bf f}}{d \boldsymbol{\theta}_e}\dot{\boldsymbol{\theta}}_e \] </div>

The error dynamics are given as

<div style="text-align:center"> \[ \begin{gathered} \dot{\bf x}_e = {\bf b}_{1e} u_{1d} - {\bf R}^T {\bf b}_1 u_1 - \boldsymbol{\omega} \times {\bf x}_e \\ \dot{\boldsymbol{\theta}}_e = \dot{\boldsymbol{\theta}}_{ref} - \dot{\boldsymbol{\theta}} = {\bf B}_{2ref} {\bf u}_2d - {\bf B}_2 {\bf u}_2 \end{gathered} \] </div>

<p> When the error dynamics are substituted into \( \dot{V} \), it becomes </p>

<div style="text-align:center"> \[ \dot{V} = {\bf p_e}^\top \{ {\bf q} + ({\bf B_{2ref}} - {\bf B_2}){\bf u_{2ref}} - {\bf B_2}{\bf u_{2ref}} \} \\ + {\bf x_e} \{v_{ref} (\cos{\psi_e} \cos{\theta_e} - 1) - v_b \} \] </div>

The feedback control law is chosen to be

<div style="text-align:center"> \[ \begin{gathered} u_1 = v_{ref} + v_b \\ {\bf u_2} = {\bf u_{2ref}} + {\bf u_{2b}} \end{gathered} \] </div>

<p> where the subscript \( b \) denotes the feedback terms. The feedback terms are chosen to be  </p>

<div style="text-align:center"> \[ \begin{gathered} v_b = v_{ref} (\cos{\psi_e} \cos{\theta_e} - 1) + \gamma^2 x_e \\ {\bf u_{2b}} = {\bf B_2}^{-1} \{ {\bf q} + ({\bf B_{2ref}} - {\bf B_2}) {\bf u_{2ref}} + {\bf p_e} \} \end{gathered} \] </div>

<p> where \( \gamma \) is a chosen constant, \( x_e \) is the error in the local \( x \) position, \( {\bf q} = \begin{bmatrix} 0 & -z_e v_{ref} / k_2 & y_e v_{ref} \cos{\theta_e} / k_3 \end{bmatrix} ^T \), and \( {\bf p_e} = \begin{bmatrix} k_1 \sin{\phi_e} & k_2 \sin{\theta_e} & k_3 \sin{\psi_e} \end{bmatrix} ^T \). When the inputs are substituted into \( \dot{V} \), the time derivative becomes  </p>

<div style="text-align:center"> \[ \dot{V} = {\bf p}_e^T {\bf p}_e - \gamma^2 x_e^2 \] </div>

<p> which is negative semi-definite. We could also see that term \( {\bf k}^T {\bf f}(\boldsymbol{\theta}_e) \in [0,2(k_1+k_2+k_3)] \) for any \( \boldsymbol{\theta}_e \). </p>

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
\[ v = v_{ref} \]\cos(e_{\theta}) + k_1 e_x
\[ \omega = \omega_{ref} + v_{ref} (k_2 e_y + k_3 \sin(e_{\theta})) \]
\[ v_z = v_{z,{ref}} + k_4 e_z \]
</div>

<p> We can also see that the term \( \frac{1 - \cos(e_{\theta})}{k_2} \in [0, \frac{2}{k_2}] \) for any \( e_{\theta} \). </p>
