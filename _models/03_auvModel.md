---
layout: page
---

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
