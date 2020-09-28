---
layout: page
---

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
