# Error-State Kalman Filter (ESKF)

Quaternion-based Error-State Kalman Filter following:

- Joan Solà, *Quaternion kinematics for the error-state Kalman filter*,  
  https://arxiv.org/abs/1711.02508

The implementation uses a 16D nominal state with quaternion attitude and a 15D error state with small-angle attitude error. Gravity estimate omitted.

## State Definition

Nominal state:

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\mathbf{q} \\
\mathbf{a}_b \\
\boldsymbol{\omega}_b
\end{bmatrix}
\in \mathbb{R}^{16}
$$

- $\mathbf{p}$: position (world)
- $\mathbf{v}$: velocity (world)
- $\mathbf{q}$: orientation quaternion (body $\to$ world)
- $\mathbf{a}_b$: accelerometer bias
- $\boldsymbol{\omega}_b$: gyroscope bias

Error state:

$$
\delta \mathbf{x} =
\begin{bmatrix}
\delta\mathbf{p} \\
\delta\mathbf{v} \\
\delta\boldsymbol{\theta} \\
\delta\mathbf{a}_b \\
\delta\boldsymbol{\omega}_b
\end{bmatrix}
\in \mathbb{R}^{15}
$$

where $\delta\boldsymbol{\theta}$ is the small-angle attitude error (not Euler angles).

## Nominal-State Kinematics

Following Solà, the continuous-time nominal dynamics are:

$$
\dot{\mathbf{p}} = \mathbf{v} \\

\dot{\mathbf{v}} = \mathbf{R}(\mathbf{q})\,(\mathbf{a}_m - \mathbf{a}_b) + \mathbf{g} \\



\dot{\mathbf{q}} = \mathbf{q} \otimes \frac{1}{2}\,\boldsymbol{\omega} \\ 
$$

$$
\quad\text{with}\quad \\
\boldsymbol{\omega}=\mathbf{\omega}_m-\boldsymbol{\omega}_b, \quad

\dot{\mathbf{a}}_b = \mathbf{a}_w, \quad

\dot{\boldsymbol{\omega}}_b = \boldsymbol{\omega}_w
$$

Discrete propagation used in code (`ESKF::propagate`):

$$
\mathbf{p} \leftarrow \mathbf{p} + \mathbf{v}\Delta t + \frac{1}{2}\mathbf{a}_w\Delta t^2 \\


\mathbf{v} \leftarrow = \mathbf{v} + \mathbf{a}_w\Delta t \\

\mathbf{q} \leftarrow \mathbf{q} \otimes \exp\left((\mathbf{\omega}_m-\boldsymbol{\omega}_b)\Delta t\right)
$$

with:

$$
\mathbf{a}_w = \mathbf{R}(\mathbf{q})\,(\mathbf{a}_m-\mathbf{a}_b)+\mathbf{g}
$$

## Error-State Kinematics

Using the standard first-order ESKF linearization from Solà:
$$
\delta \dot{\mathbf{x}} = \mathbf{F}_x\,\delta \mathbf{x} + \mathbf{i}
$$

The implementation uses the following discrete transition structure:

$$
\mathbf{F}_x =
\begin{bmatrix}
\mathbf{I} & \mathbf{I}\Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & -\mathbf{R}[\mathbf{a}_u]_\times\Delta t & -\mathbf{R}\Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \exp(-[\boldsymbol{\omega}_u]_\times\Delta t) & \mathbf{0} & -\mathbf{I}\Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

where $\mathbf{a}_u = \mathbf{a}_m-\mathbf{a}_b$, $\boldsymbol{\omega}_u = \boldsymbol{\omega}_m-\boldsymbol{\omega}_b$.

## Covariance Propagation

Covariance is propagated as:

$$
\mathbf{P} \leftarrow \mathbf{F}_x\,\mathbf{P}\,\mathbf{F}_x^\top + \mathbf{Q}_x
$$

Define the named blocks:

- $V_i = \sigma_{a_n}^2\Delta t^2\mathbf{I}$
- $\Theta_i = \sigma_{\omega_n}^2\Delta t^2\mathbf{I}$
- $A_i = \sigma_{a_w}^2\Delta t\,\mathbf{I}$
- $\Omega_i = \sigma_{\omega_w}^2\Delta t\,\mathbf{I}$

With error-state ordering
$
\delta \mathbf{x} =
[\delta\mathbf{p},\ \delta\mathbf{v},\ \delta\boldsymbol{\theta},\ \delta\mathbf{a}_b,\ \delta\boldsymbol{\omega}_b]^\top,
$
the implemented process-noise covariance is

$$
\mathbf{Q}_x =
\begin{bmatrix}
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & V_i & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \Theta_i & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & A_i & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \Omega_i
\end{bmatrix}
$$

This is to replace Joan Sola's $F_i Q_iF_i^T$ because F_i only consists of $I$ on the diagonal

## Measurement Update (Generic ESKF Form)

For a measurement model linearized in error-state form:

$$
\mathbf{r} = \mathbf{z} - h(\hat{\mathbf{x}})
\approx \mathbf{H}\,\delta\mathbf{x} + \mathbf{n}
$$

Following Solà, the error-state Jacobian is defined as:

$$
\mathbf{H} \;=\; \left.\frac{\partial h}{\partial \delta \mathbf{x}}\right|_{\mathbf{x}}
$$

and can be written with chain rule as:

$$
\mathbf{H}
=
\left.\frac{\partial h}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}}
\left.\frac{\partial \mathbf{x}}{\partial \delta\mathbf{x}}\right|_{\delta\mathbf{x}=0} = H_x \cdot X_{\delta x}
$$

The filter update is:

$$
\mathbf{K} = \mathbf{P}\mathbf{H}^\top(\mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R})^{-1} \\


\delta\hat{\mathbf{x}} = \mathbf{K}\mathbf{r}\\

\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top
$$

(Joseph form, used for improved numerical stability.)

## Error Injection / State Reset

After update, error is injected into the nominal state (Solà reset step):

$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p},
\quad
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v} \\


\mathbf{q} \leftarrow \mathbf{q} \otimes \exp_q(\delta\boldsymbol{\theta}) \\

\mathbf{a}_b \leftarrow \mathbf{a}_b + \delta\mathbf{a}_b,
\quad
\boldsymbol{\omega}_b \leftarrow \boldsymbol{\omega}_b + \delta\boldsymbol{\omega}_b
$$

Then quaternion is normalized.
Reset is trivial according to Sola.

## Weak correction with gravity observation

`correct_gravity()` uses normalized accelerometer direction as a gravity-direction measurement:

- Predicted direction in body frame: 
$$\mathbf{h}(q) = \mathbf{q}^* \otimes \mathbf{f}_w \otimes \mathbf{q}$$

with $\mathbf{f}_w=[0,0,-1]^\top$
- Residual: $\mathbf{r}=\mathbf{z}-\mathbf{h}$
- Jacobian uses attitude-error block only: $\mathbf{H}_{\theta}= [\mathbf{h(q)}]_\times$

Here H is computed directly, it skips the chain-rule in Joan Sola's p.63 eq. 278. 

Justification: 
Let 

$$
 h(q) = R(q)ᵀ \ \exp(\delta \theta) f_w
$$

be the acceleration in $g$ that we predict to measure in the body frame where $q$ is our nominal state quaternion and $\delta \theta$ is our error state. 

Pertube it with small angle $\delta \theta$ we get the true measurement (that the accel reads):

$$
  h(q_t) = (R(q) \ \exp(\delta \theta))ᵀ \ f_w = \exp(\delta \theta)^TR(q)^Tfw
$$
Now assume that for small $\delta\theta$, we have $\exp(\delta \theta) \approx I + [\delta \theta]_\times$ we get

$$
  H = \frac{\partial h(q_t)}{\partial \delta \theta} = \frac{\partial }{\partial \delta \theta} \  (I + [\delta \theta]_\times)^T \ R(q)^T \ f_w \\ 
  = \frac{\partial }{\partial \delta \theta} \ -[\delta \theta]_\times \ R(q)^T  \ f_w = [R(q)^T \ f_w]_\times \ \delta \theta \\
  = [R(q)^T [f_w]]_\times = [h(q)]_\times
$$


I also did some scaling of the covariance to the magnitude of $g$ to improve robustness

## Optical Flow Measurement Model

### Frames and notation

We use the following frames and points:

- $E$: earth / odom frame
- $B$: body frame
- $G$: drone center of mass / body reference point
- $S$: optical-flow sensor point
- $O$: observed ground point, approximated as the point seen along the sensor optical axis

Notation:

- $
{}_A\mathbf{v}_P
$ means the velocity of point $P$, expressed in frame $A$.

- $
{}_A\mathbf{r}_{PQ}
$   means the position vector from point $P$ to point $Q$, expressed in frame $A$.

The ESKF nominal velocity is:

$$
\mathbf{v} = {}_E\mathbf{v}_G
$$

which is the velocity of the drone center of mass, expressed in the earth / odom frame.

The nominal attitude quaternion maps body-frame vectors into earth-frame vectors:

$$
\mathbf{q}: B \rightarrow E
$$

Therefore, the rotation from earth to body is:

$$
{}_B\mathbf{R}_E \cdot {}_Ev \leftrightarrow q^* v q
$$

and the center-of-mass velocity expressed in body frame is:

$$
{}_B\mathbf{v}_G
=
{}_B\mathbf{R}_E \ \mathbf{v}
$$


Then in the body / sensor frame, its velocity is

$$
{}_B\mathbf{v}_S = {}_B\omega_B \times {}_Br_{GS} + {}_BR_E \ v = - {}_B\mathbf{v}_{SO}
$$
Where $O$ is a reference point on the ground that the sensor is observing, here we approximately say it is the point that the ToF ranger sees.

But the sensor actually observe: 
$$
  \frac{d}{dt}{}_B \mathbf r_{SO} = {}_B\mathbf v_{SO} - {}_B\omega_B \times \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix} 
$$
With $\rho$ being the distance from the sensor to $O$ that the ranger is measuring. Furthermore, we only observe the flow in $x$ and $y$ direction, so the full measurement is
$$
h(x) = \begin{bmatrix}f_x \\ f_y\end{bmatrix} = \frac{1}{\rho}\begin{bmatrix}1 & 0 & 0 \\ 0 & 1 & 0\end{bmatrix} [ -(w_m - w_b) \times {}_Br_{GS} - {}_BR_E \ v - (w_m - w_b) \times \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix} ]
$$

Pulling the constants out, we have 

$$
  h(x) = C + \frac{1}{\rho}S [-({}_Br_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}) \times w_b - {}_BR_E \ v]
$$

Now pertube the states with $\delta w_b$ and $\delta \theta$ we get: 
$$
  h(x_t) = C + \frac{1}{\rho} S [-({}_Br_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}) \times (w_b + \delta w_b) - \exp(\delta \theta)^T \ {}_ER_B^T  \ (v + \delta v)]
$$

And using the same small angle approximation $\exp(\delta \theta) \approx I + [\delta \theta]_\times$ we get the Jacobian blocks: 

$$
H_v​= −\frac{1}{\rho} \ ​S \ {}_B​R_E
$$

$$
H_\theta ​=− \frac{1}{\rho}​ \ ​S \ [{}_B​R_E v​]_\times​
$$

$$
  H_{w_b}​​=− \frac{1}{\rho} ​\ S \ [{}_Br_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}]_\times
$$
