# Error-State Kalman Filter (ESKF)

Quaternion-based Error-State Kalman Filter following:

- Joan Solà, *Quaternion kinematics for the error-state Kalman filter*,  
  https://arxiv.org/abs/1711.02508

The implementation uses a 16D nominal state with quaternion attitude and a 15D error state with small-angle attitude error. Gravity estimate omitted.

## State Definition

Nominal state:

$$
x =
\begin{bmatrix}
p \\
v \\
q \\
a_b \\
\omega_b
\end{bmatrix}
\in \mathbb{R}^{16}
$$

- $p$: position (world)
- $v$: velocity (world)
- $q$: orientation quaternion (body $\to$ world)
- $a_b$: accelerometer bias
- $\omega_b$: gyroscope bias

Error state:

$$
\delta x =
\begin{bmatrix}
\delta p \\
\delta v \\
\delta \theta \\
\delta a_b \\
\delta \omega_b
\end{bmatrix}
\in \mathbb{R}^{15}
$$

where $\delta \theta$ is the small-angle attitude error (not Euler angles).

## Nominal-State Kinematics

Following Solà, the continuous-time nominal dynamics are:

$$
\dot{p} = v\\\dot{v} = R(q)\ (a_m - a_b) + g \\ \dot{q} = q \otimes \frac{1}{2}\,\omega 
$$

$$
\quad\text{with}\quad \\ \omega=\omega_m-\omega_b, \quad
\dot{a}_b = a_w, \quad
\dot{\omega}_b = \omega_w
$$

Discrete propagation used in code (`ESKF::propagate`):

$$
p \leftarrow p + v\Delta t + \frac{1}{2}a_w\Delta t^2 \\ v \leftarrow v + a_w\Delta t \\ q \leftarrow q \otimes \exp\left((\omega_m-\omega_b)\Delta t\right)
$$

with:

$$
a_w = R(q)\ (a_m-a_b)+g
$$

## Error-State Kinematics

Using the standard first-order ESKF linearization from Solà:
$$
\delta \dot{x} = F_x\,\delta x + i
$$

The implementation uses the following discrete transition structure:

$$
F_x =
\begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 \\
0 & I & -R[a_u]_\times\Delta t & -R\Delta t & 0 \\
0 & 0 & \exp(-[\omega_u]_\times\Delta t) & 0 & -I\Delta t \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{bmatrix}
$$

where $a_u = a_m-a_b$, $\omega_u = \omega_m-\omega_b$.

## Covariance Propagation

Covariance is propagated as:

$$
P \leftarrow F_x\,P\,F_x^\top + Q_x
$$

Define the named blocks:

- $V_i = \sigma_{a_n}^2\Delta t^2I$
- $\Theta_i = \sigma_{\omega_n}^2\Delta t^2I$
- $A_i = \sigma_{a_w}^2\Delta t\,I$
- $\Omega_i = \sigma_{\omega_w}^2\Delta t\,I$

With error-state ordering
$
\delta x =
[\delta p,\ \delta v,\ \delta \theta,\ \delta a_b,\ \delta \omega_b]^\top,
$
the implemented process-noise covariance is

$$
Q_x =
\begin{bmatrix}
0 & 0 & 0 & 0 & 0 \\
0 & V_i & 0 & 0 & 0 \\
0 & 0 & \Theta_i & 0 & 0 \\
0 & 0 & 0 & A_i & 0 \\
0 & 0 & 0 & 0 & \Omega_i
\end{bmatrix}
$$

This is to replace Joan Sola's $F_i Q_iF_i^T$ because $F_i$ only consists of $I$ on the diagonal. 

Note that the matrix $F_x$ is sparse and mostly consist of $I$ and $0$ blocks. This means in the code we should use blockwise updates instead of a $15\times15$ expensive matrix multiplication.

## Measurement Update (Generic ESKF Form)

For a measurement model linearized in error-state form:

$$
r = z - h(\hat{x})
\approx H\,\delta x + n
$$

Following Solà, the error-state Jacobian is defined as:

$$
H \;=\; \left.\frac{\partial h}{\partial \delta x}\right|_{x}
$$

and can be written with chain rule as:

$$
H = \left.\frac{\partial h}{\partial x}\right|_{\hat{x}}\left.\frac{\partial x}{\partial \delta x}\right|_{\delta x=0} = H_x \cdot X_{\delta x}
$$

The filter update is:

$$
K = PH^\top(HPH^\top + V)^{-1} \\ \delta\hat{x} = Kr\\P \leftarrow (I-KH)P(I-KH)^\top + KVK^\top
$$

(Joseph form, used for improved numerical stability.)

## Error Injection / State Reset

After update, error is injected into the nominal state (Solà reset step):

$$
p \leftarrow p + \delta p,
\quad
v \leftarrow v + \delta v \\ q \leftarrow q \otimes \exp_q(\delta \theta) \\ a_b \leftarrow a_b + \delta a_b,
\quad
\omega_b \leftarrow \omega_b + \delta \omega_b
$$

Then quaternion is normalized.
Reset is trivial according to Sola.

## Weak correction with gravity observation

`correct_gravity()` uses normalized accelerometer direction as a gravity-direction measurement:

- Predicted direction in body frame: 
$$h(q) = q^* \otimes (-g + a_b) \otimes q$$

with $g=[0,0,-9.81]^\top$
- Residual: $r=z-h$
- Jacobian: $H_{\theta}= [h(q)]_\times$, $H_{ab} = R^T(q)$

Here H is computed directly, it skips the chain-rule in Joan Sola's p.63 eq. 278. 

Justification: 
Let 

$$
 h(q) = R(q)^T (-g + a_b)
$$

be the acceleration that we predict to measure in the body frame where $q$ is our nominal state quaternion. 

Pertube it with small angle $\delta \theta$ we get the true measurement (that the accel reads):

$$h(q_t) = (R(q) \ \exp(\delta \theta))^T \ f_w = \exp(\delta \theta)^TR(q)^T(g + a_b)
$$

Now assume that for small $\delta\theta$, we have $\exp(\delta \theta) \approx I + [\delta \theta]_\times$ we get

$$H_\theta = \frac{\partial h(q_t)}{\partial \delta \theta} = \frac{\partial }{\partial \delta \theta} \  (I + [\delta \theta]_\times)^T \ R(q)^T \ (g + a_b) \\ 
= \frac{\partial }{\partial \delta \theta} \ -[\delta \theta]_\times \ R(q)^T  \ (g + a_b) = [R(q)^T \ (g + a_b)]_\times \ \delta \theta \\
= [R(q)^T (g + a_b)]_\times = [h(q)]_\times
$$

similarly, pertube $a_b$ with $\delta a_b$ we get

$$H_{ab} = \frac{\partial h(q_t)}{\partial \delta a_b} = \frac{\partial }{\partial \delta a_b} \ R(q)^T \ (g + a_b +\delta a_b) \\ 
= R(q)^T
$$

So that the final $H$ is 
$$
H = \begin{bmatrix}0 & 0 & H_\theta & H_{ab} & 0\end{bmatrix}
$$

This means we should also compute $K$ and $P$ blockwise to save computation time.


I also did some scaling of the covariance to the magnitude of $g$ to improve robustness

## Optical Flow Measurement Model

The flow measurement is assumed to measure the velocity of a point $P$ on the ground relative to the sensor, projected into the sensor's frame of reference and only the $x$ and $y$ component is measured.

The point $P$ is defined to be the intersection of the flow sensor's principal optical axis ($z$ axis, which is the same as the drone's downward facing $z$ axis) with the flat ground plane. The distance $\rho$ is measured by the range sensor and is the distance $OP$ from the sensor to point $P$, with $\rho = {}_Sz_P$ i.e it is the $z$ coordinate of the point in sensor coordinate frame.

### Frames and notation

We use the following frames and points:

- $E$: earth / odom frame
- $B$: body frame
- $G$: drone center of mass / body reference point
- $S$: optical-flow sensor point
- $O$: observed ground point, approximated as the point seen along the sensor optical axis

Notation:

- ${}_Av_P$ means the velocity of point $P$, expressed in frame $A$.

- ${}_Ar_{PQ}$   means the position vector from point $P$ to point $Q$, expressed in frame $A$.

The ESKF nominal velocity is:

$$
v = {}_Ev_G
$$

which is the velocity of the drone center of mass, expressed in the earth / odom frame.

The nominal attitude quaternion maps body-frame vectors into earth-frame vectors:

$$
q: B \rightarrow E
$$

Therefore, the rotation from earth to body is:

$$
{}_BR_E \cdot {}_Ev \leftrightarrow q^* v q
$$

and the center-of-mass velocity expressed in body frame is:

$$
{}_Bv_G = {}_BR_E \ v
$$

Then in the body / sensor frame, its velocity is

$$
{}_Bv_S = {}_B\omega_B \times {}_Br_{GS} + {}_BR_E \ v = - {}_Bv_{SO}
$$

Where $O$ is a reference point on the ground that the sensor is observing, here we approximately say it is the point that the ToF ranger sees.

But the sensor actually observe: 

$$
\frac{d}{dt}{}_B r_{SO} = {}_B v_{SO} - {}_B\omega_B \times \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix} 
$$

With $\rho$ being the distance from the sensor to $O$ that the ranger is measuring. Furthermore, we only observe the flow in $x$ and $y$ direction, so the full measurement is

$$h(x) = \begin{bmatrix}f_x \\ f_y\end{bmatrix} = \frac{1}{\rho}\begin{bmatrix}1 & 0 & 0 \\ 0 & 1 & 0\end{bmatrix} [ -(w_m - w_b) \times {}_Br_{GS} - {}_BR_E \ v - (w_m - w_b) \times \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix} ]
$$

Pulling the constants out, we have 

$$h(x) = C + \frac{1}{\rho}S [-({}_Br_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}) \times w_b - {}_BR_E \ v]
$$

Now pertube the states with $\delta w_b$ and $\delta \theta$ we get: 

$$
h(x_t) = C + \frac{1}{\rho} S [-({}_Br_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}) \times (w_b + \delta w_b) - \exp(\delta \theta)^T \ {}_ER_B^T  \ (v + \delta v)]
$$

And using the same small angle approximation $\exp(\delta {\theta}) \approx I + [\delta {\theta}]_\times$ we get the Jacobian blocks that are the derivative w.r.t $\delta {v}$, $\delta {\theta}$ and $\delta {w_b}$: 

$$H_v = -\frac{1}{\rho} \ S \ {}_B R_E$$

$$H_\theta = - \frac{1}{\rho} \ S \ [{}_B R_E v]_\times
$$

$$H_{w_b} = - \frac{1}{\rho} \ S \ [{}_B r_{GS} + \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix}]_{\times}
$$

So that now we can construct the entire Jacobian w.r.t the error state as: 

$$H = \begin{bmatrix} 0_{2\times3} & H_v & H_\theta & 0_{2\times3} & H_{w_b}\end{bmatrix}
$$

## Range sensor measurement model 
As stated above, the range sensor measures $\rho = OP$, with O being the sensor center and $P$ being the intersection of the sensor's $z$ axis ray with the flat ground. We now derive $\rho$ from the nominal state $p$ and $q$ of the drone.

First, the coordinate of the sensor in world coordinate is

$$
{}_E r_{S} = {}_Er_G + {}_ER_B \ {}_Br_{GS} = p + {}_ER_B \ {}_Br_{GS}
$$

We know that ${}_B{\rho} = \begin{bmatrix}0 \\ 0 \\ \rho\end{bmatrix} = e_z \cdot \rho $ rotated into world frame and projected onto $z$ axis, plus the terrain height should give the negative z coordinate of $_E r_{S}$ (negative because our z points down). In math: 

$$
-e_z^T \ {}_E r_{S} =  e_z^T {}_ER_B \ e_z \cdot \rho + h_t
$$

Therefore inverting it we find the measurement

$$
h(x) = \rho = \frac{-e_z^T \ {}_E r_{S} - h_t}{e_z^T {}_ER_B \ e_z}
$$

That fancy denominator term is simply the entry at row 3, column 3 of the matrix ${}_ER_B$

Now we pertube this measurement with $\delta p$ and $\delta \theta$: 

$$
h(x_t) = \frac{-e_z^T \ (p +  \delta p + ({}_ER_B \ (I + [\delta \theta]_\times ) \ {}_Br_{GS}) - h_t}{e_z^T {}_ER_B(I + [\delta \theta]_\times ) \ e_z}
$$

And take the derivative w.r.t $\delta p$ and $\delta \theta$. 

For $\delta p$, this is easy: 

$$
H_p = \frac{\partial h(x_t)}{\partial \delta p} = \frac{-e_z^T }{e_z^T {}_ER_B \ e_z}
$$

For $\delta \theta$ the math is a bit disgusting, but basically we use the quotient rule to find

$$
H_\theta = \frac{e_z^T \ {}_ER_B [{}_Br_{GS} + e_z \rho]_\times}{e_z^T {}_ER_B \ e_z}
$$
