# Model notes

<!-- section: overview -->
### 1. Model overview and calculation

The models describe vertical motion about a static equilibrium. Upward displacement is positive. Positive roll raises the left side and positive pitch raises the front. The solver uses metres, seconds, kilograms, newtons and radians. The plots may display displacement in millimetres and angles in degrees. A dot denotes a time derivative.

Each degree of freedom contributes a mass or an inertia to the diagonal mass matrix $M$. The spring and damper forces act on relative displacement and velocity. If a component has motion ratio $\mathrm{MR}=\text{component travel}/\text{wheel travel}$, its wheel-side rates are

```math
k_\mathrm{wheel}=k_\mathrm{component}\,\mathrm{MR}^2,\qquad
c_\mathrm{wheel}=c_\mathrm{component}\,\mathrm{MR}^2.
```

The solver assembles the equations for the selected coordinates $\mathbf q$, puts them into first-order form and integrates them with fourth-order Runge–Kutta (RK4):

```math
M\ddot{\mathbf q}+C\dot{\mathbf q}+K\mathbf q=\mathbf f(t),
\qquad
\frac{d}{dt}
\begin{bmatrix}\mathbf q\\\mathbf v\end{bmatrix}
=
\begin{bmatrix}
\mathbf v\\ M^{-1}\bigl(\mathbf f(t)-C\mathbf v-K\mathbf q\bigr)
\end{bmatrix},
\qquad \mathbf v=\dot{\mathbf q}.
```

The web simulation starts at $\mathbf q(0)=\dot{\mathbf q}(0)=0$. The solver can take smaller internal steps than the output interval based on the system eigenvalues and input time scales. It splits steps at input events such as a road step or a random profile knot. The equations below describe the current selections. The calculation snapshot records the settings used for a stored result.

<!-- section: model.quarter.single -->
#### Undamped equivalent spring · quarter car, 1 DOF

The single moving coordinate $x$ represents the vertical displacement of the combined sprung and unsprung mass. The suspension wheel rate $k_w$ and the effective tyre stiffness $k_t$ form two springs in series. There is no independent wheel motion or damping in this model.

```math
m=m_s+m_u,\qquad k_w=k_s\,\mathrm{MR}^2,\qquad
k_\mathrm{eq}=\frac{k_w k_t}{k_w+k_t}.
```

Here $m_s$ is the sprung corner mass, $m_u$ is the unsprung corner mass and $k_s$ is the suspension spring rate at the component. For road displacement $z_r(t)$, the spring force on the mass is $-k_\mathrm{eq}(x-z_r)$:

```math
\boxed{m\ddot x+k_\mathrm{eq}\bigl(x-z_r(t)\bigr)=0.}
```

With the road fixed at $z_r=0$, the free-vibration equation and its solution are

```math
\boxed{\frac{d^2x}{dt^2}+\omega_n^2x=0},
\qquad
\omega_n=\sqrt{\frac{k_\mathrm{eq}}m},
\qquad
\boxed{x(t)=A\cos(\omega_n t+\phi)}.
```

The initial displacement and velocity determine the amplitude $A$ and phase $\phi$. The natural frequency is $f_n=\omega_n/(2\pi)$ in hertz. From the web simulation's zero initial state, motion begins when the road excites the spring.

<!-- section: model.quarter.one_eighth -->
#### Damped equivalent spring · quarter car, 1 DOF

The sprung mass $m_s$ and unsprung mass $m_u$ move together at displacement $x$. The suspension spring and tyre spring contribute the series stiffness $k_\mathrm{eq}$. The damper uses the suspension wheel-side damping $c_w$ and acts on the relative velocity $\dot x-\dot z_r$.

```math
m=m_s+m_u,\qquad k_w=k_s\,\mathrm{MR}^2,\qquad
k_\mathrm{eq}=\frac{k_w k_t}{k_w+k_t},\qquad
c_w=c_s\,\mathrm{MR}^2.
```

```math
\boxed{m\ddot x+c_w(\dot x-\dot z_r)+k_\mathrm{eq}(x-z_r)=0.}
```

With a fixed road, the natural frequency and damping ratio are $\omega_n=\sqrt{k_\mathrm{eq}/m}$ and $\zeta=c_w/(2m\omega_n)$. For $0\le\zeta<1$, the free motion is an exponentially decaying sinusoid when damping is present:

```math
x(t)=e^{-\zeta\omega_n t}\left(A\cos\omega_d t+B\sin\omega_d t\right),
\qquad \omega_d=\omega_n\sqrt{1-\zeta^2}.
```

At $\zeta=1$ the two characteristic roots coincide. For $\zeta>1$ the free response is a sum of two real exponentials. The actual simulation integrates the road-driven equation above.

<!-- section: model.quarter.double -->
#### Undamped two-mass suspension · quarter car, 2 DOF

The sprung mass $m_s$ moves at $z_s$ and the unsprung mass $m_u$ at $z_u$. The suspension spring with wheel-side stiffness $k_w=k_s\,\mathrm{MR}^2$ connects the two masses. The tyre spring with effective stiffness $k_t$ connects the wheel to the road $z_r$. The two springs transmit forces according to their own relative displacements.

```math
\boxed{
\begin{aligned}
m_s\ddot z_s&=-k_w(z_s-z_u),\\
m_u\ddot z_u&=+k_w(z_s-z_u)-k_t(z_u-z_r).
\end{aligned}}
```

For a fixed road, the two natural frequencies follow from the nonnegative roots $\omega^2$ of

```math
\det\!\left(
\begin{bmatrix}k_w&-k_w\\-k_w&k_w+k_t\end{bmatrix}
-\omega^2\begin{bmatrix}m_s&0\\0&m_u\end{bmatrix}
\right)=0.
```

The lower mode is typically dominated by sprung-mass motion and the higher mode by wheel hop. The tyre damping is zero in this model.

<!-- section: model.quarter.quarter -->
#### Damped two-mass suspension · quarter car, 2 DOF

The sprung mass $m_s$ moves at $z_s$ and the unsprung mass $m_u$ at $z_u$. The suspension spring $k_w=k_s\,\mathrm{MR}^2$ and damper $c_w=c_s\,\mathrm{MR}^2$ connect the masses in parallel. The tyre has effective stiffness $k_t$ and damping $c_t$ against the road $z_r$.

```math
\boxed{
\begin{aligned}
m_s\ddot z_s &=-k_w(z_s-z_u)-c_w(\dot z_s-\dot z_u),\\
m_u\ddot z_u &=+k_w(z_s-z_u)+c_w(\dot z_s-\dot z_u)\\
&\phantom{={}}-k_t(z_u-z_r)-c_t(\dot z_u-\dot z_r).
\end{aligned}}
```

The suspension force acts in opposite directions on the two masses. Road displacement and velocity excite the wheel through $k_t z_r+c_t\dot z_r$. The plotted suspension compression is $z_u-z_s$.

<!-- section: model.LR.2mass_2dof -->
#### Independent left and right masses on the road · lateral half car, 2 DOF

The coordinates $z_L$ and $z_R$ are the vertical displacements of the two side masses. Side $i$ moves with the combined mass $m_s+m_{u,i}$, where $m_s$ is the sprung mass per side. The road $z_{r,i}$ moves the suspension bases directly. Let $\mathbf z_s=[z_L,z_R]^\mathsf T$, $\mathbf z_b=\mathbf z_r$ and $\mathbf F_s$ be the suspension force on the moving masses. The inertia moment $T_x$ uses the reference unsprung mass per side $\mu$.

```math
T_x=2(m_s+\mu)a_y h_\mathrm{CG},
\qquad
\begin{aligned}
(m_s+m_{u,L})\ddot z_L&=F_{s,L}+T_x/w,\\
(m_s+m_{u,R})\ddot z_R&=F_{s,R}-T_x/w.
\end{aligned}
```

Here $w$ is the track width and $h_\mathrm{CG}$ is the centre-of-gravity height used for load transfer. Roll is calculated from $(z_L-z_R)/w$, and heave is $(z_L+z_R)/2$. Roll has no separate rotational inertia in this model.

<!-- section: model.LR.2mass -->
#### Independent left and right sprung masses with tyres · lateral half car, 4 DOF

Each side $i$ has a sprung coordinate $z_{s,i}$ with mass $m_s$ and a wheel coordinate $z_{u,i}$ with mass $m_{u,i}$. The suspension bases are $\mathbf z_b=\mathbf z_u$. A lateral acceleration $a_y$ creates a force couple across the track width $w$. Define $T_x=2(m_s+\mu)a_y h_\mathrm{CG}$, where $\mu$ is the model's reference unsprung mass per side.

```math
\begin{aligned}
m_s\ddot z_{s,L}&=F_{s,L}+T_x/w, &
m_s\ddot z_{s,R}&=F_{s,R}-T_x/w,\\
m_{u,L}\ddot z_{u,L}&=F_{t,L}-F_{s,L}, &
m_{u,R}\ddot z_{u,R}&=F_{t,R}-F_{s,R}.
\end{aligned}
```

Roll is derived from $(z_{s,L}-z_{s,R})/w$, with no separate roll inertia. The wheel forces $F_{t,i}$ come from the tyre springs and dampers.

<!-- section: model.LR.half_2dof -->
#### Body heave and roll on the road · lateral half car, 2 DOF

The sprung body has heave $z$ and roll $\phi$, with mass $2m_s$ and roll inertia $I_x$. Its suspension attachment points are $z_{s,L}=z+w\phi/2$ and $z_{s,R}=z-w\phi/2$, where $w$ is the track width. The road moves the suspension bases directly, so $\mathbf z_b=\mathbf z_r$. Let $\mu$ be the reference unsprung mass per side and $T_x=2(m_s+\mu)a_y h_\mathrm{CG}$ be the lateral inertia moment.

```math
\begin{aligned}
2m_s\ddot z &= F_{s,L}+F_{s,R},\\
I_x\ddot\phi &= \frac w2(F_{s,L}-F_{s,R})+T_x.
\end{aligned}
```

Heave responds to the sum of the vertical suspension forces. Roll responds to their difference acting through the half-track, plus the lateral inertia moment.

<!-- section: model.LR.half -->
#### Body heave and roll with tyres and roll-centre transfer · lateral half car, 4 DOF

The body has heave $z$ and roll $\phi$, with mass $2m_s$ and inertia $I_x$. Its attachment points are $z_{s,L}=z+w\phi/2$ and $z_{s,R}=z-w\phi/2$. The suspension bases $\mathbf z_b=\mathbf z_u$ move with the two wheel masses $m_{u,L}$ and $m_{u,R}$. The lateral force $Y=2(m_s+\mu)a_y$ uses the reference unsprung mass per side $\mu$. It creates a body moment $T_x=Yh_\mathrm{CG}$ and a geometric transfer force $G_L=-G_R=Yh_\mathrm{RC}/w$ on the wheels.

```math
\begin{aligned}
2m_s\ddot z &= (F_{s,L}-G_L)+(F_{s,R}-G_R),\\
I_x\ddot\phi &= \tfrac w2\bigl[(F_{s,L}-G_L)-(F_{s,R}-G_R)\bigr]+T_x,\\
m_{u,i}\ddot z_{u,i} &= F_{t,i}-F_{s,i}+G_i,\qquad i\in\{L,R\}.
\end{aligned}
```

Here $w$ is the track width, $h_\mathrm{CG}$ is the centre-of-gravity height and $h_\mathrm{RC}$ is the roll-centre height. The geometric force reacts on the body and wheel with opposite signs.

<!-- section: model.LR.rollcenter_all -->
#### Body heave, roll and dynamic frame transfer · lateral half car, 5 DOF

The body has heave $z$, roll $\phi$, mass $2m_s$ and inertia $I_x$. The attachment points are $z_{s,L}=z+w\phi/2$ and $z_{s,R}=z-w\phi/2$. Two wheel coordinates $z_{u,i}$ with masses $m_{u,i}$ define $\mathbf z_b=\mathbf z_u$. The fifth coordinate $\eta$ describes a phenomenological frame spring and damper driven by the roll-centre transfer command. Let $Y=2(m_s+\mu)a_y$ and $T_x=Yh_\mathrm{CG}$, with reference unsprung mass per side $\mu$.

```math
m_\eta\ddot\eta+c_\eta\dot\eta+k_\eta\eta=\frac{Yh_\mathrm{RC}}{w},
\qquad G_L=-G_R=k_\eta\eta+c_\eta\dot\eta.
```

```math
\begin{aligned}
2m_s\ddot z &= (F_{s,L}-G_L)+(F_{s,R}-G_R),\\
I_x\ddot\phi &= \tfrac w2\bigl[(F_{s,L}-G_L)-(F_{s,R}-G_R)\bigr]+T_x,\\
m_{u,i}\ddot z_{u,i} &=F_{t,i}-F_{s,i}+G_i,\qquad i\in\{L,R\}.
\end{aligned}
```

The frame spring $k_\eta$ and damper $c_\eta$ transmit the command through a dynamic state with mass $m_\eta$. The transferred force acts oppositely on the body and wheels. The track width $w$ and the heights $h_\mathrm{CG}$ and $h_\mathrm{RC}$ set the moment arms.

<!-- section: model.LR.rates -->
**Suspension rates and forces.** The left and right deflections are $\mathbf d=\mathbf z_s-\mathbf z_b$. The heave channel uses their sum and the roll channel their difference, both scaled by $1/\sqrt{2}$. Each side can also have a direct spring and damper.

```math
\begin{aligned}
T&=\tfrac1{\sqrt2}\begin{bmatrix}1&1\\1&-1\end{bmatrix},\\
K_s&=T^\mathsf T\operatorname{diag}(k_h,k_\phi)T+\operatorname{diag}(k_L,k_R),\\
C_s&=T^\mathsf T\operatorname{diag}(c_h,c_\phi)T+\operatorname{diag}(c_L,c_R),\\
\mathbf F_s&=-K_s\mathbf d-C_s\dot{\mathbf d}.
\end{aligned}
```

The $k$ and $c$ values in these matrices are wheel-side rates. The modal terms couple the left and right forces.

<!-- section: model.FR.half_2dof -->
#### Body heave and pitch on the road · longitudinal half car, 2 DOF

The model represents one side of the vehicle. The body has heave $z$, pitch $\theta$, sprung mass $2m_s$ and pitch inertia $I_y$. Front and rear attachment displacements are $z_{s,F}=z+l_f\theta$ and $z_{s,R}=z-l_r\theta$, where $l_f$ and $l_r$ are the distances from the centre of gravity to the axles. The bases of the suspensions follow the road, $z_{b,i}=z_{r,i}$.

```math
T_y=2(m_s+\mu)a_xh_\mathrm{CG},
\qquad
\begin{aligned}
2m_s\ddot z&=F_{s,F}+F_{s,R},\\
I_y\ddot\theta&=l_fF_{s,F}-l_rF_{s,R}+T_y.
\end{aligned}
```

Here $\mu$ is the reference unsprung mass per end. Positive longitudinal acceleration $a_x$ contributes a nose-up moment, and braking contributes a nose-down moment.

<!-- section: model.FR.half -->
#### Body heave and pitch with tyres and anti geometry · longitudinal half car, 4 DOF

The model represents one side of the vehicle. The body has heave $z$, pitch $\theta$, sprung mass $2m_s$ and pitch inertia $I_y$. Its attachment displacements are $z_{s,F}=z+l_f\theta$ and $z_{s,R}=z-l_r\theta$. Two unsprung masses $\mu$ move at $z_{u,F}$ and $z_{u,R}$ and define the suspension bases $z_{b,i}=z_{u,i}$.

```math
T_y=2(m_s+\mu)a_x h_\mathrm{CG},\qquad
G_F=\alpha_f\frac{T_y}{l_f+l_r},\qquad
G_R=-\alpha_r\frac{T_y}{l_f+l_r}.
```

```math
\begin{aligned}
2m_s\ddot z&=(F_{s,F}-G_F)+(F_{s,R}-G_R),\\
I_y\ddot\theta&=l_f(F_{s,F}-G_F)-l_r(F_{s,R}-G_R)+T_y,\\
\mu\ddot z_{u,i}&=F_{t,i}-F_{s,i}+G_i,\qquad i\in\{F,R\}.
\end{aligned}
```

The front and rear anti ratios $\alpha_f,\alpha_r$ set the geometric forces $G_F,G_R$. Those forces act on each wheel and react oppositely on the body. The signed acceleration $a_x$ reverses both the inertia moment and the geometric forces during braking.

<!-- section: model.FR.rates -->
**Suspension rates and forces.** At each end $i\in\{F,R\}$, the heave and direct corner elements act in parallel. Both ends use the selected axle's heave element rates. The heave motion ratio $r_{h,i}$ is element travel divided by symmetric axle heave travel. Let $r_c$ be the direct corner motion ratio. The rate sums and force are

```math
\begin{aligned}
k_i &= k_{h,i} r_{h,i}^{\,2}+k_{\mathrm{corner},i}r_c^{\,2},\\
c_i &= c_{h,i} r_{h,i}^{\,2}+c_{\mathrm{corner},i}r_c^{\,2},\\
F_{s,i}&=-k_i(z_{s,i}-z_{b,i})-c_i(\dot z_{s,i}-\dot z_{b,i}).
\end{aligned}
```

<!-- section: model.full.rigid -->
#### Rigid sprung body with four wheel masses · full car, 7 DOF

The body coordinates are heave $z$, roll $\phi$ and pitch $\theta$. The remaining four coordinates are the unsprung wheel displacements $z_{u,i}$ with masses $m_{u,i}$. Let $t_f,t_r$ denote front and rear track widths, and $l_f,l_r$ the distances from the centre of gravity to the axles. The body attachment displacements are

```math
\begin{aligned}
z_{s,FL/FR}&=z+l_f\theta\ \pm\tfrac{t_f}{2}\phi,\\
z_{s,RL/RR}&=z-l_r\theta\ \pm\tfrac{t_r}{2}\phi.
\end{aligned}
```

Use the plus sign on the left. Let $B$ be the matrix represented by these expressions, so $\mathbf z_s=B[z,\phi,\theta]^\mathsf T$. With total mass $m$, sprung mass $m_s=m-\sum_i m_{u,i}$, and body inertias $I_{xx},I_{yy}$, the equations are

```math
\begin{aligned}
\operatorname{diag}(m_s,I_{xx},I_{yy})
\begin{bmatrix}\ddot z\\\ddot\phi\\\ddot\theta\end{bmatrix}
&=B^\mathsf T(\mathbf F_s-\mathbf G)+[0,T_x,T_y]^\mathsf T,\\
m_{u,i}\ddot z_{u,i}&=F_{t,i}-F_{s,i}+G_i.
\end{aligned}
```

The same roll angle applies at both axles. $B^\mathsf T$ converts the corner forces to a vertical force, roll moment and pitch moment using their moment arms.

<!-- section: model.full.flex -->
#### Sprung body with front and rear roll coupled by chassis twist · full car, 8 DOF

The body coordinates are heave $z$, front and rear roll $\phi_f,\phi_r$, and pitch $\theta$. Four unsprung displacements $z_{u,i}$ with masses $m_{u,i}$ complete the model. With track widths $t_f,t_r$ and axle distances $l_f,l_r$, the corner displacements are

```math
\begin{aligned}
z_{s,FL/FR}&=z+l_f\theta\ \pm\tfrac{t_f}{2}\phi_f,\\
z_{s,RL/RR}&=z-l_r\theta\ \pm\tfrac{t_r}{2}\phi_r.
\end{aligned}
```

The plus sign is for the left side. Define $B$ by $\mathbf z_s=B[z,\phi_f,\phi_r,\theta]^\mathsf T$, the front share $\lambda=l_r/(l_f+l_r)$ and the sprung mass $m_s=m-\sum_i m_{u,i}$. The chassis spring $k_\tau$ and damper $c_\tau$ oppose relative axle roll with torque $T_\tau$:

```math
T_\tau=k_\tau(\phi_f-\phi_r)+c_\tau(\dot\phi_f-\dot\phi_r).
```

```math
\begin{aligned}
\operatorname{diag}(m_s,\lambda I_{xx},(1-\lambda)I_{xx},I_{yy})
\begin{bmatrix}\ddot z\\\ddot\phi_f\\\ddot\phi_r\\\ddot\theta\end{bmatrix}
&=B^\mathsf T(\mathbf F_s-\mathbf G)
+\begin{bmatrix}0\\\lambda T_x-T_\tau\\(1-\lambda)T_x+T_\tau\\T_y\end{bmatrix},\\
m_{u,i}\ddot z_{u,i}&=F_{t,i}-F_{s,i}+G_i.
\end{aligned}
```

The twist torque acts in opposite directions on the front and rear roll coordinates. The solver distributes both the total roll inertia and the lateral inertia moment using $\lambda$.

<!-- section: model.full.rates -->
**Suspension rates and forces.** In wheel order FL, FR, RL, RR, let $\mathbf d=\mathbf z_s-\mathbf z_u$. The channel matrix $T$ maps the four deflections to ten channels. Its first six rows represent front and rear heave, front and rear roll, central roll and warp. The identity block $I_4$ represents the four direct corner channels.

```math
T=\begin{bmatrix}
1& 1& 0& 0\\
0& 0& 1& 1\\
1&-1& 0& 0\\
0& 0& 1&-1\\
1&-1& 1&-1\\
1&-1&-1& 1\\
1& 0& 0& 0\\
0& 1& 0& 0\\
0& 0& 1& 0\\
0& 0& 0& 1
\end{bmatrix},
\qquad
\mathbf e=T\mathbf d.
```

```math
\mathbf F_s
=-T^\mathsf T\!\left[\operatorname{diag}(\mathbf k)\mathbf e+
\operatorname{diag}(\mathbf c)\dot{\mathbf e}\right].
```

Each channel uses its own wheel-side stiffness and damping from $\mathbf k$ and $\mathbf c$. The transpose maps channel forces back to wheel corners.

**Inertia moments and geometric transfer.** Let $\lambda=l_r/(l_f+l_r)$, $L=l_f+l_r$, and $h_\mathrm{CG}$ be the centre-of-gravity height. The inertia moments are $T_x=m a_yh_\mathrm{CG}$ and $T_y=m a_xh_\mathrm{CG}$. The roll-centre heights $h_{\mathrm{RC},f},h_{\mathrm{RC},r}$ and anti ratios $\alpha_f,\alpha_r$ give the geometric forces on the wheels:

```math
\begin{aligned}
R_f&=m a_y\lambda h_{\mathrm{RC},f}/t_f,
&R_r&=m a_y(1-\lambda)h_{\mathrm{RC},r}/t_r,\\
A_f&=\alpha_f T_y/(2L),&A_r&=-\alpha_r T_y/(2L),\\
\mathbf G&=[R_f+A_f,\,-R_f+A_f,\,R_r+A_r,\,-R_r+A_r]^\mathsf T.
\end{aligned}
```

The wheel receives $G_i$ and the body receives the opposite reaction at the same corner.

<!-- section: model.tires -->
**Tyre force and wheel load.** At each wheel the road position $z_{r,i}$ and velocity $\dot z_{r,i}$ act through the effective tyre stiffness $k_{t,i}$ and damping $c_{t,i}$. The tyre force is the change from static load $N_{i,0}$:

```math
F_{t,i}=k_{t,i}(z_{r,i}-z_{u,i})+c_{t,i}(\dot z_{r,i}-\dot z_{u,i}),
\qquad N_i=N_{i,0}+F_{t,i}.
```

A negative calculated load indicates that the linear contact assumption has been exceeded. The solver still applies the same tyre law.

<!-- section: input.road -->
The selected target sets which wheels receive the road. For wheel $i$, evaluate the profile at local time $\tau_i=t-t_0-\Delta_i$, where $t_0$ is the input start time and $\Delta_i$ is the wheel's delay, zero unless rear delay is applied. Unselected wheels and wheels with $\tau_i<0$ have zero road displacement and velocity.

<!-- section: input.rear_delay -->
When rear delay is enabled, the front wheels have $\Delta_i=0$ and the rear wheels use $\Delta_i=(l_f+l_r)/v_\mathrm{veh}$ at vehicle speed $v_\mathrm{veh}$. Otherwise all delays are zero.

<!-- section: input.bump -->
#### Raised cosine road bump

The entered height $H$ is the peak of a bump of duration $T_b$. Both the height and slope are zero at its ends. Outside $0\le\tau_i\le T_b$ the road displacement and velocity are zero.

```math
z_{r,i}(t)=\frac H2\left[1-\cos\left(\frac{2\pi\tau_i}{T_b}\right)\right],
\qquad
\dot z_{r,i}(t)=\frac{\pi H}{T_b}\sin\left(\frac{2\pi\tau_i}{T_b}\right).
```

<!-- section: input.step -->
#### Road step

The road jumps by the entered height $H$ and stays there. The solver supplies zero road velocity and omits the ideal impulse that would otherwise accompany an instantaneous jump.

```math
z_{r,i}(t)=
\begin{cases}
0,&\tau_i<0,\\
H,&\tau_i\ge 0,
\end{cases}
\qquad \dot z_{r,i}(t)=0\quad\text{in the solver}.
```

<!-- section: input.sine -->
#### Sinusoidal road

The road oscillates about zero with amplitude $H$ and frequency $f$ in hertz. From $\tau_i=0$ onward, its displacement and velocity are

```math
\boxed{z_{r,i}(t)=H\sin(2\pi f\tau_i)},
\qquad
\dot z_{r,i}(t)=2\pi fH\cos(2\pi f\tau_i).
```

The period is $1/f$. A damper responds to the velocity as well as the spring responding to displacement.

<!-- section: input.random -->
#### Seeded random road

The solver draws heights $h_j$ from a zero-mean normal distribution with standard deviation $H$, sets the first knot $h_0=0$, and uses the entered seed to reproduce the sequence. The knots are spaced by $\Delta t_r$. For $j=\lfloor\tau_i/\Delta t_r\rfloor$ and $u=\tau_i/\Delta t_r-j$, the solver interpolates linearly:

```math
z_{r,i}(t)=(1-u)h_j+u h_{j+1},
\qquad
\dot z_{r,i}(t)=\frac{h_{j+1}-h_j}{\Delta t_r}.
```

Selected wheels sample the same sequence at their respective local times.

<!-- section: input.iso_a -->
#### ISO 8608 class A road

The selected class uses reference displacement spectral density $G_0=16\times10^{-6}\,\mathrm{m^3}$.

<!-- section: input.iso_b -->
#### ISO 8608 class B road

The selected class uses reference displacement spectral density $G_0=64\times10^{-6}\,\mathrm{m^3}$.

<!-- section: input.iso_c -->
#### ISO 8608 class C road

The selected class uses reference displacement spectral density $G_0=256\times10^{-6}\,\mathrm{m^3}$.

<!-- section: input.iso_profile -->
The solver synthesizes a spatial track from a one-sided displacement power spectral density with reference frequency $n_0=0.1\,\mathrm{cycles/m}$ and band $0.011\le n\le2.83\,\mathrm{cycles/m}$. The track repeats after $L_p=4096\,\mathrm m$, with bin spacing $\Delta n=1/L_p$ and independent phases $\psi_j$ drawn from the selected seed.

```math
G(n)=G_0(n/n_0)^{-2},\qquad
h(s)=\sum_j\sqrt{2G(n_j)\Delta n}\cos(2\pi n_j s+\psi_j).
```

The solver subtracts $h(0)$ from the track and uses a cubic Hermite interpolant $P(s)$ through the sampled heights and slopes. Vehicle speed $v_\mathrm{veh}$ converts distance into time:

```math
z_{r,i}(t)=P(v_\mathrm{veh}\tau_i),\qquad
\dot z_{r,i}(t)=v_\mathrm{veh}\,P'(v_\mathrm{veh}\tau_i).
```

<!-- section: input.acceleration -->
Acceleration is applied from $t_0$ using a smooth ramp of duration $T_a$. Define $u_a=\min(1,\max(0,(t-t_0)/T_a))$. The scale $r(t)$ rises from zero to one and remains at one after the ramp.

```math
r(t)=\frac{1-\cos(\pi u_a)}2.
```

<!-- section: input.cornering -->
#### Lateral acceleration

```math
a_y(t)=a_{y,\mathrm{set}}\,r(t),\qquad a_x(t)=0,\qquad z_{r,i}(t)=0.
```

Positive $a_y$ applies an inertia moment that raises the left side. A negative input reverses its sign.

<!-- section: input.accelerate -->
#### Longitudinal acceleration

```math
a_x(t)=a_{x,\mathrm{set}}\,r(t),\qquad a_y(t)=0,\qquad z_{r,i}(t)=0.
```

Positive $a_x$ is acceleration and produces a nose-up inertia moment. Negative $a_x$ is braking and produces a nose-down moment.

<!-- section: input.bump_cornering -->
#### Simultaneous lateral acceleration

```math
a_y(t)=a_{y,\mathrm{set}}\,r(t),\qquad a_x(t)=0.
```

The raised cosine bump and lateral ramp start at the same input start time. The lateral ramp acts through the selected model's inertia and geometry terms while the bump moves the selected road bases.

<!-- section: input.combined -->
#### Simultaneous longitudinal and lateral acceleration

```math
a_x(t)=a_{x,\mathrm{set}}\,r(t),\qquad
a_y(t)=a_{y,\mathrm{set}}\,r(t).
```

The two acceleration ramps start together at $t_0$. The road bump also starts at $t_0$ on selected front wheels and follows any enabled rear delay on selected rear wheels. The solver applies the road and acceleration forces together.
