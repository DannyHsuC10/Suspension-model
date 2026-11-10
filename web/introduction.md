# 模型說明

<!-- section: overview -->
### 1. 模型概述與計算

這些模型描述了繞著靜態平衡點的垂直運動。向上位移為正。正側傾會抬高左側，正俯仰會抬高前端。求解器使用公尺、秒、公斤、牛頓和徑度。繪圖可能會以公釐顯示位移，以度數顯示角度。點（dot）表示時間導數。

每個自由度會對對角質量矩陣 $M$ 貢獻一個質量或轉動慣量。彈簧與阻尼力作用於相對位移與速度。若某組件的運動比率（motion ratio）為 $\mathrm{MR}=\text{組件行程}/\text{輪胎行程}$，則其輪端速率為

```math
k_\mathrm{wheel}=k_\mathrm{component}\,\mathrm{MR}^2,\qquad
c_\mathrm{wheel}=c_\mathrm{component}\,\mathrm{MR}^2.
```

求解器將所選座標 $\mathbf q$ 的方程式組合起來，轉化為一階形式，並使用四階龍格-庫塔法（Runge–Kutta, RK4）進行積分：

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

網頁模擬從 $\mathbf q(0)=\dot{\mathbf q}(0)=0$ 開始。求解器可以根據系統特徵值和輸入時間尺度，採用小於輸出間隔的內部步長。它會在輸入事件（例如路面階躍或隨機輪廓節點）處分割步長。下方的方程式描述了當前的選擇。計算快照會記錄用於儲存結果的設定。

<!-- section: model.quarter.single -->
#### 無阻尼等效彈簧 · 四分之一車，1 自由度

單一運動座標 $x$ 表示彈簧下質量與非彈簧下質量合併後的垂直位移。懸吊輪端剛度 $k_w$ 與有效輪胎剛度 $k_t$ 形成兩個串聯彈簧。此模型中沒有獨立的車輪運動或阻尼。

```math
m=m_s+m_u,\qquad k_w=k_s\,\mathrm{MR}^2,\qquad
k_\mathrm{eq}=\frac{k_w k_t}{k_w+k_t}.
```

此處 $m_s$ 為四分之一車的彈簧下質量（車身質量），$m_u$ 為四分之一車的非彈簧下質量（輪端質量），$k_s$ 為組件處的懸吊彈簧剛度。對於路面位移 $z_r(t)$，作用在質量上的彈簧力為 $-k_\mathrm{eq}(x-z_r)$：

```math
\boxed{m\ddot x+k_\mathrm{eq}\bigl(x-z_r(t)\bigr)=0.}
```

當路面固定在 $z_r=0$ 時，自由振動方程式及其解為

```math
\boxed{\frac{d^2x}{dt^2}+\omega_n^2x=0},
\qquad
\omega_n=\sqrt{\frac{k_\mathrm{eq}}m},
\qquad
\boxed{x(t)=A\cos(\omega_n t+\phi)}.
```

初始位移和速度決定了振幅 $A$ 與相位 $\phi$。自然頻率以赫茲表示為 $f_n=\omega_n/(2\pi)$。從網頁模擬的零初始狀態開始，當路面激振彈簧時運動便會開始。

<!-- section: model.quarter.one_eighth -->
#### 有阻尼等效彈簧 · 四分之一車，1 自由度

彈簧下質量 $m_s$ 與非彈簧下質量 $m_u$ 以位移 $x$ 一起運動。懸吊彈簧與輪胎彈簧提供串聯剛度 $k_\mathrm{eq}$。阻尼器使用懸吊輪端阻尼 $c_w$ 並作用於相對速度 $\dot x-\dot z_r$。

```math
m=m_s+m_u,\qquad k_w=k_s\,\mathrm{MR}^2,\qquad
k_\mathrm{eq}=\frac{k_w k_t}{k_w+k_t},\qquad
c_w=c_s\,\mathrm{MR}^2.
```

```math
\boxed{m\ddot x+c_w(\dot x-\dot z_r)+k_\mathrm{eq}(x-z_r)=0.}
```

在固定路面下，自然頻率與阻尼比分別為 $\omega_n=\sqrt{k_\mathrm{eq}/m}$ 及 $\zeta=c_w/(2m\omega_n)$。對於 $0\le\zeta<1$，存在阻尼時的自由運動為指數衰減的正弦波：

```math
x(t)=e^{-\zeta\omega_n t}\left(A\cos\omega_d t+B\sin\omega_d t\right),
\qquad \omega_d=\omega_n\sqrt{1-\zeta^2}.
```

當 $\zeta=1$ 時，兩個特徵根重合。對於 $\zeta>1$，自由響應是兩個實數指數項的總和。實際模擬會對上述受路面驅動的方程式進行積分。

<!-- section: model.quarter.double -->
#### 無阻尼雙質量懸吊 · 四分之一車，2 自由度

彈簧下質量 $m_s$ 以 $z_s$ 運動，非彈簧下質量 $m_u$ 以 $z_u$ 運動。具有輪端剛度 $k_w=k_s\,\mathrm{MR}^2$ 的懸吊彈簧連接這兩個質量。具有有效剛度 $k_t$ 的輪胎彈簧將車輪連接至路面 $z_r$。兩個彈簧根據其各自的相對位移傳遞力量。

```math
\boxed{
\begin{aligned}
m_s\ddot z_s&=-k_w(z_s-z_u),\\
m_u\ddot z_u&=+k_w(z_s-z_u)-k_t(z_u-z_r).
\end{aligned}}
```

對於固定路面，兩個自然頻率可由下式的非負根 $\omega^2$ 求得：

```math
\det\!\left(
\begin{bmatrix}k_w&-k_w\\-k_w&k_w+k_t\end{bmatrix}
-\omega^2\begin{bmatrix}m_s&0\\0&m_u\end{bmatrix}
\right)=0.
```

低頻模態通常由彈簧下質量的運動主導，高頻模態則由車輪跳動（wheel hop）主導。此模型中的輪胎阻尼為零。

<!-- section: model.quarter.quarter -->
#### 有阻尼雙質量懸吊 · 四分之一車，2 自由度

彈簧下質量 $m_s$ 以 $z_s$ 運動，非彈簧下質量 $m_u$ 以 $z_u$ 運動。懸吊彈簧 $k_w=k_s\,\mathrm{MR}^2$ 與阻尼器 $c_w=c_s\,\mathrm{MR}^2$ 以並聯方式連接這兩個質量。輪胎具有對路面 $z_r$ 的有效剛度 $k_t$ 與阻尼 $c_t$。

```math
\boxed{
\begin{aligned}
m_s\ddot z_s &=-k_w(z_s-z_u)-c_w(\dot z_s-\dot z_u),\\
m_u\ddot z_u &=+k_w(z_s-z_u)+c_w(\dot z_s-\dot z_u)\\
&\phantom{={}}-k_t(z_u-z_r)-c_t(\dot z_u-\dot z_r).
\end{aligned}}
```

懸吊力對兩個質量施加相反方向的作用力。路面位移和速度透過 $k_t z_r+c_t\dot z_r$ 激振車輪。繪圖中所顯示的懸吊壓縮量為 $z_u-z_s$。

<!-- section: model.LR.2mass_2dof -->
#### 路面上的左右獨立質量 · 橫向半車，2 自由度

座標 $z_L$ 與 $z_R$ 分別為兩側質量的垂直位移。側 $i$ 以合併質量 $m_s+m_{u,i}$ 運動，其中 $m_s$ 為單側的彈簧下質量。路面 $z_{r,i}$ 直接驅動懸吊基座。令 $\mathbf z_s=[z_L,z_R]^\mathsf T$，$\mathbf z_b=\mathbf z_r$，且 $\mathbf F_s$ 為作用在運動質量上的懸吊力。轉動慣量矩 $T_x$ 使用單側參考非彈簧下質量 $\mu$。

```math
T_x=2(m_s+\mu)a_y h_\mathrm{CG},
\qquad
\begin{aligned}
(m_s+m_{u,L})\ddot z_L&=F_{s,L}+T_x/w,\\
(m_s+m_{u,R})\ddot z_R&=F_{s,R}-T_x/w.
\end{aligned}
```

此處 $w$ 為輪距（track width），$h_\mathrm{CG}$ 為用於載荷轉移（load transfer）的重心高度。側傾由 $(z_L-z_R)/w$ 計算，起伏（heave）由 $(z_L+z_R)/2$ 計算。此模型中側傾沒有獨立的轉動慣量。

<!-- section: model.LR.2mass -->
#### 帶輪胎的左右獨立彈簧下質量 · 橫向半車，4 自由度

每側 $i$ 具有質量為 $m_s$ 的彈簧座標 $z_{s,i}$，以及質量為 $m_{u,i}$ 的車輪座標 $z_{u,i}$。懸吊基座為 $\mathbf z_b=\mathbf z_u$。橫向加速度 $a_y$ 在輪距 $w$ 跨度上產生一個力偶。定義 $T_x=2(m_s+\mu)a_y h_\mathrm{CG}$，其中 $\mu$ 為模型的單側參考非彈簧下質量。

```math
\begin{aligned}
m_s\ddot z_{s,L}&=F_{s,L}+T_x/w, &
m_s\ddot z_{s,R}&=F_{s,R}-T_x/w,\\
m_{u,L}\ddot z_{u,L}&=F_{t,L}-F_{s,L}, &
m_{u,R}\ddot z_{u,R}&=F_{t,R}-F_{s,R}.
\end{aligned}
```

側傾由 $(z_{s,L}-z_{s,R})/w$ 推導得出，無獨立的側傾轉動慣量。車輪力 $F_{t,i}$ 來自輪胎彈簧與阻尼器。

<!-- section: model.LR.half_2dof -->
#### 路面上的車身起伏與側傾 · 橫向半車，2 自由度

車身具有起伏 $z$ 與側傾 $\phi$，總質量為 $2m_s$，側傾轉動慣量為 $I_x$。其懸吊連接點為 $z_{s,L}=z+w\phi/2$ 及 $z_{s,R}=z-w\phi/2$，其中 $w$ 為輪距。路面直接驅動懸吊基座，因此 $\mathbf z_b=\mathbf z_r$。令 $\mu$ 為單側參考非彈簧下質量，$T_x=2(m_s+\mu)a_y h_\mathrm{CG}$ 為橫向慣性矩。

```math
\begin{aligned}
2m_s\ddot z &= F_{s,L}+F_{s,R},\\
I_x\ddot\phi &= \frac w2(F_{s,L}-F_{s,R})+T_x.
\end{aligned}
```

起伏響應垂直懸吊力的總和。側傾響應透過半輪距作用的力量差，加上橫向慣性矩。

<!-- section: model.LR.half -->
#### 具備輪胎與滾轉中心轉移的車身起伏與側傾 · 橫向半車，4 自由度

車身具有起伏 $z$ 與側傾 $\phi$，質量為 $2m_s$，轉動慣量為 $I_x$。其連接點為 $z_{s,L}=z+w\phi/2$ 與 $z_{s,R}=z-w\phi/2$。懸吊基座 $\mathbf z_b=\mathbf z_u$ 隨兩個車輪質量 $m_{u,L}$ 與 $m_{u,R}$ 一起運動。橫向力 $Y=2(m_s+\mu)a_y$ 使用單側參考非彈簧下質量 $\mu$。它在車身上產生一個力矩 $T_x=Yh_\mathrm{CG}$，並在車輪上產生幾何轉移力 $G_L=-G_R=Yh_\mathrm{RC}/w$。

```math
\begin{aligned}
2m_s\ddot z &= (F_{s,L}-G_L)+(F_{s,R}-G_R),\\
I_x\ddot\phi &= \tfrac w2\bigl[(F_{s,L}-G_L)-(F_{s,R}-G_R)\bigr]+T_x,\\
m_{u,i}\ddot z_{u,i} &= F_{t,i}-F_{s,i}+G_i,\qquad i\in\{L,R\}.
\end{aligned}
```

此處 $w$ 為輪距，$h_\mathrm{CG}$ 為重心高度，$h_\mathrm{RC}$ 為滾轉中心（roll-centre）高度。幾何力以相反符號作用於車身和車輪上。

<!-- section: model.LR.rollcenter_all -->
#### 車身起伏、側傾與動態車架轉移 · 橫向半車，5 自由度

車身具有起伏 $z$、側傾 $\phi$、質量 $2m_s$ 以及轉動慣量 $I_x$。連接點為 $z_{s,L}=z+w\phi/2$ 與 $z_{s,R}=z-w\phi/2$。兩個車輪座標 $z_{u,i}$ 及質量 $m_{u,i}$ 定義了 $\mathbf z_b=\mathbf z_u$。第五個座標 $\eta$ 描述由滾轉中心轉移命令所驅動的現象學車架彈簧與阻尼器。令 $Y=2(m_s+\mu)a_y$ 且 $T_x=Yh_\mathrm{CG}$，單側參考非彈簧下質量為 $\mu$。

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

車架彈簧 $k_\eta$ 與阻尼器 $c_\eta$ 透過具有質量 $m_\eta$ 的動態狀態傳遞命令。轉移力以相反方向作用於車身與車輪。輪距 $w$ 以及高度 $h_\mathrm{CG}$ 與 $h_\mathrm{RC}$ 設定了力臂。

<!-- section: model.LR.rates -->
**懸吊速率與力。** 左右側的形變量為 $\mathbf d=\mathbf z_s-\mathbf z_b$。起伏通道使用其總和，側傾通道使用其差值，兩者皆縮放 $1/\sqrt{2}$。每側也可以有直接的彈簧與阻尼器。

```math
\begin{aligned}
T&=\tfrac1{\sqrt2}\begin{bmatrix}1&1\\1&-1\end{bmatrix},\\
K_s&=T^\mathsf T\operatorname{diag}(k_h,k_\phi)T+\operatorname{diag}(k_L,k_R),\\
C_s&=T^\mathsf T\operatorname{diag}(c_h,c_\phi)T+\operatorname{diag}(c_L,c_R),\\
\mathbf F_s&=-K_s\mathbf d-C_s\dot{\mathbf d}.
\end{aligned}
```

這些矩陣中的 $k$ 與 $c$ 值為輪端速率。模態項將左右側的力量耦合起來。

<!-- section: model.FR.half_2dof -->
#### 路面上的車身起伏與俯仰 · 縱向半車，2 自由度

此模型代表車輛的一側。車身具有起伏 $z$、俯仰 $\theta$、彈簧下質量 $2m_s$ 以及俯仰轉動慣量 $I_y$。前後連接點位移分別為 $z_{s,F}=z+l_f\theta$ 與 $z_{s,R}=z-l_r\theta$，其中 $l_f$ 與 $l_r$ 為重心到前後軸的距離。懸吊基座跟隨路面，即 $z_{b,i}=z_{r,i}$。

```math
T_y=2(m_s+\mu)a_xh_\mathrm{CG},
\qquad
\begin{aligned}
2m_s\ddot z&=F_{s,F}+F_{s,R},\\
I_y\ddot\theta&=l_fF_{s,F}-l_rF_{s,R}+T_y.
\end{aligned}
```

此處 $\mu$ 為每端的參考非彈簧下質量。正向縱向加速度 $a_x$ 貢獻抬頭力矩（nose-up moment），制動則貢獻點頭力矩（nose-down moment）。

<!-- section: model.FR.half -->
#### 具備輪胎與抗俯仰幾何的車身起伏與俯仰 · 縱向半車，4 自由度

此模型代表車輛的一側。車身具有起伏 $z$、俯仰 $\theta$、彈簧下質量 $2m_s$ 以及俯仰轉動慣量 $I_y$。其連接點位移為 $z_{s,F}=z+l_f\theta$ 與 $z_{s,R}=z-l_r\theta$。兩個非彈簧下質量 $\mu$ 分別在 $z_{u,F}$ 與 $z_{u,R}$ 處運動，並定義懸吊基座 $z_{b,i}=z_{u,i}$。

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

前後抗俯仰比率 $\alpha_f,\alpha_r$ 設定了幾何力 $G_F,G_R$。這些力作用於每個車輪，並以相反方向反作用於車身。帶正負號的加速度 $a_x$ 在煞車時會同時反轉慣性矩與幾何力。

<!-- section: model.FR.rates -->
**懸吊速率與力。** 在每一端 $i\in\{F,R\}$，起伏與直接角隅（corner）元件以並聯方式運作。令 $r_{h,i}$ 與 $r_c$ 為其運動比率。速率總和與力量為

```math
\begin{aligned}
k_i &= k_{h,i} r_{h,i}^{\,2}+k_{\mathrm{corner},i}r_c^{\,2},\\
c_i &= c_{h,i} r_{h,i}^{\,2}+c_{\mathrm{corner},i}r_c^{\,2},\\
F_{s,i}&=-k_i(z_{s,i}-z_{b,i})-c_i(\dot z_{s,i}-\dot z_{b,i}).
\end{aligned}
```

<!-- section: model.full.rigid -->
#### 具有四輪質量的剛性車身 · 全車，7 自由度

車身座標為起伏 $z$、側傾 $\phi$ 與俯仰 $\theta$。其餘四個座標為非彈簧下車輪位移 $z_{u,i}$，質量為 $m_{u,i}$。令 $t_f,t_r$ 分別代表前後輪距， $l_f,l_r$ 為重心到前後軸的距離。車身連接點位移為

```math
\begin{aligned}
z_{s,FL/FR}&=z+l_f\theta\ \pm\tfrac{t_f}{2}\phi,\\
z_{s,RL/RR}&=z-l_r\theta\ \pm\tfrac{t_r}{2}\phi.
\end{aligned}
```

左側使用加號。令 $B$ 為由這些表達式表示的矩陣，使得 $\mathbf z_s=B[z,\phi,\theta]^\mathsf T$。總質量為 $m$，彈簧下質量 $m_s=m-\sum_i m_{u,i}$，車身轉動慣量為 $I_{xx},I_{yy}$，方程式為

```math
\begin{aligned}
\operatorname{diag}(m_s,I_{xx},I_{yy})
\begin{bmatrix}\ddot z\\\ddot\phi\\\ddot\theta\end{bmatrix}
&=B^\mathsf T(\mathbf F_s-\mathbf G)+[0,T_x,T_y]^\mathsf T,\\
m_{u,i}\ddot z_{u,i}&=F_{t,i}-F_{s,i}+G_i.
\end{aligned}
```

相同的側傾角適用於前後軸。$B^\mathsf T$ 利用其力臂將角隅力轉換為垂直力、側傾力矩與俯仰力矩。

<!-- section: model.full.flex -->
#### 前後軸由車身扭轉耦合的彈簧下車身 · 全車，8 自由度

車身座標為起伏 $z$、前側傾與後側傾 $\phi_f,\phi_r$，以及俯仰 $\theta$。四個非彈簧下位移 $z_{u,i}$（質量為 $m_{u,i}$）完成了模型。給定輪距 $t_f,t_r$ 與軸距 $l_f,l_r$，角隅位移為

```math
\begin{aligned}
z_{s,FL/FR}&=z+l_f\theta\ \pm\tfrac{t_f}{2}\phi_f,\\
z_{s,RL/RR}&=z-l_r\theta\ \pm\tfrac{t_r}{2}\phi_r.
\end{aligned}
```

左側使用加號。定義 $B$ 使得 $\mathbf z_s=B[z,\phi_f,\phi_r,\theta]^\mathsf T$，前輪分擔比例 $\lambda=l_r/(l_f+l_r)$，且彈簧下質量 $m_s=m-\sum_i m_{u,i}$。車架彈簧 $k_\tau$ 與阻尼器 $c_\tau$ 以扭矩 $T_\tau$ 對抗相對軸側傾：

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

扭轉力矩在前後側傾座標上以相反方向作用。求解器使用 $\lambda$ 同時分配總側傾轉動慣量與橫向慣性矩。

<!-- section: model.full.rates -->
**懸吊速率與力。** 依照 FL、FR、RL、RR 的車輪順序，令 $\mathbf d=\mathbf z_s-\mathbf z_u$。通道矩陣 $T$ 將四個形變量映射到十個通道。其前六行分別代表前後起伏、前後側傾、中央側傾與翹曲（warp）。單位矩陣區塊 $I_4$ 代表四個直接角隅通道。

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

每個通道使用來自 $\mathbf k$ 與 $\mathbf c$ 的各自輪端剛度與阻尼。轉置矩陣將通道力映射回車輪角隅。

**慣性矩與幾何轉移。** 令 $\lambda=l_r/(l_f+l_r)$，$L=l_f+l_r$，且 $h_\mathrm{CG}$ 為重心高度。慣性矩為 $T_x=m a_yh_\mathrm{CG}$ 與 $T_y=m a_xh_\mathrm{CG}$。滾轉中心高度 $h_{\mathrm{RC},f},h_{\mathrm{RC},r}$ 與抗俯仰比率 $\alpha_f,\alpha_r$ 給出了作用在車輪上的幾何力：

```math
\begin{aligned}
R_f&=m a_y\lambda h_{\mathrm{RC},f}/t_f,
&R_r&=m a_y(1-\lambda)h_{\mathrm{RC},r}/t_r,\\
A_f&=\alpha_f T_y/(2L),&A_r&=-\alpha_r T_y/(2L),\\
\mathbf G&=[R_f+A_f,\,-R_f+A_f,\,R_r+A_r,\,-R_r+A_r]^\mathsf T.
\end{aligned}
```

車輪接收 $G_i$，車身則在同一角隅接收相反的反作用力。

<!-- section: model.tires -->
**輪胎力與輪載荷。** 在每個車輪處，路面位置 $z_{r,i}$ 與速度 $\dot z_{r,i}$ 透過有效輪胎剛度 $k_{t,i}$ 與阻尼 $c_{t,i}$ 發揮作用。輪胎力是相對於靜態載荷 $N_{i,0}$ 的變化量：

```math
F_{t,i}=k_{t,i}(z_{r,i}-z_{u,i})+c_{t,i}(\dot z_{r,i}-\dot z_{u,i}),
\qquad N_i=N_{i,0}+F_{t,i}.
```

計算出的負載荷表示已超出線性接觸假設的範圍。求解器仍會套用相同的輪胎定律。

<!-- section: input.road -->
所選目標決定了哪些車輪接收路面輸入。對於車輪 $i$，在本地時間 $\tau_i=t-t_0-\Delta_i$ 處評估輪廓，其中 $t_0$ 為輸入開始時間，$\Delta_i$ 為車輪的延遲（除非套用了後軸延遲，否則為零）。未選取的車輪以及 $\tau_i<0$ 的車輪其路面位移與速度均為零。

<!-- section: input.rear_delay -->
當啟用後軸延遲時，前輪的 $\Delta_i=0$，後輪則以車速 $v_\mathrm{veh}$ 使用 $\Delta_i=(l_f+l_r)/v_\mathrm{veh}$。否則所有延遲均為零。

<!-- section: input.bump -->
#### 升高餘弦路面突起（Raised cosine road bump）

輸入的高度 $H$ 是持續時間為 $T_b$ 的突起之峰值。其高度和斜率在兩端皆為零。在 $0\le\tau_i\le T_b$ 之外，路面位移和速度均為零。

```math
z_{r,i}(t)=\frac H2\left[1-\cos\left(\frac{2\pi\tau_i}{T_b}\right)\right],
\qquad
\dot z_{r,i}(t)=\frac{\pi H}{T_b}\sin\left(\frac{2\pi\tau_i}{T_b}\right).
```

<!-- section: input.step -->
#### 路面階躍（Road step）

路面跳變輸入的高度 $H$ 並保持該高度。求解器提供零路面速度，並省略了通常伴隨瞬時跳變的理想衝激。

```math
z_{r,i}(t)=
\begin{cases}
0,&\tau_i<0,\\
H,&\tau_i\ge 0,
\end{cases}
\qquad \dot z_{r,i}(t)=0\quad\text{在求解器中}.
```

<!-- section: input.sine -->
#### 正弦波路面

路面以振幅 $H$ 和頻率 $f$（以赫茲為單位）圍繞零點振盪。從 $\tau_i=0$ 開始，其位移和速度為

```math
\boxed{z_{r,i}(t)=H\sin(2\pi f\tau_i)},
\qquad
\dot z_{r,i}(t)=2\pi fH\cos(2\pi f\tau_i).
```

週期為 $1/f$。阻尼器對速度作出響應，彈簧則對位移作出響應。

<!-- section: input.random -->
#### 種子隨機路面（Seeded random road）

求解器從均值為零、標準差為 $H$ 的常態分佈中抽取高度 $h_j$，將第一個節點設為 $h_0=0$，並使用輸入的隨機種子來重現該序列。節點間距為 $\Delta t_r$。對於 $j=\lfloor\tau_i/\Delta t_r\rfloor$ 與 $u=\tau_i/\Delta t_r-j$，求解器進行線性內插：

```math
z_{r,i}(t)=(1-u)h_j+u h_{j+1},
\qquad
\dot z_{r,i}(t)=\frac{h_{j+1}-h_j}{\Delta t_r}.
```

選取的車輪在各自的本地時間對同一序列進行取樣。

<!-- section: input.iso_a -->
#### ISO 8608 A 級路面

所選級別使用參考位移功率譜密度 $G_0=16\times10^{-6}\,\mathrm{m^3}$。

<!-- section: input.iso_b -->
#### ISO 8608 B 級路面

所選級別使用參考位移功率譜密度 $G_0=64\times10^{-6}\,\mathrm{m^3}$。

<!-- section: input.iso_c -->
#### ISO 8608 C 級路面

所選級別使用參考位移功率譜密度 $G_0=256\times10^{-6}\,\mathrm{m^3}$。

<!-- section: input.iso_profile -->
求解器由單邊位移功率譜密度合成空間軌跡，其參考頻率為 $n_0=0.1\,\mathrm{cycles/m}$，頻段為 $0.011\le n\le2.83\,\mathrm{cycles/m}$。軌跡在 $L_p=4096\,\mathrm m$ 後重複，頻點間距為 $\Delta n=1/L_p$，且獨立相位 $\psi_j$ 由所選種子抽出。

```math
G(n)=G_0(n/n_0)^{-2},\qquad
h(s)=\sum_j\sqrt{2G(n_j)\Delta n}\cos(2\pi n_j s+\psi_j).
```

求解器從軌道中減去 $h(0)$，並透過取樣的高度與斜率使用三次埃爾米特內插（cubic Hermite interpolant）$P(s)$。車速 $v_\mathrm{veh}$ 將距離轉換為時間：

```math
z_{r,i}(t)=P(v_\mathrm{veh}\tau_i),\qquad
\dot z_{r,i}(t)=v_\mathrm{veh}\,P'(v_\mathrm{veh}\tau_i).
```

<!-- section: input.acceleration -->
加速度從 $t_0$ 開始施加，使用持續時間為 $T_a$ 的平滑斜坡函數。定義 $u_a=\min(1,\max(0,(t-t_0)/T_a))$。比例因子 $r(t)$ 從零上升到一，並在斜坡結束後保持為一。

```math
r(t)=\frac{1-\cos(\pi u_a)}2.
```

<!-- section: input.cornering -->
#### 橫向加速度

```math
a_y(t)=a_{y,\mathrm{set}}\,r(t),\qquad a_x(t)=0,\qquad z_{r,i}(t)=0.
```

正的 $a_y$ 施加一個抬高左側的慣性矩。負輸入會反轉其正負號。

<!-- section: input.accelerate -->
#### 縱向加速度

```math
a_x(t)=a_{x,\mathrm{set}}\,r(t),\qquad a_y(t)=0,\qquad z_{r,i}(t)=0.
```

正的 $a_x$ 代表加速，並產生抬頭慣性矩。負的 $a_x$ 代表煞車，並產生點頭慣性矩。

<!-- section: input.bump_cornering -->
#### 同時施加橫向加速度

```math
a_y(t)=a_{y,\mathrm{set}}\,r(t),\qquad a_x(t)=0.
```

升高餘弦路面突起與橫向斜坡在相同的輸入開始時間開始。橫向斜坡透過所選模型的慣性與幾何項起作用，同時突起使所選的路面基座發生位移。

<!-- section: input.combined -->
#### 同時施加縱向與橫向加速度

```math
a_x(t)=a_{x,\mathrm{set}}\,r(t),\qquad
a_y(t)=a_{y,\mathrm{set}}\,r(t).
```

兩個加速度斜坡在 $t_0$ 同時開始。路面突起也在 $t_0$ 時於選定的前輪上開始，並在選定的後輪上遵循任何啟用的後軸延遲。求解器將路面力與加速度力同時施加。