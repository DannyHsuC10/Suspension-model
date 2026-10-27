# suspension_geo 資料夾說明

`suspension_geo` 依分析視角分成 `fv`（前視）、`sv`（側視）、`tv`（上視）及 `we3D`（3D 輪端）。同一類的互動程式、mapping、分析程式與結果放在同一個子資料夾，車輛參數與共用幾何工具保留在根目錄。

整理日期：2026-10-25

## 資料夾分類

```text
suspension_geo/
├── car.py                           # 各視角共用的車輛與前後懸吊參數
├── geometry.py                      # 共用 Point、Link、RigidBody2D 等物件
├── Geometric_Analysis.py            # 前視求解器與側視共用的幾何運算
├── visualization.py                 # 共用 Matplotlib 繪圖工具
├── data/
│   └── data_run15_selected.csv       # FV、SV 共用的 Run 15 輸入資料
├── fv/
│   ├── sus_fv_v5.py                 # 前視互動圖
│   ├── sus_fv_maping.py             # 前視 mapping
│   ├── rc_camber_map.py             # mapping 結果繪圖
│   ├── optimize_fv_geometry.py      # RC 高度 / FVSA 離散最佳化
│   ├── optimization_results/        # 最佳化 cost 圖、歷程與最佳 mapping
│   ├── lab.py                       # 直接求解範例
│   ├── run15_pose_analysis.py        # Run 15 前視分析
│   ├── suspension_kinematics_data.pkl
│   ├── data_run15_geometry.csv
│   └── data_run15_geometry.png
├── sv/
│   ├── sus_sv_v4.py                 # 側視互動圖
│   ├── run15_sv_anti_analysis.py     # Run 15 anti 分析
│   ├── data_run15_sv_anti.csv
│   └── data_run15_sv_anti.png
├── tv/
│   └── sus_tv_v2.py                 # 上視轉向幾何
└── we3D/
    └── sus_3Dwe_v3.py               # 3D 輪端轉向與力臂分析
```

新增 mapping 或分析結果時，放進對應視角資料夾；多個視角共用的原始輸入放在 `data/`。既有結果隨檔案搬移保留，整理資料夾本身不會重新計算結果。

## 執行方式

從專案根目錄 `D:\Danny\DH3868\Suspension-model` 執行：

```powershell
python -m suspension_geo.fv.sus_fv_v5
python -m suspension_geo.fv.sus_fv_maping
python -m suspension_geo.fv.rc_camber_map
python -m suspension_geo.fv.optimize_fv_geometry
python -m suspension_geo.fv.lab
python -m suspension_geo.fv.run15_pose_analysis
python -m suspension_geo.sv.sus_sv_v4
python -m suspension_geo.sv.run15_sv_anti_analysis
python -m suspension_geo.tv.sus_tv_v2
python -m suspension_geo.we3D.sus_3Dwe_v3
```

也可直接執行腳本，例如 `python suspension_geo/fv/sus_fv_v5.py`，或進入 `fv/` 後執行 `python sus_fv_v5.py`。Python 環境需要 `numpy`、`scipy`、`pandas` 與 `matplotlib`。

FV、SV 的 Run 15 分析預設都讀取 `data/data_run15_selected.csv`，並把 CSV 與 PNG 輸出到各自的 `fv/` 或 `sv/`，不受執行時所在目錄影響。可用 `--input`、`--output`、`--figure` 指定其他路徑。FV mapping 的 PKL 固定寫入 `fv/suspension_kinematics_data.pkl`，結果繪圖也從相同位置讀取。

## 前視分析目的

這個資料夾的目的是把前視懸吊幾何從車輛參數轉成可計算、可視覺化的模型。它可以用來回答幾類問題：

- 靜態前視幾何長什麼樣子。
- 下控制臂角度改變時，輪胎接地點與羊角如何運動。
- 左右輪不同 travel 對 roll angle、camber、roll center 的影響。
- 給定 heave/roll 或左右輪行程時，反算對應的懸吊姿態。
- 批次掃描整個行程範圍，產生 RC/camber map。

## 核心資料流

```text
car.py
  定義車輛、輪胎、前懸吊設計參數
      |
Geometric_Analysis.py
  由參數計算靜態幾何點，建立左右懸吊物件
      |
geometry.py
  提供 Point、Link、RigidBody2D、Wheel 等基礎幾何物件
      |
visualization.py
  將幾何物件轉成 Matplotlib 圖形
      |
fv/sus_fv_v5.py
  互動式前視圖，滑桿調整左右 lower arm angle
      |
fv/sus_fv_maping.py
  批次掃描左右角度，輸出 fv/suspension_kinematics_data.pkl
      |
fv/rc_camber_map.py
  讀取 pkl，畫出 camber surface 與 roll center map
```

## 檔案總覽

| 檔案 | 內容與目的 |
| --- | --- |
| `car.py` | 定義 `Car` 車輛/輪胎參數物件，以及 `SuspensionF` 前懸吊幾何與分析設定物件。 |
| `geometry.py` | 基礎 2D 幾何物件。包含 `Point`、`Link`、`RigidBody2D`、`Polygon`、`Wheel`、`Body`。 |
| `Geometric_Analysis.py` | 前視懸吊幾何與運動學核心。負責算靜態點位、上下控制臂、羊角、instant center、roll center、camber、行程反算等。 |
| `visualization.py` | Matplotlib 視覺化封裝。把 `Point`、`Link`、`Polygon` 畫成可更新的 plot object。 |
| `fv/sus_fv_v5.py` | 互動式前視懸吊圖。用兩個 slider 控制左右 lower arm angle，並即時更新幾何與 roll center。 |
| `sv/sus_sv_v4.py` | 互動式前後軸側視懸吊圖。前後輪中心相隔 `Car.l`，使用 `SuspensionF` 與 `SuspensionR` 幾何；兩個滑桿分別控制前、後輪的 Al 高度並更新當下 IC。後軸幾何沿車身縱向反向，後輪 IC 位於後輪後方，前輪 IC 位於前輪前方。 |
| `tv/sus_tv_v2.py` | 上視轉向幾何，以轉向機位移控制左右輪轉向並顯示掃描曲線。 |
| `we3D/sus_3Dwe_v3.py` | 3D 輪端轉向圖，顯示接地點軌跡、轉向力臂與輪胎傾角。 |
| `fv/sus_fv_maping.py` | 批次掃描左右 lower arm angle，計算 heave、roll、左右 camber、RC 位置與各點座標，輸出 `.pkl`。 |
| `fv/rc_camber_map.py` | 讀取同資料夾的 `suspension_kinematics_data.pkl`，畫出 camber 與 roll center map。 |
| `fv/lab.py` | 直接求解範例。展示如何用 theta、wheel travel、heave/roll 姿態反推 camber 與 roll center。 |
| `fv/run15_pose_analysis.py` | 用 Run 15 姿態資料分析前視 camber、roll center 與 camber gain，輸出同資料夾的 `data_run15_geometry.csv/.png`。 |
| `sv/run15_sv_anti_analysis.py` | 用 Run 15 資料分析側視 IC、pitch center 與 anti，輸出同資料夾的 `data_run15_sv_anti.csv/.png`。 |
| `fv/suspension_kinematics_data.pkl` | `sus_fv_maping.py` 產生的幾何掃描資料。 |
| `data/data_run15_selected.csv` | FV、SV 共用的 Run 15 原始輸入。 |

## 主要概念

| 名稱 | 在此資料夾中的意義 |
| --- | --- |
| `kg` | Kingpin ground 或 kingpin 相關下方點，用於建立 kingpin 幾何。 |
| `kua` | Kingpin/upright 上方接點。 |
| `lak` | Lower arm 與 upright 下方接點。 |
| `gnd` | 輪胎接地點。 |
| `cpb_rl`, `cpb_ru` | 車體側控制臂接點。 |
| `IC` | Instant Center，上下控制臂延長線交點。 |
| `ICo` | 設計目標或初始 instant center 參考點。 |
| `RC` | Roll Center，左右 wheel force line 的交點。 |
| `FVSA` | Front View Swing Arm，用於前視懸吊幾何設計。 |
| `KPI` | Kingpin Inclination，主銷內傾角。 |
| `scrub_radius` | 輪胎接地中心與 kingpin 地面投影的水平距離。 |

### Camber 正負定義

FV mapping 與 `SuspensionDirectSolver` 共用 `SuspensionSide.calc_camber_angle()`：以輪胎底邊 `wheel_pts[0] → wheel_pts[1]` 相對當下左右輪接地點連線的角度計算 camber，並校正左輪的鏡像方向。

- 正 camber：輪胎上端向車外傾。
- 負 camber：輪胎上端向車內傾。
- 零 camber：輪胎底邊與地面連線平行，輪胎相對地面直立。

地面基準會隨左右輪行程改變。計算使用 `atan2` 保留方向，角度單位為 deg。更新前產生的結果不含正負方向；重新執行 `fv/sus_fv_maping.py` 可更新 mapping 的 PKL。`fv/run15_pose_analysis.py` 與 `fv/lab.py` 也使用相同正負定義，既有 Run 15 CSV/PNG 需重新執行分析才會更新。

## `car.py`

`Car` 只放車輛與輪胎資料；`SuspensionF` 與 `SuspensionR` 分別放前、後懸吊硬點、側視 anti 設定與幾何分析的顯示/掃描設定。

| 物件 | 主要內容 |
| --- | --- |
| `Car` | 軸距、重心位置、前後輪距、質量、重力、車底高度、車體截面、輪胎參數與負載輪胎半徑。 |
| `SuspensionF` / `SuspensionR` | 各軸的前視 `scrub_radius`、`kpi`、`h_rc`、`fvsa`、`Au`/`Al`、車身側硬點，以及側視 caster、mechanical trail、anti-dive/anti-lift 與顯示設定。側視圖中的前後輪中心距使用 `Car.l`。 |

前視分析會同時建立兩個物件，再交給幾何 helper：

```python
from suspension_geo.car import Car, SuspensionF
from suspension_geo import Geometric_Analysis as ga

car = Car()
suspension_f = SuspensionF()
sus_helper = ga.SusGeometryHelper(car, suspension_f)
```

`SusGeometryHelper` 從 `Car` 讀取車輛/輪胎資料，從 `SuspensionF` 讀取懸吊硬點與幾何目標。

`Car` 的主要參數包含：

| 參數 | 說明 |
| --- | --- |
| `l`, `lf`, `lr` | 軸距與重心到前/後軸距離。 |
| `tf`, `tr` | 前/後輪距。front view 目前主要使用 `tf`。 |
| `h_cog` | 重心高度。 |
| `m`, `w` | 車重與重量。 |
| `bottom` | 車底距地高度。 |
| `body_face_f` | 前視圖使用的車體截面 polygon。 |
| `free_radius`, `load_Radius`, `tire_width` | 輪胎自由半徑、負載半徑與胎寬。 |

`SuspensionF` 與 `SuspensionR` 的參數名稱都不加 `_f` 或 `_r` 後綴，由物件本身表示軸別。側視圖使用前軸為正 x 方向，後軸沿 x 方向反轉；輪心 x 間距固定為 `Car.l`。兩個側視滑桿分別改變前後軸 Al 高度。後軸設定目前的預設值與前軸相同，可在 `SuspensionR` 中獨立調整。

## `geometry.py`

這個檔案是幾何物件的底層工具。

| 類別 | 功能 |
| --- | --- |
| `Point` | 2D 點，提供 `x`、`y`、`move()`、`mirror_x()`。 |
| `Link` | 兩點連桿，提供 vector、length、angle、direction、midpoint。 |
| `RigidBody2D` | 用兩個參考點更新剛體姿態，適合讓 upright 與輪胎點一起旋轉/平移。 |
| `Polygon` | 多點 polygon 基底。 |
| `Wheel` | 繼承 `Polygon`，表示輪胎外形。 |
| `Body` | 繼承 `Polygon`，表示車身外形。 |

## `Geometric_Analysis.py`

這是 front view 幾何計算的核心。

### 幾何函式

| 函式 | 功能 |
| --- | --- |
| `rotate_point_about(center, length, theta)` | 給定中心、長度與角度，計算旋轉後的點。 |
| `circle_intersection(c1, r1, c2, r2)` | 求兩圓交點，用於由上下臂長度求解羊角點。 |
| `simulate_side_theta(sus, theta)` | 給定單側懸吊與下臂角度，計算新的 `kua`、`lak`。 |
| `update_upright(sus, kua_new, lak_new)` | 根據新的上下羊角點更新接地點位置。 |
| `calculate_theta_limits(sus, target_travel=0.025)` | 由目標 wheel travel 估算 lower arm angle 可用範圍。 |

### 主要類別

| 類別 | 功能 |
| --- | --- |
| `SusGeometryHelper` | 從 `Car` 與 `SuspensionF` 參數建立靜態點位，並提供線交點、IC 更新、RC 計算。 |
| `SuspensionSide` | 單側懸吊物件。包含 point、link、force line、wheel、upright body，並可鏡像生成左側。 |
| `SuspensionDirectSolver` | 直接求解器。可用 theta、左右 wheel travel 或 heave/roll 姿態反算 camber 與 RC。 |

## 視覺化腳本

### `sus_fv_v5.py`

這是最直觀的前視圖工具。執行後會開啟 Matplotlib 視窗，包含：

- 車體截面。
- 左右輪胎。
- 上下控制臂。
- upright/kingpin。
- instant center 與 force line。
- CG 與 roll center。
- 兩個 slider：`Right Arm Angle`、`Left Arm Angle`。
- 圖形上方的即時數值面板：左右輪有號 camber（deg）、roll center X/Y（mm）、roll（deg）與 heave（mm）。

滑動任一 slider 時，幾何與面板會一起更新，開啟視窗時也會顯示初始值。計算與 FV mapping 共用相同定義：camber 上端向外為正；roll 為左輪指向右輪的接地點連線相對水平的角度；heave 為左右接地點相對初始高度的平均位移。RC 無有限交點時顯示 `N/A` 並隱藏動態 RC 點。

執行方式：

```powershell
cd D:\Danny\DH3868\Suspension-model
python -m suspension_geo.fv.sus_fv_v5
```

### `sus_fv_maping.py`

這個腳本做批次掃描。流程是：

1. 建立 `Car`、`SusGeometryHelper`。
2. 先建立右側懸吊，再鏡像出左側。
3. 用 `calculate_theta_limits()` 找出約 ±25 mm travel 的角度範圍。
4. 建立 `theta_left_array` 與 `theta_right_array`。
5. 雙重迴圈掃描左右角度。
6. 對每個狀態計算：
   - left/right travel
   - heave
   - roll angle
   - left/right camber
   - roll center
   - IC、GND、KUA、LAK、wheel points
7. 輸出 `suspension_kinematics_data.pkl`。
8. import `rc_camber_map` 直接畫圖。

目前掃描解析度是：

```python
steps = 30
```

所以總共會產生 `30 x 30 = 900` 筆幾何狀態。

### `rc_camber_map.py`

這個腳本讀取 `suspension_kinematics_data.pkl`，再把一維 dataset 還原成 2D grid。它會產生兩組圖：

| 圖 | 內容 |
| --- | --- |
| Camber 3D surface | 以 heave 與 roll angle 為輸入軸，分別畫 left/right camber 曲面。 |
| Roll Center map | 畫 RC X/Y migration cloud，以及 RC height 隨左右 wheel travel 變化的 3D 曲面。 |

### `optimize_fv_geometry.py`

用 SciPy `differential_evolution` 的 `integrality` 限制，在指定尺寸格點上最佳化 RC 高度與 FVSA。起點為 RC 高度 **35 mm**、FVSA **1000 mm**；RC 高度範圍 **−5～50 mm**、步距 **1 mm**，FVSA 範圍 **750～2500 mm**、步距 **50 mm**。整個搜尋過程都使用這些離散尺寸，關閉連續尺寸的 polishing。

每個候選設計會重新建立懸吊硬點與行程角度範圍，依 `front_view_target_travel`（預設每輪約 ±25 mm）取樣完整 mapping，預設左右各 30 點，共 900 個姿態。左右輪可獨立求解，因此程式只需解出單輪各角度，再鏡像並組合出完整網格；測試會核對它與既有 FV mapping 的結果一致。幾何無解或 IC/RC 無有限交點時，整組設計列為無效，避免以不完整的 mapping 降低 cost。

RC cost 的固定目標是 **`(X, Y) = (0, y_best) = (0, 33) mm`**，由 `Y_BEST_MM = 33.0` 定義。`y_best` 不參與最佳化，也不隨起點或候選設計的靜態 RC 高度改變；最佳化起點仍為 RC 高度 35 mm。令每個 mapping 點到固定目標的距離為 `r`：

```text
y_best = 33 mm  # 固定常數
r² = (x_mm - 0.0)² + (y_mm - y_best)²
RC_distance_raw = mean(k_rc × r²)  # 原本的徑向距離目標，mm²
k_rc = 3 if r > RC_LIMIT_MM else 1
RC_height_raw = mean((y_mm - y_best)²)  # 獨立的純高度目標，mm²

Camber_raw = k_camber × mean(camber_deg²)  # deg²，左右輪全部取樣一起平均
k_camber = 3 if 任一輪、任一行程點 |camber| > 2 deg else 1

RC_distance_scale = 起始設計的 RC_distance_raw
RC_height_scale = 起始設計的 RC_height_raw
Camber_scale = 起始設計的 Camber_raw
RC_distance cost = RC_distance_raw / RC_distance_scale
RC_height cost = RC_height_raw / RC_height_scale
RC cost = (rc_distance_weight × RC_distance cost + rc_height_weight × RC_height cost)
          / (rc_distance_weight + rc_height_weight)
Camber cost = Camber_raw / Camber_scale

w_rc = rc_weight / (rc_weight + camber_weight)
w_camber = camber_weight / (rc_weight + camber_weight)
Total cost = w_rc × RC cost + w_camber × Camber cost
```

徑向距離、純高度、camber 三個正規化分母在每次執行開始時，以 RC 高度 **35 mm**、FVSA **1000 mm** 的完整 mapping 計算一次，之後保持固定。一般情況下起點三項 cost 都是 **1**；小於 1 表示優於起點，大於 1 表示劣於起點。若某項起始誤差已經為零，改用該項容許尺度的平方作分母，避免除以零。

RC 內部預設 `rc_distance_weight = 1`、`rc_height_weight = 5`，即徑向距離占 **1/6**、純高度目標占 **5/6**。高度誤差具有獨立尺度，避免較大的 X 遷移把 Y 目標淹沒，造成最佳化一味提高 RC 來減少 X 遷移。純高度目標直接量測整個 mapping 的 `(Y−33)²`，不會因 X 改變圓形懲罰的觸發點數而降低。

外部預設 `rc_weight = camber_weight = 1`，RC 與 camber 係數各為 **50%**，起始 total cost 為 **1**。預設 total cost 等價於 `(1/12) × RC_distance cost + (5/12) × RC_height cost + 0.5 × Camber cost`。原始平方誤差、三項正規化 cost、加權貢獻與固定分母都會印出並儲存。圖上虛線另外標示 RC 高度的貢獻。

目前 `RC_LIMIT_MM = 50.0`，RC 懲罰圓心固定為 `(0, 33) mm`；半徑由這個常數設定，獨立於正規化分母。原本的徑向距離目標完整保留超標 3 倍規則，另外加入不受 X 懲罰觸發影響的純高度目標。3 倍門檻是軟懲罰，最佳解仍可能包含超過門檻的行程點。Camber 目標使用角度（deg）接近零的平方誤差；camber gain（deg/mm）需另行定義目標。

33 mm 是全 mapping 的 RC 高度目標，設計的靜態 RC 高度仍是最佳化變數。終端與 JSON 另列出 `rc_y_mean_mm`、`rc_y_error_rms_mm`、`rc_y_min_mm`、`rc_y_max_mm`，分別表示 mapping 的平均 RC 高度、相對 33 mm 的高度 RMS 誤差與高度範圍。

差分演化完成後，預設再用 SciPy `brute` 檢查全部 **56 × 36 = 2,016** 組尺寸，確認此 mapping 與離散尺寸範圍中的最佳解。重複尺寸使用快取。程式會分別列印 DE 的停止狀態與完整格點驗證狀態。

```powershell
python -m suspension_geo.fv.optimize_fv_geometry
# 批次執行，存圖而不開啟視窗
python -m suspension_geo.fv.optimize_fv_geometry --no-show
# 進一步加強 RC 高度接近 33 mm：RC 內部距離:純高度 = 1:10
python -m suspension_geo.fv.optimize_fv_geometry --rc-height-weight 10
# 提高整個 RC 目標相對於 camber 的重要性：RC:camber = 3:1
python -m suspension_geo.fv.optimize_fv_geometry --rc-weight 3
# 提高 camber 重要性：RC / camber 係數為 1/3、2/3
python -m suspension_geo.fv.optimize_fv_geometry --rc-weight 1 --camber-weight 2
```

輸出存於 `fv/optimization_results/`：

- `cost_history.png`：候選 cost、目前最佳 cost、RC/camber 分項與尺寸變化圖；橫軸為實際計算過的不重複設計數。
- `optimization_history.csv`：每組設計的原始誤差、正規化 cost、加權貢獻、尺寸、階段與當時最佳解。
- `optimization_summary.json`：起點、最佳解、距離/高度/camber 固定正規化分母、兩層權重比例、RC 高度診斷、停止狀態與全格點驗證結果。
- `optimal_mapping.csv`：最佳設計完整 mapping 的 RC、camber、左右行程與 roll。

可用 `--rc-weight`、`--camber-weight` 調整權重，`--output-dir` 改輸出位置，`--steps` 改 mapping 解析度。RC 目標固定為 `(0, 33) mm`，已移除隨候選設計改變目標的 `--reference` 選項。`--camber-penalty point` 可改成只加重超標行程點。`--no-grid-check` 可略過完整格點驗證，此時不保證離散全域最佳。程式以新物件計算，不會將最佳尺寸自動寫回 `car.py` 或覆蓋既有 mapping PKL。

演算法參考：[SciPy differential_evolution](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html)。

### `lab.py`

`lab.py` 是非圖形互動的求解範例，展示 `SuspensionDirectSolver` 的三種用法：

| 方法 | 用途 |
| --- | --- |
| `solve_by_thetas(theta_l, theta_r)` | 已知左右 lower arm angle，直接求 heave、roll、camber、RC。 |
| `solve_by_pose(target_heave_m, target_roll_deg)` | 已知車身 heave/roll，反算左右 lower arm angle，再求 camber、RC。 |
| `solve_by_travels(l_travel_m, r_travel_m)` | 已知左右 wheel travel，反算幾何狀態。 |

## 輸出資料格式

`suspension_kinematics_data.pkl` 是一個 list，每筆資料是一個 dict。每筆狀態包含：

| key | 說明 |
| --- | --- |
| `travel_left`, `travel_right` | 左右輪行程，單位 m。 |
| `heave` | 左右輪平均行程，單位 m。 |
| `roll_deg` | 左右接地點連線相對水平的角度，單位 deg。 |
| `camber_left_deg`, `camber_right_deg` | 左右輪有號 camber，單位 deg；輪胎上端向外為正、向內為負。 |
| `theta_left`, `theta_right` | 左右 lower arm angle，單位 rad。 |
| `rc` | roll center 座標 `[x, y]`，單位 m。 |
| `ic_left`, `ic_right` | 左右 instant center 座標。 |
| `gnd_left`, `gnd_right` | 左右輪胎接地點座標。 |
| `kua_left`, `kua_right` | 左右 upright 上點座標。 |
| `lak_left`, `lak_right` | 左右 lower arm/upright 下點座標。 |
| `wheel_pts_left`, `wheel_pts_right` | 左右輪胎 polygon 點座標。 |

## 建議使用順序

1. 先看 `car.py`，確認車輛與前懸吊參數。
2. 看 `Geometric_Analysis.py` 的 `calc_static_points()`，理解靜態點位如何算出。
3. 執行 `sus_fv_v5.py`，用 slider 看幾何如何動。
4. 執行 `lab.py`，測試指定 theta、travel、heave/roll 的求解結果。
5. 執行 `sus_fv_maping.py` 產生完整掃描資料。
6. 用 `rc_camber_map.py` 查看 camber 與 roll center map。

## 與整體專案的關係

`suspension_geo` 專注在各視角的幾何與運動學，不直接處理完整車輛動態力學。前視分析可作為懸吊設計前處理工具：

- 提供靜態懸吊幾何。
- 估算 roll center 高度與遷移。
- 產生 camber gain / camber map。
- 把幾何結果提供給後續的 dynamics 或 full suspension model 參考。

相較於 `dynamics_analyze/` 的彈簧阻尼時間域模擬，`suspension_geo/` 更關心「幾何點位如何移動」；相較於 `Suspension_sys/` 的全車模型，它更適合快速調整與理解懸吊幾何設計。

## 注意事項

- `Car` 與 `SuspensionF` 可分別調整；更新其中的硬點或參數後，互動圖與 mapping 分析都從這兩個物件讀取設定。
- `Geometric_Analysis.py` 的 `calc_roll_center()` 目前會 `print("rc_pos", rc_pos)`，批次掃描時終端會輸出很多 RC 座標。
- `geometry.Link` 中 `angle` property 定義了兩次，結果相同，但可日後整理。
- `SuspensionSide.update_kinematics()` 內有 `self.upper_force.p2 = self.IC` 等設定，但 `Link` 使用的是 `start/end` 屬性；若 force line 沒有如預期更新，這裡可以檢查是否應改為更新 `end`。
- `.pkl` 是 Python pickle 格式，適合本專案內部使用；若要和其他工具交換資料，建議另外輸出 CSV 或 JSON。
