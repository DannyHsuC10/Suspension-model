# suspension_geo 資料夾說明

`suspension_geo` 依分析視角分成 `fv`（前視）、`sv`（側視）、`tv`（上視）及 `we3D`（3D 輪端）。同一類的互動程式、mapping、分析程式與結果放在同一個子資料夾，車輛參數與共用幾何工具保留在根目錄。

整理日期：2026-10-25

## 資料夾分類

```text
suspension_geo/
├── car.py                           # 各視角共用的車輛與前後懸吊參數
├── analysis_settings.py             # 分析範圍、掃描解析度、繪圖與介面設定
├── geometry.py                      # 共用 Point、Link、RigidBody2D 等物件
├── Geometric_Analysis.py            # 前視求解器與側視共用的幾何運算
├── combined_camber.py              # 左右懸吊抬升與轉向的 camber 角度疊加
├── data_signals_camber.py           # CSV 四輪行程、前輪轉向與 roll → 四輪 camber
├── data_signals.csv                 # 四輪簧上/簧下位移、轉向與前後 roll 輸入
├── data_signals_camber.csv          # 逐筆四輪 camber gain 角度與 camber 輸出
├── data_signals_camber.png          # 位移、camber gain、懸吊/輪胎/總 roll 與 camber 時序圖
├── data_signals_camber_front_inside_outside.png # 依橫向加速度判斷的前輪內外側 camber 比較
├── data_signals_camber_vehicle_comparison.png # 速度、三軸加速度、前輪轉角與內外側 camber 比較
├── visualization.py                 # 共用 Matplotlib 繪圖工具
├── data/
│   └── data_run15_selected.csv       # FV、SV 共用的 Run 15 輸入資料
├── fv/
│   ├── sus_fv_v5.py                 # 前視互動圖
│   ├── sus_fv_maping.py             # 前視 mapping
│   ├── rc_camber_map.py             # mapping 結果繪圖
│   ├── optimize_fv_geometry.py      # RC 高度 / FVSA 離散最佳化
│   ├── optimization_results/        # 最佳化 cost 圖、歷程與最佳 mapping
│   ├── fv_camber_rc.py              # 直接求解 camber、camber gain 與 RC 範例
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
    ├── sus_3Dwe_v3.py               # 3D 輪端轉向與力臂分析
    └── steering_camber.py          # 輸入左右輪端轉角，計算 camber 變化與 gain
```

新增 mapping 或分析結果時，放進對應視角資料夾；多個視角共用的原始輸入放在 `data/`。既有結果隨檔案搬移保留，整理資料夾本身不會重新計算結果。

## 執行方式

從專案根目錄 `D:\Danny\DH3868\Suspension-model` 執行：

```powershell
python -m suspension_geo.fv.sus_fv_v5
python -m suspension_geo.fv.sus_fv_maping
python -m suspension_geo.fv.rc_camber_map
python -m suspension_geo.fv.optimize_fv_geometry
python -m suspension_geo.fv.fv_camber_rc
python -m suspension_geo.fv.run15_pose_analysis
python -m suspension_geo.sv.sus_sv_v4
python -m suspension_geo.sv.run15_sv_anti_analysis
python -m suspension_geo.tv.sus_tv_v2
python -m suspension_geo.we3D.sus_3Dwe_v3
python -m suspension_geo.we3D.steering_camber --left 25 --right 18
python -m suspension_geo.combined_camber --steer-left 25 --steer-right 18 --travel-left-mm 10 --travel-right-mm -5
python -m suspension_geo.data_signals_camber
```

也可直接執行腳本，例如 `python suspension_geo/fv/sus_fv_v5.py`，或進入 `fv/` 後執行 `python sus_fv_v5.py`。Python 環境需要 `numpy`、`scipy`、`pandas` 與 `matplotlib`。

FV、SV 的 Run 15 分析預設都讀取 `data/data_run15_selected.csv`，並把 CSV 與 PNG 輸出到各自的 `fv/` 或 `sv/`，不受執行時所在目錄影響。可用 `--input`、`--output`、`--figure` 指定其他路徑。FV mapping 的 PKL 固定寫入 `fv/suspension_kinematics_data.pkl`，結果繪圖也從相同位置讀取。

### 使用 data_signals.csv 計算四輪 Camber

執行 `data_signals_camber.py`，預設讀取腳本同目錄的 `data_signals.csv`，產生 `data_signals_camber.csv`、`data_signals_camber.png`、`data_signals_camber_front_inside_outside.png` 與 `data_signals_camber_vehicle_comparison.png`。路徑不受執行目錄影響；輸入檔不會被覆寫。輸出保留原始列順序與全部欄位，另外加入四輪的中間角度與最終 camber、前後軸的輪胎變形 roll 與總 roll，以及前輪內外側的輪位與對地 camber。

```powershell
python -m suspension_geo.data_signals_camber

# 使用本專案既有 Python 環境
& .\Suspension_sys\.venv\Scripts\python.exe -m suspension_geo.data_signals_camber

# 自訂初始 camber，順序為 FL、FR、RL、RR，單位 deg
python -m suspension_geo.data_signals_camber --static-camber -2 -2 -1.5 -1.5

# 自訂路徑，或只輸出 CSV
python -m suspension_geo.data_signals_camber --input suspension_geo/data_signals.csv --output suspension_geo/data_signals_camber.csv --no-plot
```

需要 `numpy`、`scipy`；繪圖另需 `matplotlib`。也可直接執行 `python suspension_geo/data_signals_camber.py`。檔案上方的 `STATIC_CAMBER_DEG` 可修改四輪初始 camber；未提供時均為 **0°**，不假設量測或車輛設定中另有靜態負 camber，也不把第一筆資料當成零點。

必要輸入欄位為 `time_s`、四輪的 `zs_FL_mm`～`zs_RR_mm` 與 `zu_FL_mm`～`zu_RR_mm`、`steer_FL_deg`、`steer_FR_deg`、`roll_front_deg`、`roll_rear_deg`。位移為 mm，角度為 deg。後輪只計算懸吊行程；若提供 `steer_RL_deg`、`steer_RR_deg`，其值需為零。本次 CSV 的後輪轉向皆為零。

計算順序：

1. 各輪依需求保留位移差 `d = zs - zu`。`zs` 與 `zu` 向上為正，因此 `d > 0` 為伸長；既有 FV 求解器以壓縮為正，輸入行程須用 `q = -d = zu - zs`。
2. 前輪使用 `SuspensionF`，後輪使用 `SuspensionR`，套用各自的輪距與負載輪胎半徑。由 `combined_camber.py` 計算有限行程造成的 FV camber 角度變化，前輪另加 `steering_camber.py` 的轉向角度變化。
3. **先把 FV camber 換回相對車身的角度。**原求解器的 camber 已相對左右接地點連線計算。若該連線角度為 `α`，左輪 FV 變化加 `α`、右輪減 `α`，才能取得車身座標下的 camber 變化，再與轉向項疊加。
4. CSV 的 `roll_front_deg`、`roll_rear_deg` 視為純懸吊幾何造成的 roll，另外以左右輪 `zu` 的高度差計算輪胎變形造成的 roll，再相加得到總 roll。依需求使用小角度近似 `φ_tire [rad] = (zu_left - zu_right) / track`，並轉成 deg；`zu` 單位為 mm，所以輪距也換成 mm。輪距讀取 `Car.tf = 1.250 m`、`Car.tr = 1.240 m`，即前軸 1250 mm、後軸 1240 mm。不扣除第一筆或平均值，同軸兩側 `zu` 相等時輪胎 roll 為零。
5. 對地 camber 使用各軸的總 roll。依 `Suspension_sys/Vehicle_status.py::body_to_corner()`，正 roll 代表左側抬高；因此左輪減總 roll，右輪加總 roll。懸吊行程仍由 `zu - zs` 計算，FV 與轉向造成的車身相對 camber gain 定義不變。

```text
tire_roll_front = (zu_FL_mm - zu_FR_mm) / 1250 × 180/π
tire_roll_rear  = (zu_RL_mm - zu_RR_mm) / 1240 × 180/π
total_roll_front = roll_front + tire_roll_front
total_roll_rear  = roll_rear  + tire_roll_rear

camber_gain_FL = FV_body_FL + steering_FL
camber_gain_FR = FV_body_FR + steering_FR
camber_gain_RL = FV_body_RL
camber_gain_RR = FV_body_RR

camber_FL = static_FL + camber_gain_FL - total_roll_front
camber_FR = static_FR + camber_gain_FR + total_roll_front
camber_RL = static_RL + camber_gain_RL - total_roll_rear
camber_RR = static_RR + camber_gain_RR + total_roll_rear
```

以上角度均為 deg，正 camber 為輪胎上端向車外、負為向車內。這裡依「角度疊加」的需求，`camber_gain` 指相對車身的**有限角度變化**，不是 deg/mm 的局部導數，也不以局部導數乘上有限行程估算角度。原 FV camber 不能直接再加減總 roll，否則會混入接地點連線傾角。

| 新增欄位（`wheel` 替換為 FL、FR、RL、RR） | 定義 |
| --- | --- |
| `relative_displacement_{wheel}_mm` | 原始 `zs - zu`，伸長正。 |
| `wheel_travel_{wheel}_mm` | 求解器使用的 `zu - zs`，壓縮正。 |
| `static_camber_{wheel}_deg` | 指定的初始 camber。 |
| `fv_camber_gain_{wheel}_deg` | 換回車身基準後，懸吊行程造成的角度變化。 |
| `steering_camber_gain_{wheel}_deg` | 轉向造成的角度變化；後輪為 0。 |
| `camber_gain_{wheel}_deg` | 上述兩項角度變化之和，相對車身。 |
| `roll_camber_{wheel}_deg` | 懸吊與輪胎總 roll 貢獻，左輪為負總 roll、右輪為正總 roll。 |
| `camber_{wheel}_deg` | 初始 camber + gain + 總 roll 貢獻，相對水平地面。 |
| `fv_contact_line_front_deg` / `fv_contact_line_rear_deg` | FV 求解器接地點連線的傾角，僅供換算基準與檢查。 |
| `tire_roll_front_deg` / `tire_roll_rear_deg` | 左右 `zu` 高度差除以各軸輪距，再由 rad 轉 deg 的輪胎變形 roll。 |
| `total_roll_front_deg` / `total_roll_rear_deg` | 原始 CSV 懸吊 roll + 輪胎變形 roll；原始 `roll_front_deg`、`roll_rear_deg` 欄位保留。 |
| `front_solver_ok` / `rear_solver_ok` | 該筆、該軸是否完整計算成功。 |
| `front_error` / `rear_error` | 缺值、非數值或幾何不可達時的具體原因。 |

資料列不會因計算失敗被刪除：受影響結果留為 `NaN` 並標明原因，另一軸可繼續計算。缺少 roll 時仍保留可計算的車身相對 camber gain，最終對地 camber 留為 `NaN`。程式會回報前後軸失敗筆數；有失敗列時結束代碼為 1。缺少必要欄位或時間非遞增則停止並顯示錯誤。

前輪內外側比較使用 `ay_m_s2`（m/s²），沿用車身座標 **X 向前、Y 向左、Z 向上**；加速度指向彎內，因此依下表判斷，不使用方向盤轉角或 roll 決定內外側：

| 橫向加速度 | 轉彎方向 | 外側前輪 | 內側前輪 |
| --- | --- | --- | --- |
| `ay > 0` | 左轉 | FR | FL |
| `ay < 0` | 右轉 | FL | FR |
| `ay = 0`、缺值或非有限值 | 無法分類 | 空白 | 空白 |

比較圖包含內外側對地 camber 時序、`外側 − 內側` 角度差與橫向加速度。保留原本的 camber 正負號；角度差小於零表示外側 camber 比內側更負。直行或無法分類時 camber 留為 `NaN`；左右轉切換處斷線，避免將不同實體輪胎連成同一條連續曲線。平均值與範圍只使用內外兩輪皆有效的轉彎樣本，為逐筆樣本統計，未依時間或輪胎負載加權。

CSV 另加入 `front_outside_wheel`、`front_inside_wheel`、`camber_front_outside_deg`、`camber_front_inside_deg` 與 `camber_front_outside_minus_inside_deg`。產生比較圖時必須有 `ay_m_s2` 欄位；`--no-plot` 關閉全部三張圖。可用 `--front-comparison-figure 路徑.png` 自訂內外側比較圖位置。來源若採 Y 向右為正，請先反轉 `ay_m_s2` 再使用此判斷。

`data_signals_camber_vehicle_comparison.png` 以同一時間軸上下對齊四個區塊：車速 `speed_km_h`（km/h）、三軸加速度 `ax_m_s2`／`ay_m_s2`／`az_m_s2`（m/s²）、左右前輪輪端轉角 `steer_FL_deg`／`steer_FR_deg`（deg）、前輪內外側對地 camber（deg）。左右轉向同樣以左轉為正，camber 沿用依 `ay` 分類與換輪斷線的規則。可用 `--vehicle-comparison-figure 路徑.png` 自訂輸出；產生此圖時另需車速與三軸加速度欄位。

本分析沿用既有的角度疊加近似：主銷 KPI/caster 不隨懸吊行程更新；pitch、bump steer、轉向頂升平衡與完整 3D 旋轉耦合未納入。左右 `zu` 差依本次資料定義解讀為輪胎變形造成的 roll，並以小角度比值加入懸吊 roll；最終角度仍以水平地面為基準，沒有另外扣除道路橫坡。

驗證：`python -m unittest suspension_geo.tests.test_data_signals_camber -v`。

### 懸吊抬升與轉向 Camber 疊加

`combined_camber.py` 使用 `fv/fv_camber_rc.py` 相同的 `SuspensionDirectSolver` 與 `we3D/steering_camber.py` 的計算函式。輸入左右輪端轉角，以及左右輪側相對車身的幾何抬升量，輸出各項 camber 角度貢獻、疊加後的左右輪 camber、各自的局部 gain 與前視 RC。

```powershell
# 左輪左轉 25°、上抬 10 mm；右輪左轉 18°、下伸 5 mm
python -m suspension_geo.combined_camber --steer-left 25 --steer-right 18 --travel-left-mm 10 --travel-right-mm -5

# 使用檔案上方 STEER_LEFT_DEG、STEER_RIGHT_DEG、TRAVEL_LEFT_MM、TRAVEL_RIGHT_MM 的設定
python -m suspension_geo.combined_camber

# 加入各輪初始 camber（預設均為 0°）
python -m suspension_geo.combined_camber --steer-left 25 --steer-right 18 --travel-left-mm 10 --travel-right-mm -5 --static-camber-left -2 --static-camber-right -2
```

也可直接執行 `python suspension_geo/combined_camber.py`。若終端機沒有可用的 `python`，本專案已有的環境可用 `& .\Suspension_sys\.venv\Scripts\python.exe -m suspension_geo.combined_camber` 執行；需要 `numpy`、`scipy`。

轉角單位為 **deg，兩輪均以左轉為正、右轉為負**；抬升量單位為 **mm，輪側相對車身向上／懸吊壓縮為正，下伸為負**。若輸入的是車身相對輪側抬升量，需先反轉正負號。Camber 沿用上端向車外為正、向車內為負。

每一輪的角度疊加定義如下，`zL`、`zR` 為左右行程，`δ` 為該輪轉角，`γ0` 為該輪初始 camber：

```text
Δγ_FV       = γ_FV(zL, zR) - γ_FV(0, 0)
Δγ_steering = γ_steering(δ; γ0) - γ0
Δγ_total    = Δγ_FV + Δγ_steering
γ_final     = γ0 + Δγ_total
```

使用原求解器直接計算有限行程與轉角的角度變化，不以局部 gain 乘上輸入量近似。**疊加的是 deg 角度變化量**；懸吊局部 gain 為 **deg/mm**，轉向局部 gain 為 **deg/deg**，兩者單位不同，分別列出。初始 camber 只計入一次。

```python
from suspension_geo.combined_camber import calculate_combined_camber

result = calculate_combined_camber(
    steer_left_deg=25.0, steer_right_deg=18.0,
    travel_left_mm=10.0, travel_right_mm=-5.0,
)
print(result["total_camber_change_left_deg"], result["total_camber_change_right_deg"])
print(result["camber_left_deg"], result["camber_right_deg"])
```

| 回傳欄位（`side` 替換為 `left` 或 `right`） | 意義 |
| --- | --- |
| `fv_camber_change_{side}_deg` | 前視懸吊相對零行程的角度變化，deg。 |
| `steering_camber_change_{side}_deg` | 轉向相對直行的角度變化，deg。 |
| `total_camber_change_{side}_deg` | 兩項角度變化的總和，deg。 |
| `camber_{side}_deg` | 初始 camber 加上總變化，deg。 |
| `fv_camber_gain_{side}_deg_per_mm` | 固定另一側行程時，對自身壓縮行程的局部斜率，deg/mm。 |
| `steering_camber_gain_{side}_deg_per_deg` | 固定行程時，對該輪輪端轉角的局部斜率，deg/deg。 |
| `rc_pos` | 前視 RC 座標 `[X, Y]`，m；無有限交點時為 `None`。 |

兩個模型共用目前 `car.py` 的前懸吊參數，也可傳入 `car=` 與 `suspension=` 自訂物件。每次呼叫均建立獨立的 FV 求解器，不改動既有 mapping 或原腳本的全域求解器。

這是依需求將兩種角度變化疊加的近似模型。FV 保留相對左右接地點連線的地面基準；轉向項保留設計狀態的 KPI/caster，**不隨行程重新計算主銷方向**，也不把 FV camber 再傳入轉向項。沒有計算完整 3D 懸吊與轉向耦合、bump steer 或轉向頂升平衡。前視 RC、heave、roll 也僅來自輸入的懸吊行程。不可達行程或無效輸入會顯示錯誤。

驗證：`python -m unittest suspension_geo.tests.test_combined_camber -v`。

### 前視互動圖接點座標 CSV

執行 `fv/sus_fv_v5.py` 時，會自動在腳本同一資料夾產生 `fv/sus_fv_v5_points.csv`。開啟時先輸出初始姿態；每次移動左右滑桿後，覆寫為目前畫面姿態的座標，關閉視窗後保留最後一次成功輸出。不受執行時所在目錄影響。

CSV 欄位為 `side, point, description, x_mm, y_mm`，共 26 個點：左右各自的上下控制臂車身接點（`cpb_rl`、`cpb_ru`）、羊角接點（`kua`、`lak`）、主銷地面參考點（`kg`）、輪胎接地點（`gnd`）、瞬心（`IC`）、初始瞬心參考點（`ICo`）、四個輪胎外框點（`wp0`～`wp3`），以及中央重心（`cg`）與滾動中心（`rc`）。`ICo` 是固定的初始設計參考，`IC` 隨當前幾何重新計算。

座標沿用前視圖：原點在初始地面與車身中心線交點，X 正向為圖中的右側，Y 正向向上，單位均為 **mm**。這是二維前視座標；無有限交點的座標欄位留白。CSV 使用 UTF-8 BOM，方便 Excel 讀取中文說明；若檔案被其他程式鎖定，終端機會顯示寫入失敗原因，懸吊圖仍可操作。

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
  定義車輛、輪胎、前後懸吊設計參數
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
  互動式前視圖，滑桿調整左右 lower arm angle（介面設定來自 analysis_settings.py）
      |
fv/sus_fv_maping.py
  依 analysis_settings.py 的範圍與步數掃描，輸出 fv/suspension_kinematics_data.pkl
      |
fv/rc_camber_map.py
  讀取 pkl，畫出 camber surface 與 roll center map
```

## 檔案總覽

| 檔案 | 內容與目的 |
| --- | --- |
| `car.py` | 定義 `Car` 車輛/輪胎參數，以及 `SuspensionF`、`SuspensionR` 前後懸吊實體設計參數。 |
| `analysis_settings.py` | 定義獨立的 `AnalysisSettings`，管理分析範圍、掃描步數、圖窗、滑桿、背景外框與側視座標參考。 |
| `geometry.py` | 基礎 2D 幾何物件。包含 `Point`、`Link`、`RigidBody2D`、`Polygon`、`Wheel`、`Body`。 |
| `Geometric_Analysis.py` | 前視懸吊幾何與運動學核心。負責算靜態點位、上下控制臂、羊角、instant center、roll center、camber、行程反算等。 |
| `combined_camber.py` | 指定左右輪端轉角與輪側抬升量，疊加 FV 與轉向 camber 角度變化，另列各自局部 gain 與前視 RC。 |
| `visualization.py` | Matplotlib 視覺化封裝。把 `Point`、`Link`、`Polygon` 畫成可更新的 plot object。 |
| `fv/sus_fv_v5.py` | 互動式前視懸吊圖。用兩個 slider 控制左右 lower arm angle，即時更新幾何、roll center 與同資料夾的 `sus_fv_v5_points.csv` 接點座標。 |
| `sv/sus_sv_v4.py` | 互動式前後軸側視懸吊圖。前後輪中心相隔 `Car.l`，使用 `SuspensionF` 與 `SuspensionR` 幾何；兩個滑桿分別控制前、後輪的 Al 高度並更新當下 IC。後軸幾何沿車身縱向反向，後輪 IC 位於後輪後方，前輪 IC 位於前輪前方。 |
| `tv/sus_tv_v2.py` | 上視轉向幾何，以轉向機位移控制左右輪轉向並顯示掃描曲線。 |
| `we3D/sus_3Dwe_v3.py` | 3D 輪端轉向圖，顯示接地點軌跡、轉向力臂與輪胎傾角。 |
| `we3D/steering_camber.py` | 輸入左右前輪輪端轉角，求有號 camber、相對直行變化量（deg）與局部轉向 camber gain（deg/deg）。[使用方式與角度定義](we3D/README.md)。 |
| `fv/sus_fv_maping.py` | 批次掃描左右 lower arm angle，計算 heave、roll、左右 camber、RC 位置與各點座標，輸出 `.pkl`。 |
| `fv/rc_camber_map.py` | 讀取同資料夾的 `suspension_kinematics_data.pkl`，畫出 camber 與 roll center map。 |
| `fv/fv_camber_rc.py` | 直接求解範例。展示如何用 theta、wheel travel、heave/roll 姿態反推 camber 與 roll center。 |
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

地面基準會隨左右輪行程改變。計算使用 `atan2` 保留方向，角度單位為 deg。更新前產生的結果不含正負方向；重新執行 `fv/sus_fv_maping.py` 可更新 mapping 的 PKL。`fv/run15_pose_analysis.py` 與 `fv/fv_camber_rc.py` 也使用相同正負定義，既有 Run 15 CSV/PNG 需重新執行分析才會更新。

## `car.py`

參數分成兩類獨立物件：`car.py` 的 `Car`、`SuspensionF`、`SuspensionR` 保留實體車輛與懸吊設計資料；`analysis_settings.py` 的 `AnalysisSettings` 管理分析、顯示與介面設定。

| 物件 | 主要內容 |
| --- | --- |
| `Car` | 軸距、重心位置、前後輪距、質量、重力、車底高度、輪胎參數與負載輪胎半徑。 |
| `SuspensionF` / `SuspensionR` | 各軸的前視 `scrub_radius`、`kpi`、`h_rc`、`fvsa`、`Au`/`Al`、車身側硬點，以及側視 caster、mechanical trail、煞車分配與 anti-dive/anti-lift。側視圖中的前後輪中心距使用 `Car.l`。 |
| `AnalysisSettings` | 掃描行程與步數、圖窗尺寸、座標範圍、滑桿位置與範圍、車體背景外框、側視座標參考與輔助線長度。 |

前視分析會同時建立兩個物件，再交給幾何 helper：

```python
from suspension_geo.car import Car, SuspensionF
from suspension_geo import Geometric_Analysis as ga

car = Car()
suspension_f = SuspensionF()
sus_helper = ga.SusGeometryHelper(car, suspension_f)
```

`SusGeometryHelper` 從 `Car` 讀取車輛/輪胎資料，從 `SuspensionF` 或 `SuspensionR` 讀取各軸懸吊硬點與幾何目標。傳入前軸時使用 `Car.tf` 與前輪負載半徑，傳入後軸時使用 `Car.tr` 與後輪負載半徑。

前視圖的前、後軸車身側接點均由各自懸吊物件的兩個 X 設定決定，取代原本的 `sus_contact` 斜線端點。`SuspensionF` 設定如下：

```python
self.lower_mount_x = 210 / 1000  # 下控制臂車身側 X [m]
self.upper_mount_x = 255 / 1000  # 上控制臂車身側 X [m]
```

`SuspensionR` 沿用原設定的 X 尺寸，獨立設定為：

```python
self.lower_mount_x = 205 / 1000  # 後軸下控制臂車身側 X [m]
self.upper_mount_x = 250 / 1000  # 後軸上控制臂車身側 X [m]
```

兩個值均為右側的非負 X 座標，左側自動鏡像為負。前軸四個接點的 X 為下側 ±210 mm、上側 ±255 mm；後軸則為下側 ±205 mm、上側 ±250 mm，各軸可分別修改。Y 不需輸入：以各自羊角接點 `(x_u, y_u)` 與設計瞬心 `(x_ic, y_ic)`，直接代入 `y = y_u + (x_mount - x_u) * (y_ic - y_u) / (x_ic - x_u)`。這樣會保留各軸靜態 FVSA 與 Roll Center 設計條件；改變 KPI、scrub radius、FVSA 或 RC 高度時，Y 會重新計算，而設定的 X 保持固定。滑桿動作期間，車身側接點的 X、Y 均固定。

`Car` 的主要參數包含：

| 參數 | 說明 |
| --- | --- |
| `l`, `lf`, `lr` | 軸距與重心到前/後軸距離。 |
| `tf`, `tr` | 前/後輪距。front view 目前主要使用 `tf`。 |
| `h_cog` | 重心高度。 |
| `m`, `w` | 車重與重量。 |
| `bottom` | 車底距地高度。 |
| `free_radius`, `load_Radius`, `tire_width` | 輪胎自由半徑、負載半徑與胎寬。 |

`SuspensionF` 與 `SuspensionR` 的參數名稱都不加 `_f` 或 `_r` 後綴，由物件本身表示軸別。側視圖使用前軸為正 x 方向，後軸沿 x 方向反轉；輪心 x 間距固定為 `Car.l`。兩個側視滑桿分別改變前後軸 Al 高度。前、後軸參數可在各自的 `SuspensionF`、`SuspensionR` 中獨立調整。

## `analysis_settings.py`

`AnalysisSettings` 與車輛參數分開建立，供 FV 互動圖、mapping、最佳化與 SV 工具使用。修改預設分析或介面設定時，編輯這個檔案；幾何求解器 `SusGeometryHelper` 仍只依賴 `Car` 與各軸懸吊參數。

| 設定 | 用途與預設值 |
| --- | --- |
| `front_view_target_travel` | FV 掃描半行程 0.025 m，並非實車機械行程限制。 |
| `front_view_mapping_steps` | 每輪 30 點，左右組合共 900 點。 |
| `front_view_figure_size` / `side_view_figure_size` | FV `(8, 6)`、SV `(10, 8)` inches。 |
| `front_view_slider_axes` / `side_view_slider_axes` | 滑桿在圖窗中的位置。 |
| `front_view_plot_bottom` / `side_view_plot_bottom` | 圖窗底部為滑桿保留的空間。 |
| `front_view_x_limits` / `front_view_y_limits` | FV 圖的 X/Y 顯示範圍。 |
| `front_view_body_half_width` / `front_view_body_top` | FV 車體背景外框半寬 0.21 m、頂部高度 0.5 m。 |
| `contact_point_sv` / `wheel_center_x_sv` | SV 局部接地參考 `(0, 0)` 與後輪心圖面原點 `0` m。 |
| `kingpin_length_sv` / `side_view_path_half_length` | 主銷輔助線長 0.6 m、運動路徑繪製半長 0.5 m。 |
| `side_view_wheel_outline_points` | 輪胎外框 48 個頂點。 |
| `side_view_front_travel_limit` / `side_view_rear_travel_limit` | SV 前、後滑桿半行程分別為 0.03 / 0.025 m。 |

例如，在自訂分析中分開使用實體參數與分析設定：

```python
from suspension_geo.car import Car, SuspensionF
from suspension_geo.analysis_settings import AnalysisSettings
from suspension_geo import Geometric_Analysis as ga

car = Car()
suspension = SuspensionF()
settings = AnalysisSettings()
settings.front_view_mapping_steps = 40

helper = ga.SusGeometryHelper(car, suspension)
right = ga.SuspensionSide("Right")
right.setup_right_side(helper.calc_static_points())
limits = ga.calculate_theta_limits(right, settings.front_view_target_travel)
```

原先 `SuspensionF`／`SuspensionR` 的掃描與介面欄位已移到 `AnalysisSettings`，不再由懸吊物件提供。SV 滑桿行程改為上表的前、後軸欄位；`Car.body_face_f` 改為 `settings.front_view_body_outline(car.bottom)`，讓背景外框依目前的實體車底高度建立。`SideSuspension(..., settings=settings)` 可傳入獨立設定，未提供時使用該側視模組共用的 `analysis_settings`。

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
| `SusGeometryHelper` | 從 `Car` 與 `SuspensionF`／`SuspensionR` 參數建立各軸靜態點位，並提供線交點、IC 更新、RC 計算。 |
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

用 SciPy `differential_evolution` 的 `integrality` 限制，在指定尺寸格點上最佳化 RC 高度與 FVSA。起點為 RC 高度 **0 mm**、FVSA **1000 mm**；RC 高度範圍 **−5～50 mm**、步距 **1 mm**，FVSA 範圍 **750～2500 mm**、步距 **50 mm**。整個搜尋過程都使用這些離散尺寸，關閉連續尺寸的 polishing。

每個候選設計會重新建立懸吊硬點與行程角度範圍，依 `front_view_target_travel`（預設每輪約 ±25 mm）取樣完整 mapping，預設左右各 30 點，共 900 個姿態。左右輪可獨立求解，因此程式只需解出單輪各角度，再鏡像並組合出完整網格；測試會核對它與既有 FV mapping 的結果一致。幾何無解或 IC/RC 無有限交點時，整組設計列為無效，避免以不完整的 mapping 降低 cost。

RC cost 的固定目標是 **`(X, Y) = (0, y_best) = (0, 30) mm`**，由 `Y_BEST_MM = 30.0` 定義。`y_best` 不參與最佳化，也不隨起點或候選設計的靜態 RC 高度改變。令每個 mapping 點到固定目標的距離為 `r`：

```text
y_best = 30 mm  # 固定常數
r² = (x_mm - 0.0)² + (y_mm - y_best)²
RC_distance_raw = mean(r²)  # 徑向距離目標，mm²，尚未套用 RC 懲罰
k_rc = 3 if 任一 mapping 點 r > RC_LIMIT_MM else 1
RC_height_raw = mean((y_mm - y_best)²)  # 獨立的純高度目標，mm²

Camber_raw = k_camber × mean(camber_deg²)  # deg²，左右輪全部取樣一起平均
k_camber = 3 if 任一輪、任一行程點 |camber| > 2 deg else 1

RC_distance_scale = 起始設計的 RC_distance_raw
RC_height_scale = 起始設計的 RC_height_raw
Camber_scale = 起始設計的 Camber_raw
RC_distance cost = RC_distance_raw / RC_distance_scale
RC_height cost = RC_height_raw / RC_height_scale
RC_base = (rc_distance_weight × RC_distance cost + rc_height_weight × RC_height cost)
          / (rc_distance_weight + rc_height_weight)
RC cost = k_rc × RC_base  # 整個 RC cost 乘一次，包含徑向與高度兩項
Camber cost = Camber_raw / Camber_scale

w_rc = rc_weight / (rc_weight + camber_weight)
w_camber = camber_weight / (rc_weight + camber_weight)
Total cost = w_rc × RC cost + w_camber × Camber cost
```

徑向距離、純高度、camber 三個正規化分母在每次執行開始時，以 RC 高度 **0 mm**、FVSA **1000 mm** 的完整 mapping 計算一次，之後保持固定。RC 的兩個分母都使用未懲罰誤差，3 倍懲罰在正規化與 RC 內部加權合併後才套用，避免起始設計也超標時將懲罰抵消。起點的兩項正規化 RC 誤差及 camber cost 一般都是 **1**；若起點任一 RC 點超標，其最終 RC cost 為 **3**。若某項起始誤差已經為零，改用該項容許尺度的平方作分母，避免除以零。

RC 內部目前 `rc_distance_weight = 1`、`rc_height_weight = 1`，即徑向距離與純高度目標各占 **1/2**；命令列高度權重預設沿用 `Settings.rc_height_weight`。高度誤差具有獨立尺度，避免較大的 X 遷移把 Y 目標淹沒。純高度的原始誤差直接量測整個 mapping 的 `(Y−30)²`；最終徑向與高度加權貢獻都包含同一個全域 RC 懲罰倍率。

外部預設 `rc_weight = camber_weight = 1`，RC 與 camber 係數各為 **50%**。目前權重下 total cost 等價於 `k_rc × (0.25 × RC_distance cost + 0.25 × RC_height cost) + 0.5 × Camber cost`；一般非零誤差起點若 RC 未超標，total cost 為 **1**，超標則為 **2**。原始平方誤差、三項正規化 cost、加權貢獻與固定分母都會印出並儲存。圖上虛線另外標示含懲罰的 RC 高度貢獻。

目前 `RC_LIMIT_MM = 50.0`，RC 懲罰圓心固定為 `(0, 30) mm`；半徑由這個常數設定，獨立於正規化分母。只要任一點嚴格超出半徑，整個正規化 RC cost 就乘以 `PENALTY_FACTOR = 3.0`，包含徑向與高度項；恰好在邊界不觸發，多點超標也只乘一次。這個倍率會直接進入最佳化器使用的 total cost，但仍是軟懲罰，並不保證所有行程點都在範圍內。Camber 目標使用角度（deg）接近零的平方誤差；camber gain（deg/mm）需另行定義目標。

30 mm 是全 mapping 的 RC 高度目標，設計的靜態 RC 高度仍是最佳化變數。終端與 JSON 另列出 `rc_y_mean_mm`、`rc_y_error_rms_mm`、`rc_y_min_mm`、`rc_y_max_mm`，分別表示 mapping 的平均 RC 高度、相對 30 mm 的高度 RMS 誤差與高度範圍。`rc_cost_unpenalized` 是懲罰前的 RC cost，`rc_penalty_factor` 為 1 或 3，`rc_cost` 為兩者相乘；`rc_penalty_points` 保留實際超標點數。`rc_distance_cost` 與 `rc_height_cost` 不含懲罰，各自的加權 contribution 則含懲罰。

差分演化完成後，預設再用 SciPy `brute` 檢查全部 **56 × 36 = 2,016** 組尺寸，確認此 mapping 與離散尺寸範圍中的最佳解。重複尺寸使用快取。程式會分別列印 DE 的停止狀態與完整格點驗證狀態。

```powershell
python -m suspension_geo.fv.optimize_fv_geometry
# 批次執行，存圖而不開啟視窗
python -m suspension_geo.fv.optimize_fv_geometry --no-show
# 進一步加強 RC 高度接近 30 mm：RC 內部距離:純高度 = 1:10
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

可用 `--rc-weight`、`--camber-weight` 調整權重，`--output-dir` 改輸出位置，`--steps` 改 mapping 解析度。RC 目標固定為 `(0, 30) mm`，已移除隨候選設計改變目標的 `--reference` 選項。`--camber-penalty point` 可改成只加重 camber 超標行程點，RC 懲罰仍為全域。`--no-grid-check` 可略過完整格點驗證，此時不保證離散全域最佳。程式以新物件計算，不會將最佳尺寸自動寫回 `car.py` 或覆蓋既有 mapping PKL。

演算法參考：[SciPy differential_evolution](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html)。

### `fv_camber_rc.py`

`fv_camber_rc.py` 是非圖形互動的求解範例，展示 `SuspensionDirectSolver` 的三種用法：

| 方法 | 用途 |
| --- | --- |
| `solve_by_thetas(theta_l, theta_r)` | 已知左右 lower arm angle，直接求 heave、roll、camber、RC。 |
| `solve_by_pose(target_heave_m, target_roll_deg)` | 已知車身 heave/roll，反算左右 lower arm angle，再求 camber、RC。 |
| `solve_by_travels(l_travel_m, r_travel_m)` | 已知左右 wheel travel，反算幾何狀態。 |

三種求解方式均額外回傳 `camber_gain_left_deg_per_mm` 與
`camber_gain_right_deg_per_mm`，`fv_camber_rc.py` 的三個範例也會印出這兩項。
定義為目前姿態下，各輪有號 camber 對自身輪側壓縮行程的局部變化率
`∂γ_left/∂z_left`、`∂γ_right/∂z_right`，單位 **deg/mm**，計算各側時固定另一側行程。
行程正值代表壓縮，負 gain 代表壓縮時 camber 往負方向變化。
Camber 沿用相對當前左右接地點連線的定義，因此 gain 包含該連線隨單輪行程
改變的效果；它不是車身座標下的 camber gain，也不是每度 roll 的 camber 變化率。
以該側下臂角 `theta ± 1e-5 rad` 取樣，將 camber 差除以實際輪側垂直位移差
（換算為 mm）。取樣後恢復指定姿態，不改變原有 RC、camber 與行程結果；
若鄰近幾何不可達或位移差接近零，該側 gain 回傳 `NaN`。

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
4. 執行 `fv/fv_camber_rc.py`，測試指定 theta、travel、heave/roll 的求解結果。
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
