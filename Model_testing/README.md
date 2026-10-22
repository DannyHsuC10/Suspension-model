# dynamics_analyze 資料夾說明

`dynamics_analyze` 是懸吊動態分析的獨立工作區，主要用簡化模型快速比較不同懸吊架構、路面輸入與車身姿態反應。它把分析分成三條主線：

1. `quarter`：單輪/四分之一車垂向模型。
2. `LR`：左右輪半車模型，重點是 heave、roll、左右載重轉移與 roll center 影響。
3. `FR`：前後半車模型，重點是 heave、pitch、加速/煞車造成的縱向載重轉移。

整理日期：2026-08-29

## 整體目的

這個資料夾的用途是建立一個「可替換模型與輸入」的懸吊模擬平台。核心流程固定如下：

```text
建立 suspension 參數
      |
建立 state 初始狀態
      |
每個時間步 dt
      |
road function 更新路面/加速度輸入
      |
model function 計算力、力矩、加速度
      |
state.Integration() 做時間積分
      |
SimulationLogger 記錄結果
      |
輸出 .pkl 或開啟視覺化
```

目前主要時間積分方式是 semi-implicit Euler，時間步長預設多為 `dt = 0.001 s`。

## 根層檔案

| 檔案 | 內容與目的 |
| --- | --- |
| `data_creator.py` | 批次產生模擬資料的主要腳本。用 `ROADS_*`、`MODELS_*`、`DATA_*` 字典組合不同路面、模型與參數，並將結果存成 `.pkl`。 |
| `lab_quartersus.py` | quarter-car 模型互動測試腳本，跑完後用 `Dynamic_Visualizer_quarter.Visualizer` 顯示結果。 |
| `lab_LRsus.py` | 左右半車/roll 模型測試腳本，跑完後用 `RollCarVisualizer` 顯示車身左右動態。 |
| `lab_FRsus.py` | 前後半車/pitch 模型測試腳本，跑完後用 `PitchCarVisualizer` 顯示車身前後動態。 |
| `lab_rollcenter.py` | roll center 與 transient load transfer rate 分析腳本，會分離 geometry 與 elastic 的載重轉移速率。 |
| `lab2.py` | 額外實驗腳本，屬於分析沙盒。 |
| `load_and_show.py` | 載入既有 `.pkl` 模擬結果並顯示視覺化，可用來快速檢查已存資料。 |
| `.vscode/settings.json` | VS Code 本地設定。 |

## 子資料夾

| 資料夾 | 內容與目的 |
| --- | --- |
| `model/` | 懸吊動態模型函式。模型函式接收 `(sus, state)`，更新 state 內的力、力矩與加速度。 |
| `road/` | 路面、加速度、過彎等輸入函式。road function 接收 `state`，更新 `zr`、`vzr`、`ax`、`ay`、`Fx_front`、`Fx_rear` 等輸入。 |
| `Parameter_set/` | 懸吊參數與狀態類別。包含 quarter、LR、FR 三種分析尺度。 |
| `data_visualization/` | 模擬結果 logger 與 Matplotlib 動態視覺化工具。 |
| `data/` | 已產生的 `.pkl` 模擬結果，分成 `quarter/`、`LR/`、`FR/`。 |

## `Parameter_set/` 詳解

### `Parameter_set/Suspension.py`

這個檔案定義懸吊參數類別。

| 類別 | 用途 | 主要參數 |
| --- | --- | --- |
| `SuspensionQuarter` | 四分之一車垂向模型參數。 | `ms`、`mu`、`ks`、`cs`、`kt`、`ct`、`kw`、`cw`、`ride_rate`。 |
| `SuspensionLR` | 左右半車/roll 模型參數。 | `track`、`Ix`、`ms`、`mu`、`h_rc`、`h_cg`、`k_heave`、`k_roll`、`c_heave`、`c_roll`、`kt`、`ct`、`k_frame`。 |
| `SuspensionFR` | 前後半車/pitch 模型參數。 | `Iy`、`ms`、`mu`、`lf`、`lr`、`h_cg`、`anti_dive_front`、`anti_squat_rear`、`k_heave_f`、`k_heave_r`、`c_heave_f`、`c_heave_r`、`kt`、`ct`。 |

### `Parameter_set/suspension_state.py`

這個檔案定義模擬狀態與積分方法。

| 類別 | 用途 | 主要狀態 |
| --- | --- | --- |
| `quarter_suspension_state` | 單角/單輪狀態。 | `t`、`dt`、`zs`、`zu`、`vzs`、`vzu`、`azs`、`azu`、`Fs`、`Ft`、`zr`、`vzr`。 |
| `LR_suspension_state` | 左右半車狀態，由兩個 quarter state 組成。 | `left`、`right`、`ay`、`F_heave`、`F_roll`、`Fz`、`Mx`、`az_body`、`alpha_roll`、`heave`、`roll`、`d_frame`。 |
| `FR_suspension_state` | 前後半車狀態，由 front/rear quarter state 組成。 | `front`、`rear`、`ax`、`Fx_front`、`Fx_rear`、`Fz`、`My`、`az_body`、`alpha_pitch`、`heave`、`pitch`。 |

## `model/` 詳解

### `model/quarter_model.py`

這裡放 quarter-car 相關垂向模型。

| 函式 | 模型意義 |
| --- | --- |
| `single_spring_model(sus, state)` | 最簡化單彈簧模型，用 `ride_rate` 把懸吊與輪胎合併。 |
| `one_eighth_suspension_model(sus, state)` | 使用 `ride_rate + 懸吊阻尼` 的簡化模型。 |
| `Double_spring_model(sus, state)` | 純雙彈簧模型，分開簧上、簧下與輪胎彈簧，但未加入阻尼。 |
| `Quarter_suspension_model(sus, state)` | 雙彈簧阻尼 quarter-car 模型，分開懸吊彈簧/阻尼與輪胎彈簧/阻尼。 |

### `model/LR_model.py`

這裡放左右向半車模型，主要用來分析左右輪輸入、roll、heave、roll center 與載重轉移。

| 函式 | 模型意義 |
| --- | --- |
| `two_mass_suspension_2DOF_model(sus, state)` | 兩質點、2DOF 的 modal 模型，只看 heave/roll，較簡化。 |
| `two_mass_suspension_model(sus, state)` | 4DOF 左右半車模型，包含左右簧上與簧下。 |
| `Half_suspension_2DOF_model(sus, state)` | 2DOF 半車模型，加入虛擬剛體耦合，不含完整簧下輪胎動態。 |
| `Half_suspension_model(sus, state)` | 4DOF 半車模型，包含輪胎力、roll center 幾何力與車身 roll dynamics。 |
| `Half_rollcenter_all_model(sus, state)` | roll center transient 測試模型，加入車架/幾何力傳遞延遲系統，用於 `lab_rollcenter.py`。 |

### `model/FR_model.py`

這裡放前後向半車模型，主要用來分析 pitch、加速、煞車與 front/rear 路面輸入。

| 函式 | 模型意義 |
| --- | --- |
| `Half_suspension_2DOF_model(sus, state)` | 不考慮簧下與 anti 的簡化 pitch 模型。 |
| `Half_suspension_model(sus, state)` | 4DOF 前後半車模型，包含簧下動態、輪胎力、anti-dive/anti-squat 幾何效果。 |

## `road/` 詳解

### `road/quarter_road.py`

| 函式/類別 | 輸入型態 |
| --- | --- |
| `road_step` | 單輪階躍路面。 |
| `road_impulse` | 單輪矩形衝擊路面。 |
| `road_bump` | 半正弦 bump。 |
| `road_sine` | 連續正弦路面。 |
| `RandomRoad` | 隨機高度路面，依 `dt_update` 更新高度。 |

### `road/LR_road.py`

| 函式 | 輸入型態 |
| --- | --- |
| `road_step_left` | 左輪單邊階躍。 |
| `road_step_both` | 左右同步階躍。 |
| `road_impulse_left` | 左輪單邊衝擊。 |
| `road_impulse_both` | 左右同步衝擊。 |
| `road_sin_left` | 左輪單邊正弦。 |
| `road_sin_both` | 左右同步正弦。 |
| `road_sin_alternate` | 左右反相正弦，用來激發 roll。 |
| `Cornering_left` | 直接給定側向加速度 `ay`，用來分析過彎載重轉移。 |
| `Cornering_ramp_to_max` | 側向加速度 ramp 輸入，適合 transient load transfer rate。 |

### `road/FR_road.py`

| 函式 | 輸入型態 |
| --- | --- |
| `road_impulse_front_rear` | 前後輪同步衝擊。 |
| `road_impulse_rear` | 只有後輪衝擊。 |
| `road_impulse_drive` | 前輪先壓過障礙，後輪依軸距/速度延遲通過。 |
| `road_sine_front_rear` | 前後反相正弦路面。 |
| `accelerate` | 給定正向 `ax` 與前後輪縱向力，模擬加速。 |
| `breaking` | 給定負向 `ax` 與前後輪縱向力，模擬煞車。 |

## `data_visualization/` 詳解

| 檔案 | 內容與目的 |
| --- | --- |
| `Simulation_Logger.py` | 遞迴記錄 state 物件中的數值欄位，輸出/讀取 pickle package。輸出格式包含 `data` 與 `metadata`。 |
| `Dynamic_Visualizer_quarter.py` | quarter-car 垂向模型動畫/圖表視覺化。 |
| `Dynamic_Visualizer_roll.py` | 左右半車 roll 視覺化。 |
| `Dynamic_Visualizer_pitch.py` | 前後半車 pitch 視覺化。 |

`SimulationLogger.add_state(state)` 會把巢狀物件展開。例如 LR state 中的 `left.zs` 會被記成 `left_zs`，`right.zu` 會被記成 `right_zu`。

## `data/` 已產生資料

`data/` 內是由模擬腳本產生的 pickle 結果檔，每個檔名大致遵守：

```text
{model_name}_{road_name}.pkl
```

### `data/quarter/`

包含 4 種 quarter 模型與 5 種路面輸入的排列組合：

| 模型名稱 | 對應函式 |
| --- | --- |
| `quarter` | `Quarter_suspension_model` |
| `double` | `Double_spring_model` |
| `single` | `single_spring_model` |
| `one_eighth` | `one_eighth_suspension_model` |

| 路面名稱 | 對應輸入 |
| --- | --- |
| `step` | 階躍 |
| `bump` | 半正弦 bump |
| `impulse` | 衝擊 |
| `sine` | 正弦 |
| `random` | 隨機路面 |

### `data/LR/`

包含左右半車模型與左右路面/過彎輸入，例如：

| 模型名稱 | 對應函式 |
| --- | --- |
| `2mass_2dof` | `two_mass_suspension_2DOF_model` |
| `2mass` | `two_mass_suspension_model` |
| `half_2dof` | `Half_suspension_2DOF_model` |
| `half` | `Half_suspension_model` |

| 輸入名稱 | 意義 |
| --- | --- |
| `step_L` | 左輪階躍 |
| `step` | 左右同步階躍 |
| `impulse_L` | 左輪衝擊 |
| `impulse` | 左右同步衝擊 |
| `sin_L` | 左輪正弦 |
| `sin` | 左右同步正弦 |
| `sin_A` | 左右反相正弦 |
| `Corner` | 過彎側向加速度 |

### `data/FR/`

包含前後半車模型與 pitch/加速/煞車輸入：

| 模型名稱 | 對應函式 |
| --- | --- |
| `2dof` | `Half_suspension_2DOF_model` |
| `half` | `Half_suspension_model` |

| 輸入名稱 | 意義 |
| --- | --- |
| `impulse` | 前後同步衝擊 |
| `impulse_r` | 後輪衝擊 |
| `impulse_D` | 前輪先、後輪延遲通過障礙 |
| `sin` | 前後反相正弦 |
| `ass` | 加速輸入，檔名應是 accelerate 的縮寫或筆誤 |
| `brk` | 煞車輸入 |

## `data_creator.py` 使用方式

`data_creator.py` 透過三組字典決定要跑哪一類分析：

| 分析類型 | road 字典 | model 字典 | data 設定 |
| --- | --- | --- | --- |
| quarter | `ROADS_Q` | `MODELS_Q` | `DATA_Q` |
| 左右/roll | `ROADS_LR` | `MODELS_LR` | `DATA_LR` |
| 前後/pitch | `ROADS_FR` | `MODELS_FR` | `DATA_FR` |

檔案底部目前設定為：

```python
ROADS = ROADS_FR
MODELS = MODELS_FR
data = DATA_FR
```

也就是目前會批次產生 `FR` 前後半車資料。若要改成產生 quarter 資料，可改成：

```python
ROADS = ROADS_Q
MODELS = MODELS_Q
data = DATA_Q
```

若要改成左右/roll 資料，可改成：

```python
ROADS = ROADS_LR
MODELS = MODELS_LR
data = DATA_LR
```

執行時建議先切到此資料夾：

```powershell
cd D:\Danny\DH3868\Suspension-model\dynamics_analyze
python data_creator.py
```

## 常見工作流程

### 跑單次 quarter-car 實驗

1. 開啟 `lab_quartersus.py`。
2. 在 simulation loop 中選擇一個 road input，例如 `qr.road_bump(state)`。
3. 選擇一個模型，例如 `sm.Quarter_suspension_model(sus,state)`。
4. 執行腳本後用 quarter visualizer 看 `zs`、`zu`、`zr`、`Fs`、`Ft` 等反應。

### 跑左右 roll 實驗

1. 開啟 `lab_LRsus.py`。
2. 選擇 `LR_road` 的路面或過彎輸入，例如 `hr.Cornering_left(state)`。
3. 選擇 `LR_model` 的半車模型，例如 `sm.Half_suspension_model(sus,state)`。
4. 用 `RollCarVisualizer` 看車身 roll 與左右輪反應。

### 跑前後 pitch 實驗

1. 開啟 `lab_FRsus.py`。
2. 選擇 `FR_road` 的輸入，例如 `hr.breaking(state)` 或 `hr.accelerate(state)`。
3. 選擇 `FR_model` 的模型。
4. 用 `PitchCarVisualizer` 看 pitch 與前後輪反應。

### 載入既有結果

1. 開啟 `load_and_show.py`。
2. 修改 `result_file` 指向想看的 `.pkl`。
3. 根據資料類型選擇 visualizer，例如 quarter 用 `Visualizer`，LR 用 `RollCarVisualizer`。

## 目前看到的注意事項

- `data_creator.py` 底部會直接跑完整批次模擬；執行前要先確認 `ROADS`、`MODELS`、`data` 指到想要的分析類型。
- 多個模型內有 `print()` debug 輸出，例如 roll/pitch moment 計算，批次執行時終端輸出可能很多。
- `FR` 的 `breaking` 函式名稱應是煞車，但英文拼字通常是 `braking`；目前程式使用 `breaking`，改名前需同步更新引用。
- `data/FR/*_ass.pkl` 可能是 accelerate 的縮寫或筆誤；若要讓檔名更直覺，可考慮改成 `accel` 或 `accelerate`。
- `.pkl` 是 Python pickle 格式，適合本專案快速讀寫，但不適合當長期跨版本資料交換格式。若要分享給其他工具，建議另存 CSV、Parquet 或 JSON。

## 這個資料夾與整體專案的關係

`dynamics_analyze` 是全車懸吊模型的簡化實驗場。相較於 `Suspension_sys/` 的完整系統模型，這裡更適合快速做：

- quarter-car 垂向反應比較。
- 左右輪輸入造成的 roll 與載重轉移分析。
- 前後輪輸入、加速、煞車造成的 pitch 分析。
- 不同模型複雜度之間的差異比較。
- 產生 `.pkl` 後用視覺化工具檢查時間域反應。

簡單說，`dynamics_analyze` 的目的就是用較小、較透明的模型，快速驗證懸吊動態想法，再把可信的概念帶回更完整的全車模型。
