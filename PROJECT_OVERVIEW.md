# Suspension-model 資料夾說明

本資料夾是一組以賽車/車輛懸吊為核心的 Python 分析專案集合。整體目的不是單一可安裝套件，而是把懸吊幾何、彈簧阻尼 K&C、液壓/互聯系統、車身動態、四輪載重、輪胎 Pacejka Magic Formula、轉向反應、前視幾何與空力/車高檢查等不同分析任務放在同一個工作區中。

整理日期：2026-08-29

## 一句話總覽

這個專案主要用來建立與驗證車輛懸吊模型：從幾何點位與 roll center/camber/anti 特性，到彈簧阻尼與輪胎力，再到四輪載重、車身 heave/roll/pitch/warp 動態反應與視覺化輸出。

## 根目錄內容

| 路徑 | 內容與目的 |
| --- | --- |
| `Suspension_sys/` | 主要的全車懸吊系統模型。包含懸吊幾何、K&C 元件、液壓模型、全車狀態、路面輸入、STL 視覺化與模擬資料。 |
| `dynamics_analyze/` | 動態分析工作區。包含 quarter、left-right、front-rear 不同自由度模型、路面輸入、狀態參數、模擬資料產生器與視覺化工具。 |
| `suspension_geo/` | 懸吊幾何分析與互動視覺化。依 `fv/`、`sv/`、`tv/`、`we3D/` 分類各視角程式、mapping 與結果；共用輸入放在 `data/`。 |
| `Steering_response_analysis/` | 轉向/輪胎反應分析。包含四輪載重、車輛參數、wheel output 工具，以及 Pacejka Magic Formula 輪胎模型與 `.tir` 參數。 |
| `Aerodynamic and suspension inspection/` | 空力與懸吊檢查資料。包含四輪載重計算、roll/pitch/attitude、空力作用、CSV 結果、圖檔，以及「會不會撞地板」計算文件。 |
| `RENAME.md` | 一行 side slip angle 公式筆記。 |
| `.gitignore`, `.gitattributes` | Git 設定檔。 |
| `__pycache__/` | Python 執行後產生的快取資料夾，不屬於主要原始碼。 |

## 主要資料夾詳解

### `Suspension_sys/`

這是目前最接近「主系統」的資料夾，內部有 `pyproject.toml`、`uv.lock` 與 `.venv/`，代表它曾以 Python/uv 專案方式管理依賴。依賴包含 `numpy`、`pandas`、`matplotlib`、`scipy`、`pyvista`。

重要內容：

| 路徑 | 說明 |
| --- | --- |
| `model/Suspension_model_v12.py` | 全車懸吊動態模型核心。處理輪胎 K&C force、unsprung/sprung dynamics、shock force、液壓 modal 轉換、幾何力、車身力矩與狀態更新。 |
| `Vehicle_status.py` | 全車狀態資料結構，描述 sprung/unsprung 位移、速度、加速度、姿態等狀態與積分更新。 |
| `Suspension_LP03.py` | 懸吊參數/系統組合類別，應是全車懸吊元件的設定入口之一。 |
| `part_suspension_state.py` | FR/LR 等局部懸吊狀態類別。 |
| `road.py` | 路面輸入函式。 |
| `lab_fullsus.py`, `LP02_v.py`, `load_and_show.py`, `load_and_show STL_v2.py` | 實驗/展示腳本，用來跑模型或載入顯示 STL。 |
| `Data_extraction.py` | 從模擬/資料物件中擷取特定資料欄位。 |
| `static_state.json`, `final_state.json`, `data.pkl` | 模擬狀態或輸出資料。 |
| `STL/` | 車體與四輪 STL 模型，例如 `LP02.stl`、`W_FL.stl`、`W_FR.stl`、`W_RL.stl`、`W_RR.stl`。 |

`Suspension_sys/model/` 內部再分為幾個系統：

| 路徑 | 說明 |
| --- | --- |
| `model/KC_sys/` | 彈簧、阻尼、motion ratio、shock absorber 與 K&C builder/library。核心檔案有 `spring.py`、`damper.py`、`Shock_Absorbers.py`、`MotionRatio.py`、`KC_builder.py`、`KC_library.py`。 |
| `model/sus_geo/` | 懸吊幾何模型。包含 roll center、camber gain、anti rate、geometry force、geometry builder/library。 |
| `model/Hyd_steady.py` | 穩態液壓/互聯系統模型。 |
| `model/Hyd_Transient .py` | 暫態液壓模型；檔名在 `Transient` 後面有空白，引用或命令列操作時要特別注意。 |
| `model/data_manager/` | CSV 參數資料載入、路徑管理、備份與測試。 |

`Suspension_sys/model/data_manager/data/` 內有三個主要參數表：

| 檔案 | 說明 |
| --- | --- |
| `spring.csv` | 彈簧/剛度相關資料。 |
| `damper.csv` | 阻尼器資料。 |
| `motion_ratio.csv` | motion ratio 資料。 |

`Suspension_sys/data_visualization/` 包含 quarter、pitch、roll 與 dashboard 視覺化工具，以及 `Simulation_Logger.py` 與 `plot_results.py`，用來記錄與繪製模擬結果。

### `dynamics_analyze/`

這個資料夾是獨立的動態分析沙盒，將懸吊模型拆成較小的模型族，方便用不同路面輸入做比較。

重要內容：

| 路徑 | 說明 |
| --- | --- |
| `data_creator.py` | 批次產生模擬資料的腳本。它組合 road function、model function、state 與 suspension parameter，跑 0 到 2 秒的模擬並輸出 `.pkl`。 |
| `model/quarter_model.py` | quarter-car / single / double spring / one-eighth 等垂向模型。 |
| `model/LR_model.py` | 左右向 roll/左右輪模型，例如 two-mass、half、2DOF 等。 |
| `model/FR_model.py` | 前後向 pitch/front-rear 模型。 |
| `road/quarter_road.py` | quarter model 的 step、bump、impulse、sine、random road。 |
| `road/LR_road.py` | 左右輪路面與 cornering 輸入。 |
| `road/FR_road.py` | 前後輪路面、加速、煞車輸入。 |
| `Parameter_set/Suspension.py` | quarter/LR/FR 懸吊參數類別。 |
| `Parameter_set/suspension_state.py` | quarter/LR/FR 狀態類別與積分更新。 |
| `data_visualization/` | quarter、roll、pitch 動畫/圖表視覺化與 logger。 |
| `data/quarter/`, `data/LR/`, `data/FR/` | 已產生的模擬 `.pkl` 結果。 |

建議使用方式是先看 `data_creator.py` 的 `ROADS_*`、`MODELS_*`、`DATA_*` 字典，再決定要跑哪一組模型與路面輸入。

### `suspension_geo/`

這是懸吊幾何工作區，依前視、側視、上視與 3D 輪端分類；共用車輛參數、幾何與繪圖工具保留在根目錄。各類分析程式與結果放在同一個子資料夾。

重要內容：

| 路徑 | 說明 |
| --- | --- |
| `geometry.py` | 基礎幾何物件，如 `Point`、`Link`、`RigidBody`、`Polygon`、`Wheel`、`Body`。 |
| `car.py` | 車輛幾何與參數。 |
| `Geometric_Analysis.py` | 幾何計算工具與懸吊側邊模型，包括旋轉、圓交點、四連桿模擬、roll center 計算等。 |
| `fv/sus_fv_v5.py` | Matplotlib 互動滑桿視覺化主腳本，可調整左右 lower arm angle 並即時更新 roll center。 |
| `fv/sus_fv_maping.py`, `fv/rc_camber_map.py` | 幾何 map/roll center/camber 相關分析。 |
| `fv/run15_pose_analysis.py` | Run 15 前視分析，CSV/PNG 結果保存在 `fv/`。 |
| `sv/` | 側視互動程式、Run 15 anti 分析與 CSV/PNG 結果。 |
| `tv/`, `we3D/` | 上視轉向幾何與 3D 輪端分析。 |
| `data/data_run15_selected.csv` | FV、SV 共用的 Run 15 輸入。 |
| `visualization.py` | Matplotlib 繪圖物件封裝，例如 point/link/polygon plot。 |
| `fv/suspension_kinematics_data.pkl` | 幾何掃描或運動學資料。 |

如果要理解「懸吊幾何怎麼被建出來」，建議閱讀順序是：`car.py` -> `geometry.py` -> `Geometric_Analysis.py` -> `fv/sus_fv_v5.py`。完整目錄與執行方式見 `suspension_geo/README.md`。

### `Steering_response_analysis/`

這個資料夾負責轉向與輪胎輸出相關計算。它將四輪垂直載重與輪胎 Magic Formula 模型結合，輸出四輪 `Fx`、`Fy`、`Fz`、`Mx`、`Mz`、rolling resistance torque 等。

重要內容：

| 路徑 | 說明 |
| --- | --- |
| `wheel.py` | 車輪輸出工具。包含 slip angle、slip ratio、四輪 Magic Formula 輸出與 friction circle 繪圖。 |
| `Four_wheel_load.py` | 四輪載重求解工具，與空力/檢查資料夾中的版本功能相近。 |
| `Vehicle_parameters.py` | 車輛質量、重心高度、軸距、輪距、ride rate、tire stiffness、modal stiffness 等參數。 |
| `main.py` | 轉向分析試跑腳本；目前可看出會呼叫四輪載重與輪胎輸出，但 `ax`、`ay`、`SA_list` 等變數需先定義才能直接執行。 |
| `Pacejka_MF_model/` | Pacejka Magic Formula 輪胎模型套件、說明文件、圖表與 `.tir` 參數。 |

`Pacejka_MF_model/` 內的重點：

| 路徑 | 說明 |
| --- | --- |
| `model/api.py` | 輪胎模型對外 API，供 `wheel.py` 呼叫。 |
| `model/Fx_models.py`, `Fy_models.py`, `Mx_models.py`, `Mz_models.py`, `RRT_models.py` | 各方向力/力矩模型。 |
| `model/_common.py` | Magic Formula 共用計算工具。 |
| `tir/FSAE_43075R20.tir`, `tir/D2704_mf612.tir` | 輪胎參數檔。 |
| `Demo.py`, `MF_tester.py` | 範例與測試腳本。 |
| `Figures/` | 模型輸出圖，例如固定 Fz、固定 slip angle、固定 slip ratio、friction circle 等結果。 |
| `introduction*.md`, `model/introduction/*.md` | Magic Formula 與各模型的說明文件。 |

### `Aerodynamic and suspension inspection/`

這個資料夾看起來是空力與懸吊檢查的分析結果區，原本可能叫做「前翼計算」，目前 Git 狀態顯示可能正在改名成英文資料夾。

重要內容：

| 路徑 | 說明 |
| --- | --- |
| `Four_wheel_load.py` | 四輪載重求解，支援 CG 法、least squares、Lagrange、coupled stiffness 等不同方法，並可加入外力/空力作用點。 |
| `roll.py` | roll、pitch、attitude、aero force 等簡化計算。 |
| `ay_ax_hf.csv`, `ax_pitch.csv`, `ay_roll.csv`, `hf_v.csv` | 空力/姿態/車高掃描結果資料。 |
| `Figure_1.png`, `Figure_2.png`, `Figure_3.png` | 分析輸出圖。 |
| `會不會撞地板計算.md`, `會不會撞地板計算.pdf` | 車高/撞地板檢查文件。 |
| `前翼計算.zip` | 相關分析封存檔。 |

## 資料與輸出類型

| 類型 | 數量 | 用途 |
| --- | ---: | --- |
| `.py` | 80 | 模型、分析腳本、視覺化與資料處理。 |
| `.pkl` | 66 | 模擬輸出與幾何/動態資料快取。 |
| `.png` | 22 | Pacejka 與空力/懸吊分析圖表。 |
| `.md` | 13 | 輪胎模型、分析公式與專案說明文件。 |
| `.csv` | 10 | 彈簧、阻尼、motion ratio、空力/姿態掃描資料。 |
| `.stl` | 5 | 車體與輪胎 3D 視覺化模型。 |
| `.json` | 3 | 懸吊系統狀態輸入/輸出。 |
| `.tir` | 2 | 輪胎 Magic Formula 參數。 |

以上統計已排除 `.git`、`.venv` 與 `__pycache__`。

## 建議閱讀順序

1. 想理解全車懸吊系統：先看 `Suspension_sys/model/Suspension_model_v12.py` 的流程圖與函式，再看 `Suspension_sys/Vehicle_status.py`、`Suspension_sys/Suspension_LP03.py`。
2. 想理解彈簧/阻尼/K&C：看 `Suspension_sys/model/KC_sys/`，從 `spring.py`、`damper.py`、`MotionRatio.py` 到 `KC_builder.py`。
3. 想理解幾何效應：看 `Suspension_sys/model/sus_geo/` 與 `suspension_geo/Geometric_Analysis.py`。
4. 想跑簡化動態模型：看 `dynamics_analyze/data_creator.py`，再看 `dynamics_analyze/model/` 與 `dynamics_analyze/road/`。
5. 想分析輪胎與轉向：看 `Steering_response_analysis/wheel.py` 與 `Steering_response_analysis/Pacejka_MF_model/model/api.py`。
6. 想看空力造成的載重/姿態/車高影響：看 `Aerodynamic and suspension inspection/Four_wheel_load.py`、`roll.py` 與該資料夾中的 CSV/PNG/MD。

## 執行與環境

`Suspension_sys/pyproject.toml` 顯示此專案需要：

```toml
requires-python = ">=3.14"
dependencies = [
    "matplotlib>=3.11.1",
    "numpy>=2.5.1",
    "pandas>=3.0.5",
    "pyvista>=0.48.4",
    "scipy>=1.18.0",
]
```

注意：根目錄不是標準單一 Python package，多個子資料夾使用相對 import 或直接從該資料夾執行腳本。若要執行特定腳本，建議先切到對應資料夾，再執行該檔案。

例如：

```powershell
cd Suspension_sys
python lab_fullsus.py
```

或：

```powershell
cd dynamics_analyze
python data_creator.py
```

實際能否直接執行取決於本機 Python 版本、套件環境，以及腳本內是否還有未定義的實驗參數。

## 目前看到的注意事項

- `Suspension_sys/README.md` 目前是空檔，根目錄原本也沒有完整總覽文件，因此本檔作為新的專案地圖。
- `Suspension_sys/model/Hyd_Transient .py` 檔名含有空白，import 或命令列操作時容易出錯。
- `Steering_response_analysis/main.py` 中使用了 `ax`、`ay`、`SA_list` 等變數，但片段中未先定義；若要當主程式執行，需要補齊輸入。
- 專案中有 `.venv/`，它是本地虛擬環境，不建議當成專案原始碼閱讀或納入架構說明。
- Git 狀態顯示有資料夾改名/搬移的痕跡：舊的中文資料夾被標示刪除，新的 `Aerodynamic and suspension inspection/` 目前是未追蹤資料夾。這可能是正常改名流程，但提交前應確認是否要保留英文資料夾名稱。

## 專案目的整理

整體而言，這個資料夾的目的可以分成四層：

1. 建立車輛與懸吊參數：質量、重心、軸距、輪距、彈簧、阻尼、motion ratio、輪胎剛度與 Magic Formula 參數。
2. 建立懸吊幾何與力學模型：roll center、camber gain、anti geometry、幾何力、液壓互聯、heave/roll/pitch/warp modal 轉換。
3. 執行動態模擬與檢查：不同路面輸入、加速/煞車/過彎、四輪載重轉移、車身姿態、是否撞地板。
4. 視覺化與驗證結果：Matplotlib 動畫/圖表、PyVista STL 顯示、CSV/PKL 結果保存、Pacejka 圖表輸出。

換句話說，它是一個用來支援懸吊設計、車輛動態分析與參數驗證的研究/開發工作區。
