# 懸吊模型改進方向

本文件整理 `dynamics_analyze`、`suspension_geo`、`Suspension_sys` 三個主要資料夾的後續改進方向。重點不是一次大改，而是把目前已經能運作的研究型程式，逐步整理成更穩定、可重複驗證、可擴充的懸吊分析工具。

整理日期：2026-08-29

## 總體方向

目前三個資料夾各自有清楚用途：

| 資料夾 | 現在定位 | 建議未來定位 |
| --- | --- | --- |
| `dynamics_analyze/` | 簡化動態模型與時間域測試沙盒。 | 作為模型驗證與基準測試平台。 |
| `suspension_geo/` | 依 `fv/`、`sv/`、`tv/`、`we3D/` 分類的懸吊幾何工具與分析結果。 | 作為幾何資料產生器，輸出可被動態模型使用的查表資料。 |
| `Suspension_sys/` | 全車懸吊整合模型。 | 作為主模擬核心，整合 K&C、幾何、輪胎與車身動態。 |

建議的總體改進方向：

1. 統一資料格式與命名規則。
2. 把 debug/實驗腳本與核心模型分離。
3. 建立可重複執行的測試案例。
4. 讓 `suspension_geo` 的幾何 map 可以接到 `Suspension_sys` 的 geometry lookup。
5. 讓 `dynamics_analyze` 成為 `Suspension_sys` 的簡化驗證基準。
6. 減少 import 時自動執行、印字或產生圖表的行為。

## 優先處理項目

| 優先級 | 項目 | 影響 |
| --- | --- | --- |
| 高 | 修正 import 時自動執行的程式碼 | 讓模組可以安全被其他腳本引用。 |
| 高 | 統一 wheel order、座標方向、正負號定義 | 避免力、位移、roll/pitch 判讀錯誤。 |
| 高 | 移除或開關大量 `print()` debug 輸出 | 批次模擬與視覺化會更乾淨。 |
| 中 | 建立共用 `config/parameters` | 避免三個資料夾各自寫一套車輛參數。 |
| 中 | 把 `.pkl` 結果加上 metadata | 方便知道資料由哪個模型、參數、路面輸入產生。 |
| 中 | 增加基本測試 | 確認模型改動後不會破壞力平衡與資料格式。 |
| 低 | 整理檔名與拼字 | 提升可讀性與未來維護性。 |

## `dynamics_analyze/` 改進方向

`dynamics_analyze` 適合保留為簡化模型驗證區。它的價值是模型小、物理意義清楚、容易測試。

### 短期改進

- 把 `data_creator.py` 底部直接執行批次模擬的程式包進 `if __name__ == "__main__":`。
- 將目前手動切換的：

```python
ROADS = ROADS_FR
MODELS = MODELS_FR
data = DATA_FR
```

改成可由參數選擇，例如 `--mode quarter`、`--mode LR`、`--mode FR`。

- 修正命名：
  - `breaking` 建議改成 `braking`。
  - `ass` 建議改成 `accel` 或 `accelerate`。
- 移除模型函式內的 debug `print()`，或改成 `debug=False` 控制。
- 在 `.pkl` metadata 中記錄：
  - model name
  - road name
  - dt
  - simulation time
  - suspension parameters

### 中期改進

- 建立 `README` 中提到的標準案例，例如：
  - quarter bump response
  - LR opposite sine roll response
  - FR braking pitch response
- 建立簡單驗證測試：
  - 靜止無輸入時，位移不應發散。
  - 左右同步輸入時，roll 應接近 0。
  - 前後同步輸入時，pitch 應接近 0。
- 把 state、suspension、road、model 的 interface 寫清楚，避免新增模型時不知道該更新哪些欄位。
- 將 logger output 統一成可轉 CSV 或 DataFrame 的格式。

### 長期改進

- 把 `dynamics_analyze` 變成 `Suspension_sys` 的 benchmark。
- 每當 `Suspension_sys` 主模型改動時，用簡化模型做 sanity check。
- 逐步把 quarter/LR/FR 模型與全車模型的參數來源合併，避免同一個車重、輪距、剛度在多個檔案重複定義。

## `suspension_geo/` 改進方向

`suspension_geo` 的核心價值是產生幾何 map。它應該逐步從「互動畫圖工具」升級成「幾何資料產生器」。

### 短期改進

- 把 `car.py` 底部直接呼叫 `Car()` 移到 `if __name__ == "__main__":`。
- 把 `Geometric_Analysis.py` 中 `calc_roll_center()` 的 `print("rc_pos", rc_pos)` 改成可選 debug。
- 修正 `SuspensionSide.update_kinematics()` 中 force line 更新方式：
  - 目前有 `self.upper_force.p2 = self.IC`。
  - 但 `Link` 類別使用 `start` / `end` 屬性。
  - 建議確認是否應改成更新 `end`，避免 force line 沒有跟著 IC 正確更新。
- `geometry.Link` 中 `angle` property 定義兩次，建議刪除重複定義。
- 為 `fv/suspension_kinematics_data.pkl` 加上版本與參數 metadata。

### 中期改進

- 將 `fv/sus_fv_maping.py` 的掃描輸出格式標準化，例如：

```text
geometry_map/
  front_geometry_map.pkl
  front_geometry_map.csv
  metadata.json
```

- 將 camber、RC height、IC position 對 heave/roll/travel 的關係輸出成查表格式，供 `Suspension_sys/model/sus_geo/TableRollCenter`、`TableCamber` 使用。
- 增加幾何求解失敗處理：
  - 兩圓無交點時記錄失敗狀態。
  - RC 無交點時保留 `nan` 並記錄原因。
- 把 Matplotlib 互動顯示與資料掃描分開，避免 import mapping 腳本時直接畫圖。

### 長期改進

- 讓 `suspension_geo` 成為正式的 suspension geometry preprocessor。
- 支援前/後懸吊各自不同幾何。
- 支援輸出：
  - roll center height vs roll/heave
  - camber gain vs travel
  - track change vs travel
  - scrub change vs travel
- 將輸出 map 直接餵給 `Suspension_sys` 的 `create_table_geometry()`。

## `Suspension_sys/` 改進方向

`Suspension_sys` 是最重要的主模型。後續改進應優先讓它穩定、可測試、可輸入真實幾何/K&C 資料。

### 短期改進

- 把 `Suspension_LP03.py` 底部的 `Suspension_A()` 移到 `if __name__ == "__main__":`。
- 把 `model/Suspension_model_v12.py` import 時自動印出的流程圖改成文件或 debug mode。
- 移除或控制 `sprung_moment()` 的大量 `print()`。
- 檢查 `tire_normal_force()`：

```python
N = np.maximum(Ft_kc, np.zeros(4))
N = Ft_kc
```

第二行會覆蓋非負限制。需要決定法向力是否允許為負，並刪掉矛盾邏輯。

- 修正 `road_FL_bump()` 註解與程式不一致：
  - 註解寫 5 cm。
  - 程式是 `0.01 m`，也就是 1 cm。
- 將 `Hyd_Transient .py` 改名成 `Hyd_Transient.py`，避免檔名空白造成引用問題。

### 中期改進

- 建立明確的座標與正負號文件：
  - `Fx`, `Fy`, `Fz` 正方向。
  - `theta = [front_roll, rear_roll, pitch, yaw]` 的正方向。
  - `zs`, `zu`, `zr` 壓縮/伸張方向。
  - `Fs`, `Ft`, `N` 的作用方向。
- 建立 `VehicleConfig` 或 `SuspensionConfig`，集中管理：
  - mass
  - CG height
  - wheelbase
  - track
  - inertia
  - tire radius
  - K&C 設定
  - geometry 設定
- 把 `Suspension_A` 拆成：
  - vehicle parameters
  - K&C setup
  - tire setup
  - geometry setup
  - inertia setup
- 讓 `lab_fullsus.py` 可用參數切換 road input、模擬時間、初始狀態與視覺化模式。
- 將 `SimulationLogger` metadata 補齊，記錄模型版本、輸入、參數與初始 state。

### 長期改進

- 將 `suspension_geo/fv` 產生的 RC/camber map 接入 `model/sus_geo/create_table_geometry()`。
- 將輪胎 Pacejka 模型與垂向懸吊模型更明確整合：
  - `N` 由懸吊垂向模型輸出。
  - Pacejka 使用 `N`、`SA`、`SL`、`IA` 計算 `Fx/Fy/Mz`。
  - `Fx/Fy` 再回饋給 anti/roll center 幾何力。
- 建立完整 closed-loop 單步流程：

```text
road / driver input
  -> suspension vertical dynamics
  -> tire normal force
  -> tire lateral/longitudinal force
  -> geometry force
  -> body dynamics
  -> state integration
```

- 建立 regression tests，確保：
  - 靜態狀態力平衡。
  - 無輸入時狀態不漂移。
  - 左右對稱輸入不產生不合理 roll。
  - 前後對稱輸入不產生不合理 pitch。
  - 單輪 bump 的 response 不爆炸。

## 三個資料夾的整合建議

### 建議資料流

```text
suspension_geo/
  產生幾何 map
      |
      v
Suspension_sys/model/sus_geo/
  使用 TableRollCenter / TableCamber / TableAnti
      |
      v
Suspension_sys/
  全車懸吊與輪胎力整合模擬
      |
      v
dynamics_analyze/
  用簡化模型做 sanity check 與 benchmark
```

### 共用設定

建議未來新增一個共用設定位置，例如：

```text
config/
  vehicle.yaml
  suspension_kc.yaml
  geometry_front.yaml
  tire.yaml
```

三個資料夾都從同一份設定讀取車重、軸距、輪距、CG、剛度、阻尼與輪胎半徑，避免同一個數字在不同檔案出現不同版本。

### 共用資料格式

建議所有模擬結果至少包含：

| 欄位 | 說明 |
| --- | --- |
| `data` | 時間序列資料。 |
| `metadata` | 模型名稱、輸入名稱、參數、版本、產生時間。 |
| `state_schema` | 主要欄位的意義、單位與 shape。 |

### 命名規則

建議統一：

| 類型 | 建議 |
| --- | --- |
| 輪序 | 固定使用 `[FL, FR, RL, RR]`。 |
| 單位 | 程式內統一 SI unit，圖表標籤再轉 mm/deg。 |
| 檔名 | 使用英文小寫與底線，例如 `hyd_transient.py`、`braking`、`accelerate`。 |
| 模型函式 | 使用一致命名，例如 `quarter_model()`、`half_lr_model()`、`half_fr_model()`。 |
| debug | 使用 `debug=False` 或 logging，不直接在核心函式 print。 |

## 建議執行順序

如果要逐步整理，建議照以下順序：

1. 先修所有 import side effect：避免 import 時自動執行、印圖、跑模擬。
2. 統一 wheel order、單位與正負號文件。
3. 整理 debug print。
4. 為 `dynamics_analyze` 建立基本 benchmark cases。
5. 為 `suspension_geo/fv` 的 pkl 加上 metadata，並輸出 geometry lookup table。
6. 將 lookup table 接進 `Suspension_sys/model/sus_geo/`。
7. 建立全車模型 regression tests。
8. 再做架構重構與命名整理。

## 最終目標

理想狀態下，三個資料夾可以形成一條清楚的開發鏈：

- `suspension_geo` 負責幾何。
- `dynamics_analyze` 負責簡化模型驗證。
- `Suspension_sys` 負責完整全車模擬。

這樣未來調整懸吊幾何、彈簧阻尼、輪胎參數或車身設定時，就可以先用簡化模型確認方向，再用全車模型驗證完整動態反應，最後用視覺化工具檢查結果是否合理。
