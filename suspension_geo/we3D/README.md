# 轉向造成的左右輪 Camber

`steering_camber.py` 輸入左右前輪的**輪端轉角**，計算固定車身、水平地面下，由 KPI 與 caster 造成的左右輪 camber。預設從 `suspension_geo/car.py` 的 `SuspensionF` 讀取 KPI（`kpi`，rad）與 caster（`caster_sv_deg`，deg）；靜態 camber 預設為 0°。

## 執行

在專案根目錄執行，角度單位均為 **deg**：

```powershell
# 左右輪分別輸入：左輪左轉 25°、右輪左轉 18°
python -m suspension_geo.we3D.steering_camber --left 25 --right 18

# 只給一個角度：左右輪都左轉 20°（平行轉向）
python -m suspension_geo.we3D.steering_camber 20

# 右轉時，兩個角度都填負值
python -m suspension_geo.we3D.steering_camber --left -18 --right -25

# 不帶參數會提示輸入一個左右輪共用的轉角
python -m suspension_geo.we3D.steering_camber

# 覆寫設計參數與各輪靜態 camber
python -m suspension_geo.we3D.steering_camber --left 25 --right 18 --kpi 10 --caster 4 --static-camber-left -2 --static-camber-right -2
```

也可直接執行 `python suspension_geo/we3D/steering_camber.py --left 25 --right 18`，或進入 `we3D` 後執行 `python steering_camber.py --left 25 --right 18`。需要 `numpy`；匯入計算函式不會開啟圖形或讀寫結果檔案。

## 輸入與輸出定義

- 轉向角：輪胎平面與水平地面的交線，相對車頭的俯視角。**左轉為正、右轉為負**，左右輪使用相同正負號；不是方向盤轉角。
- Camber：**輪胎上端向車外為正、向車內為負**，左右輪都以各自的車外方向判斷。
- `camber_left_deg`、`camber_right_deg`：轉向後的有號 camber，包含指定的靜態 camber。
- `camber_change_left_deg`、`camber_change_right_deg`：相對直行的 camber 變化量，`camber(δ) - camber(0)`，單位 **deg**。
- `camber_gain_left_deg_per_deg`、`camber_gain_right_deg_per_deg`：指定轉角下的局部斜率 `d(camber)/d(輪端轉角)`，單位 **deg/deg**。不是總變化量除以轉角，也不是前視懸吊壓縮的 **deg/mm**。
- `kingpin_rotation_left_deg`、`kingpin_rotation_right_deg`：為達到輸入輪端轉角，實際需要繞主銷旋轉的角度。

例如目前 `car.py` 的 KPI = 10°、caster = 4°、靜態 camber = 0°，左右輪都左轉 20° 時：

| 輸出 | 左輪 | 右輪 |
| --- | ---: | ---: |
| Camber / 相對直行變化量 [deg] | +1.984851 | -0.760101 |
| 局部 camber gain [deg/deg] | +0.126639 | -0.005389 |
| 繞主銷旋轉角 [deg] | +20.406001 | +20.324340 |

## 在其他 Python 程式呼叫

```python
from suspension_geo.we3D.steering_camber import calculate_steering_camber

result = calculate_steering_camber(
    steer_left_deg=25.0,
    steer_right_deg=18.0,
    static_camber_left_deg=-2.0,
    static_camber_right_deg=-2.0,
)
print(result["camber_change_left_deg"], result["camber_change_right_deg"])
print(result["camber_gain_left_deg_per_deg"], result["camber_gain_right_deg_per_deg"])
```

可傳入 `suspension=SuspensionF()` 或修改過的前懸吊物件。每次呼叫都讀取該物件目前的參數，不修改物件。只傳 `steer_left_deg` 時，右輪使用相同角度；已知 Ackermann 或量測的左右輪角度時，請分別傳入，不會自行假設左右轉角關係。

## 計算方式與適用範圍

使用右手座標：X 向前、Y 向左、Z 向上。令左輪 `s = +1`、右輪 `s = -1`，主銷向上單位向量由 `[-tan(caster), -s × tan(KPI), 1]` 正規化取得。這使 caster 與 KPI 分別等於主銷在側視、前視投影的傾角；正 caster 的主銷上端向後，正 KPI 的上端向車內。

先建立帶有靜態 camber 的朝外輪軸單位向量，再用 Rodrigues 公式繞主銷旋轉。反解使輪胎平面符合輸入俯視轉角的主銷旋轉角，取從直行所需旋轉量最小且輪軸仍朝外的解。Camber 由旋轉後的輪軸與水平面之間的有號角度計算，gain 則由解析導數 `d(camber)/d(主銷角)` 除以 `d(輪端轉角)/d(主銷角)` 得到。

`sus_3Dwe_v3.py` 視覺化的滑桿是**繞主銷的旋轉角**，傾角圖與即時讀值皆保留正負號：初始朝 +Y 的輪軸向量，旋轉後朝上為正、朝下為負。這是輪軸的有號仰角；本計算腳本則以**俯視輪端轉角**輸入，並以輪胎頂端向車外為正定義左右輪 camber。主銷傾斜時，兩種轉向角度通常略有不同。

模型只計算固定姿態的轉向幾何，不含懸吊壓縮／回彈、車身 roll、轉向頂升引起的姿態平衡、襯套變形或輪胎變形。Scrub radius 與 trail 影響位置、力矩與頂升，但不影響固定主銷方向下輪胎的旋轉朝向，因此這裡不需用它們求 camber。所有輸入角度須為有限值且嚴格介於 -90° 與 +90°；無法達成的幾何與導數奇異點會顯示錯誤。

驗證指令：

```powershell
python -m unittest suspension_geo.tests.test_steering_camber -v
```

## `sus_3Dwe_pose.py`：輪胎中心座標系與三軸旋轉角

這個新腳本沿用 `sus_3Dwe_v3.py` 的單輪模型、主銷方向公式及正負號，從 `Car` 讀取輪胎尺寸、從 `SuspensionF` 讀取目前的 KPI 與 caster。掃描 **−30° 到 +30°**，預設每 **0.5°** 一筆，共 **121 筆**。

```powershell
# 在專案根目錄產生檔案，再輸入 Y 讀取 CSV 並播放動畫
python -m suspension_geo.we3D.sus_3Dwe_pose

# 將 +30° 姿態存成 PNG；輸入 Y 後，動畫仍由 -30° 開始
python -m suspension_geo.we3D.sus_3Dwe_pose --angle 30

# 只輸出完整掃描 CSV 與 PNG，不開視窗
python -m suspension_geo.we3D.sus_3Dwe_pose --no-show

# 調整動畫速度：每幀 100 ms（預設 50 ms）
python -m suspension_geo.we3D.sus_3Dwe_pose --interval 100

# 也可直接在 we3D 目錄執行
python sus_3Dwe_pose.py --angle -30 --step 1
```

先輸出 CSV 與 PNG，然後終端顯示 `Enter Y to read the CSV and play...`。輸入 `Y`（小寫 `y` 也可）後，程式會**重新開啟剛輸出的 CSV**，按主銷角由 −30° 排列至 +30°，用 Matplotlib `FuncAnimation` 循環播放；輸入其他內容則結束。`--no-show` 不詢問、不開啟動畫。

動畫包含輪胎、固定參考座標系、位於輪胎中心的紅／綠／藍三軸（`xw`／`yw`／`zw`），以及 XYZ 旋轉角和輪心位置曲線。每幀的輪心、三軸方向、角度讀值與曲線指示線皆取自 CSV，**不會重新用 KPI／caster 計算姿態**；輪胎外形尺寸仍從 `Car` 讀取。可按 `Pause`／`Resume` 暫停或繼續，關閉動畫視窗即結束。

CSV 必須包含輸出格式中的全部欄位，數值須有限，三條輪胎單位軸須組成右手正交座標系；欄位缺失或資料無效時會顯示錯誤。原本的 `create_pose_figure(...)` 仍可在 Python 中呼叫，以滑桿查看由模型計算的姿態。

定義如下：

- 輸入角度是原 `sus_3Dwe_v3.py` 滑桿的**繞主銷旋轉角**，正值依主銷向量的右手定則；與 `steering_camber.py` 的俯視輪端轉角定義不同。
- 固定參考系以 **0° 時的輪胎中心為原點**，方向沿用原圖的 X／Y／Z；輪胎初始輪軸沿 +Y。`center_x_mm`／`center_y_mm`／`center_z_mm` 為此參考系下的輪心位置，並非車輛重心或地面原點座標。
- 原模型的主銷通過輪心，因此整個掃描中輪心皆為 **`(0, 0, 0)`**，只有輪胎座標系的方向改變。此腳本同樣未加入 scrub radius／trail 的主銷偏移或懸吊行程。
- `rx_deg`／`ry_deg`／`rz_deg` 是相對零轉向參考系的**固定軸 XYZ Euler 角**，採右手正向，明確使用 `R = Rz(rz) @ Ry(ry) @ Rx(rx)`；不是將主銷角分別乘上軸向量分量，也不直接等於 camber／輪軸仰角。
- 輪胎局部點轉到固定參考系：`p_ref = center_m + R @ p_wheel`。`R` 的三個 column 就是輪胎 X／Y／Z 單位軸在參考系下的方向。

預設輸出在腳本旁的 `wheel_pose_results/`，可用 `--output-dir` 改路徑：

| 檔案 | 內容 |
| --- | --- |
| `wheel_center_pose.csv` | 全掃描主銷角、輪心 XYZ [mm]、Euler XYZ [deg]、三條輪胎單位軸的參考系 XYZ 分量。`wheel_x_in_ref_x/y/z` 是輪胎 X 軸；Y、Z 軸以此類推。 |
| `wheel_center_pose.png` | 完整曲線與 `--angle` 指定的輪胎姿態；動畫不會自動覆寫這張圖，可在 Matplotlib 視窗另存。 |

供其他腳本呼叫時，匯入不會開啟圖形或輸出檔案：

```python
from suspension_geo.we3D.sus_3Dwe_pose import calculate_wheel_pose

pose = calculate_wheel_pose(30.0)
print(pose["center_m"])          # [m]
print(pose["euler_xyz_deg"])     # [Rx, Ry, Rz]，deg
print(pose["rotation_matrix"])   # 各 column = 輪胎 X/Y/Z 軸在參考系的方向
```

也可單獨重播現有 CSV，而不重新產生資料：

```python
import matplotlib.pyplot as plt
from suspension_geo.we3D.sus_3Dwe_pose import DEFAULT_OUTPUT_DIR, create_pose_animation

figure = create_pose_animation(DEFAULT_OUTPUT_DIR / "wheel_center_pose.csv", interval_ms=50)
plt.show()
```

驗證指令：

```powershell
python -m unittest suspension_geo.tests.test_sus_3Dwe_pose -v
```
