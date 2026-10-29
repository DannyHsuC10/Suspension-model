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

`sus_3Dwe_pose.py` 與 `sus_3Dwe_v3.py` 共用 `wheel_end_geometry.py` 的單輪模型、主銷方向與初始輪心偏移，從 `Car` 讀取輪胎尺寸，從 `SuspensionF` 讀取 KPI、caster、`mechanical_trail_sv` 與 `scrub_radius`。掃描 **−30° 到 +30°**，預設每 **0.5°** 一筆，共 **121 筆**。

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

動畫包含輪胎、固定參考座標系、位於輪胎中心的紅／綠／藍三軸（`xw`／`yw`／`zw`）、輪心軌跡，以及 XYZ 旋轉角和輪心位置曲線。每幀的輪心、三軸方向、角度讀值與曲線指示線皆取自 CSV，**不會重新用 KPI／caster 計算姿態**；新的 CSV 另儲存固定主銷方向，因此動畫中的黑色主銷線也使用檔案資料。輪胎外形尺寸仍從 `Car` 讀取。可按 `Pause`／`Resume` 暫停或繼續，關閉動畫視窗即結束。

CSV 必須包含輸出格式中的全部欄位，數值須有限，三條輪胎單位軸須組成右手正交座標系；欄位缺失或資料無效時會顯示錯誤。原本的 `create_pose_figure(...)` 仍可在 Python 中呼叫，以滑桿查看由模型計算的姿態。

定義如下：

- 輸入角度是原 `sus_3Dwe_v3.py` 滑桿的**繞主銷旋轉角**，正值依主銷向量的右手定則；與 `steering_camber.py` 的俯視輪端轉角定義不同。
- 固定參考系以**靜態主銷軸與地面的交點為原點 `(0, 0, 0)`**，初始地面為 **`Z = 0`**，主銷固定穿過原點，輪胎初始輪軸沿 +Y。`center_x_mm`／`center_y_mm`／`center_z_mm` 都相對這個固定地面原點；靜態輪心座標為 `[trail, -scrub, tire_radius]`。
- 本版原點已由先前的主銷輪心高度移到**靜態地面交點**；先前產生的 CSV 需要重新生成。這次只平移固定參考系，不改變輪胎運動、旋轉角度或軸向量。計算轉向位移時，仍將各筆 `center_*_mm` 減去 **0° 那筆**的輪心位置。
- `rx_deg`／`ry_deg`／`rz_deg` 是相對零轉向參考系的**固定軸 XYZ Euler 角**，採右手正向，明確使用 `R = Rz(rz) @ Ry(ry) @ Rx(rx)`；不是將主銷角分別乘上軸向量分量，也不直接等於 camber／輪軸仰角。
- 輪胎局部點轉到固定參考系：`p_ref = center_m + R @ p_wheel`。`R` 的三個 column 就是輪胎 X／Y／Z 單位軸在參考系下的方向。

### Mechanical trail／scrub 的初始偏移與旋轉

兩個腳本保留原 `v3` 的主銷公式，令 `a` 為向上的主銷單位向量、`r = Car.free_radius`。初始參考地面為 **`z = 0`**；正 trail 表示初始胎底相對主銷地面交點沿 **+X** 偏移，正 scrub 表示沿 **−Y** 偏移（此單輪模型的外側）。這是本專案原 wheel-end／SV／FV 模型的符號，與 `steering_camber.py` 的左右輪座標定義不同。

```text
a = normalize([sin(caster), sin(KPI), cos(KPI) * cos(caster)])
G = [0, 0, 0]                        # 靜態主銷與地面交點，固定原點
P0 = [mechanical_trail, -scrub, 0]    # 初始胎底參考點，位於 z = 0
C0 = [mechanical_trail, -scrub, r]    # 初始輪心，z = 輪胎自由半徑

C(δ) = R(a, δ) @ C0                   # 轉向後輪心
P(δ) = R(a, δ) @ P0                   # 初始胎底材質點的旋轉軌跡
p_ref = R(a, δ) @ (C0 + p_wheel)      # 所有輪胎點同樣先偏移再旋轉
```

以目前 `car.py` 的 trail **15 mm**、scrub **25 mm**、自由半徑 **205.74 mm**，靜態胎底為 **`P0 = (15, -25, 0) mm`**，靜態輪心為 **`C0 = (15, -25, 205.74) mm`**。轉向時輪心會繞固定主銷走圓弧，與主銷的垂直距離及沿軸分量保持不變；輪胎 XYZ Euler 朝向仍由原主銷方向與轉角決定。

與上一版輪心高度原點的關係：`G_old = -r * a / a.z`，`p_new = p_old - G_old`。因為 `G_old` 位於同一條主銷軸上，平移後仍可直接使用 `R @ p_new_initial`，不需要改變旋轉矩陣。

這裡計算固定主銷下的剛體運動，未解懸吊行程、車身升降平衡或輪胎變形。`v3` 的胎底曲線是**初始胎底材質點**的軌跡，轉向後不會強制貼住原地面，也不等同重新求得的實際接地點；圖中的力矩臂以這個參考點與固定主銷計算。

預設輸出在腳本旁的 `wheel_pose_results/`，可用 `--output-dir` 改路徑：

| 檔案 | 內容 |
| --- | --- |
| `wheel_center_pose.csv` | 全掃描主銷角、相對靜態主銷地面交點的輪心 XYZ [mm]、Euler XYZ [deg]、三條輪胎單位軸及固定主銷方向。`wheel_x_in_ref_x/y/z` 是輪胎 X 軸；Y、Z 軸以此類推；`kingpin_axis_x/y/z` 為主銷單位向量。 |
| `wheel_center_pose.png` | 完整曲線與 `--angle` 指定的輪胎姿態；動畫不會自動覆寫這張圖，可在 Matplotlib 視窗另存。 |

仍可載入不含三個 `kingpin_axis_*` 欄位的舊版 CSV；這種檔案不會顯示黑色主銷線。讀檔不會自動轉換舊版座標原點，若要使用本版的地面原點，應重新產生 CSV。

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
