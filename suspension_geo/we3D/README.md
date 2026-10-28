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

`sus_3Dwe_v3.py` 現有視覺化的滑桿是**繞主銷的旋轉角**，且傾角圖使用絕對值；這支計算腳本以**俯視輪端轉角**輸入並保留左右輪 camber 的正負號。主銷傾斜時，兩種轉向角度通常略有不同。

模型只計算固定姿態的轉向幾何，不含懸吊壓縮／回彈、車身 roll、轉向頂升引起的姿態平衡、襯套變形或輪胎變形。Scrub radius 與 trail 影響位置、力矩與頂升，但不影響固定主銷方向下輪胎的旋轉朝向，因此這裡不需用它們求 camber。所有輸入角度須為有限值且嚴格介於 -90° 與 +90°；無法達成的幾何與導數奇異點會顯示錯誤。

驗證指令：

```powershell
python -m unittest suspension_geo.tests.test_steering_camber -v
```
