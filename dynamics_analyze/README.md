# dynamics_analyze：KC 物件設定

`suspension_set/Suspension.py` 定義車輛參數與各位置的 `KCSystem` 物件；
`model/model.py` 的三個模型直接呼叫物件的 `force(displacement, velocity)` 取得力量。
更換彈簧、阻尼或 motion ratio 時，只需修改或替換設定物件。

## 各模型使用的物件

| 設定類別 | 懸吊物件 | 輪胎物件 | 車架物件 |
| --- | --- | --- | --- |
| 四種 `SuspensionQuarter*` | `corner` | `tire` | — |
| `SuspensionLR` | `heave`、`roll`、`corner_left`、`corner_right` | `tire_left`、`tire_right` | `frame` |
| `SuspensionFR` | `heave_f`、`heave_r`、`corner_front`、`corner_rear` | `tire_front`、`tire_rear` | — |

目前的剛性設定如下，所有已安裝線性元件的阻尼係數均由 `zeta = 0.7` 計算：

- Quarter 的懸吊 k 分別為 `70000`、tender `23800`、avg `43000`、heave `70000` N/m，
  c 仍由 `2 * 0.7 * sqrt(k * 13)` 計算。tender 保留原本的線性近似。
- LR 的 heave k 為 `18000`、roll 為 `50000`、frame 為 `500000` N/m。
- FR 的前 heave k 為 `30000`、後 heave 為 `28000` N/m。
- Quarter 與 FR 輪胎 k 為 `80000`，LR 為 `800000` N/m。
- 原本 k、c 都為零的直推避震器使用 `NullKC()`；每個位置可獨立替換物件。

各設定類別以 `self.zeta = 0.7` 搭配 `c = 2 * self.zeta * sqrt(k * m)` 初始化阻尼。
沿用 Quarter 的參考質量慣例，懸吊與輪胎使用 `mu`，車架使用 `m_frame`。
這是各元件依參考質量設定的阻尼比，並不表示耦合多自由度模型的每個模態阻尼比都等於 0.7。
若修改剛性或參考質量，需同時依公式重新計算 c；單獨修改 `self.zeta` 不會更新已建立的物件。

## 修改設定

從專案根目錄匯入：

```python
import numpy as np
from dynamics_analyze.suspension_set.Suspension import SuspensionQuarter, SuspensionLR
from dynamics_analyze.model.KC_sys import LinearKC, Ohlins, SeriesKC

sus = SuspensionQuarter()
sus.tire = LinearKC(k=80000, c=2 * sus.zeta * np.sqrt(80000 * sus.mu), name="tire")
sus.corner = LinearKC(k=70000, c=1335.5148819767445, mr_value=1.0, name="corner")

# 也可以直接修改既有線性物件的參數。
sus.tire.spring.k = 90000
sus.tire.damper.c = 2 * sus.zeta * np.sqrt(sus.tire.spring.k * sus.mu)

lr = SuspensionLR()
lr.corner_left = LinearKC(k=10000, c=500, mr_value=0.9, name="corner_left")
lr.heave = Ohlins(
    k=18000,
    low_comp=1800, high_comp=700,
    low_reb=2500, high_reb=1000,
    transition=0.05,
    name="heave",
)

# 真正的串聯彈簧可自行替換，不會自動套用到原本 tender 設定。
sus.corner = SeriesKC(
    main=50000, low=15000, travel=0.03,
    low_comp=1800, high_comp=700,
    low_reb=2500, high_reb=1000,
    transition=0.05,
    name="corner_series",
)
```

`LinearKC`、`SeriesKC`、`Ohlins`、`LookupKC` 與 `NullKC` 均使用相同力量介面。
`force()` 回傳 `(force, details)`；模型使用第一個值，第二個值包含彈簧力、阻尼力等資訊。
motion ratio 的平方轉換由 `KCSystem` 處理，模型不再重複相乘。
Quarter 的 `ride_rate` 依目前物件在零位移的輪端剛度計算，替換物件後會同步更新。

## 力量正負號

`KCSystem.force(x, v)` 對線性系統回傳 `-(k*x + c*v) * MR**2`。
懸吊輸入為 `zs - zu`、`vzs - vzu`；輪胎輸入為 `zu - zr`、`vzu - vzr`。
Quarter 與 LR 記錄的 `Fs`、`Ft` 延用原本反力符號，因此取 KC 回傳值的負值。
FR 的 `Fs` 同樣記錄反力；`Ft` 延用原本的「簧下合力」定義，包含輪胎、懸吊與縱向載重轉移。

FR 原先輪胎式的阻尼項為 `+c*(vzu-vzr)`；現在直接使用 KC 力量，改為
`-c*(vzu-vzr)`，讓阻尼抵抗相對運動。因此 FR 的動態結果會有這項修正造成的差異。

## 執行與驗證

在專案根目錄執行：

```powershell
python -m dynamics_analyze.lab_tender
python -m dynamics_analyze.lab_rollcenter
python -m dynamics_analyze.lab_FRsus
python -m unittest discover -s dynamics_analyze/tests -v
```

也可切到 `dynamics_analyze` 後直接執行各 `lab_*.py`。
`lab_rollcenter` 使用現有的 `Cornering_left(t0=0.0)` 階躍轉彎輸入與
`Half_rollcenter_LR_model`；`lab_FRsus` 使用 `accelerate(ax=-2)` 作為煞車輸入。
這些呼叫取代了原本尚未定義的 `Cornering_ramp_to_max`、`Half_rollcenter_all_model` 與 `breaking`。

測試涵蓋剛性與阻尼比、線性公式比對、KC 物件替換、非線性系統、motion ratio、
四種 Quarter 的完整凸起路面軌跡、兩種半車的兩秒模擬，以及從實驗目錄匯入。
