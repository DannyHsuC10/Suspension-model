# static 懸吊幾何靜態視圖

`static` 提供 FV（前視）、SV（側視）、TV（上視）與 3D 輪端四種視圖。各腳本以單一圖窗顯示初始靜態幾何，僅保留固定姿態的幾何計算與繪圖。

## 執行方式

從專案根目錄執行，Python 環境需要 `numpy` 與 `matplotlib`：

```powershell
python -m static.fv.sus_fv_v5
python -m static.sv.sus_sv_v4
python -m static.tv.sus_tv_v2
python -m static.we3D.sus_3Dwe_v3
```

也可直接執行，例如 `python static/fv/sus_fv_v5.py`；或進入對應子資料夾後執行腳本。

## 視圖內容

| 腳本 | 顯示內容 |
| --- | --- |
| `fv/sus_fv_v5.py` | 在同一張 3D 靜態圖顯示相距一個軸距的前、後 FV 平面、中央的垂直 SV 平面、四輪的合成 3D 主銷與上、下球接頭，以及半透明六面體車身。 |
| `sv/sus_sv_v4.py` | 前後軸初始側視幾何與瞬心（IC），輪心縱向間距為軸距。 |
| `tv/sus_tv_v2.py` | 初始轉向幾何。 |
| `we3D/sus_3Dwe_v3.py` | 初始輪胎、主銷軸、輪心、輪軸、輪胎底部參考點及其對主銷的靜態力臂。 |

FV 使用唯一的固定設計瞬心 `IC`，座標由輪距、FVSA（`fvsa`）與設計 RC 高度（`h_RC`）決定，不再另外求解或更新瞬心。SV 的 IC 仍由側視靜態幾何計算。

FV 的前、後軸分別讀取 `static/car.py` 的 `SuspensionF` 與 `SuspensionR`，搭配 `Car.tf`、`Car.tr` 及各軸的負載輪胎半徑 `Car.load_Radius` 計算初始幾何。3D 座標中，X 為車身縱向、Y 向車輛左方、Z 向上；前軸平面位於 `X = 0`，後軸平面位於 `X = -Car.l`。每個 FV 平面原有的橫向 X 映射為 3D 的 `Y = -X_FV`，原高度 Y 映射為 `Z = Y_FV`。

中央 SV 平面位於車身中心線 `Y = 0`，與兩個 FV 平面垂直，顯示側視輪胎參考圈、主銷、Au／Al 構造線與切線，以及俯仰中心（PC）。幾何重用 `sv/sus_sv_v4.py` 的 `build_side_view_geometry(...)`，將既有 SV 的縱向座標整體鏡射為 `X = settings.wheel_center_x_sv - Car.l - x_SV`、高度映射為 `Z = y_SV`，使前後軸對齊 FV，並保留 caster、trail、IC 與 PC 的相對幾何。遠端 SV 瞬心的真實座標仍保留，畫面先聚焦車身附近，超出範圍的構造線僅裁切顯示。

FV 的四個輪胎改以封閉圓柱體顯示，圓柱軸沿 3D 的 Y 方向，半徑取自 `Car.free_radius`、寬度取自 `Car.tire_width`。輪胎中心高度為 `Car.free_radius`，底部位於 `Z = 0`，保留原輪胎外框的高度範圍；懸吊接點仍使用各軸的負載輪胎半徑計算。

中央 SV 平面的輪胎圓圈是側視投影參考，實體輪胎仍為左右兩側的四個圓柱體。

車身以八個 FV 車身側等效接點作為頂點：前／後、左／右各自的上 A 臂 `cpb_ru` 與下 A 臂 `cpb_rl`，共八點。這些等效點代表各 A 臂車身側兩個樞軸的平均位置；上、下臂點各自保留，不再互相平均成四點。八點連成前、後、左、右、上、下六個四邊面，構成封閉半透明六面體，取代原本固定尺寸的橘色矩形。頂點完全隨前後懸吊參數及軸距變化；可由 `figure.chassis_geometry` 讀取 `vertices`、`faces` 與 `edges`，面透明度由 `AnalysisSettings.chassis_face_alpha` 設定。

FV、SV 的其他元素均保留，另在 FL、FR、RL、RR 四個輪位合成 3D 主銷。每個主銷點在同一高度 `Z` 取 SV 轉換後的縱向 `X` 與 FV 的橫向 `Y = -X_FV`，同時保留 caster 與 KPI；左右輪各自建立。上 A 臂外端球接頭標為 `UBJ`，下 A 臂外端球接頭標為 `LBJ`，兩點均位於合成主銷軸線上。

`fv/kingpin_geometry.py` 的 `build_kingpin_geometry(...)` 回傳四個輪位的 `UBJ`、`LBJ`、主銷地面交點 `ground`、軸線頂端 `axis_top` 與對應 FV／SV 投影座標。建立圖形後，可由 `figure.kingpins` 取得這些靜態座標。

這個 3D 視圖呈現 FV、SV 投影參考及合成的 3D 主銷，僅計算初始靜態姿態，沒有滑桿、動態運動學求解或 CSV 匯出。

## 參數與共用工具

- `car.py`：`Car`、`SuspensionF`、`SuspensionR` 的車輛、輪胎與前後懸吊設計參數。
- `analysis_settings.py`：FV/SV 的靜態圖窗、座標範圍與參考線設定；FV 另有 3D 初始視角設定。
- `geometry.py`、`Geometric_Analysis.py`、`visualization.py`：幾何物件、懸吊幾何計算與繪圖工具。
- `fv/kingpin_geometry.py`：依同一高度的 FV／SV 座標合成四輪 3D 主銷與 `UBJ`／`LBJ`。
- `fv/chassis_geometry.py`：以八個 A 臂車身側等效接點建立六面體車身。
- `tv/sus_tv_v2.py`：檔案上方另有視圖參數。
- `we3D/sus_3Dwe_v3.py`：檔案上方另有靜態輪胎外形及視圖參數。
- `we3D/wheel_end_geometry.py`：3D 輪端共用的主銷軸與初始輪心計算。

修改 `static/car.py` 或對應視圖參數後，重新執行腳本即可更新圖形。
