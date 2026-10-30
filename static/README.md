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
| `fv/sus_fv_v5.py` | 在同一張 3D 靜態圖顯示前、後 FV 平面、中央 SV 平面、四個 3D 輪胎圓柱及輪胎與輪框的 FV 矩形投影、3D 主銷與球接頭、半透明六面體車身、八個 A 臂三角參考面、輪框干涉邊界圓、前輪選定交點、後輪八個極限干涉交點及前後輪上、下 A 臂延伸至車體的接點。 |
| `sv/sus_sv_v4.py` | 前後軸初始側視幾何與瞬心（IC），輪心縱向間距為軸距。 |
| `tv/sus_tv_v2.py` | 初始轉向幾何。 |
| `we3D/sus_3Dwe_v3.py` | 初始輪胎、主銷軸、輪心、輪軸、輪胎底部參考點及其對主銷的靜態力臂。 |

FV 使用唯一的固定設計瞬心 `IC`，座標由輪距、FVSA（`fvsa`）與設計 RC 高度（`h_RC`）決定，不再另外求解或更新瞬心。SV 的 IC 仍由側視靜態幾何計算。

FV 的前、後軸分別讀取 `static/car.py` 的 `SuspensionF` 與 `SuspensionR`，搭配 `Car.tf`、`Car.tr` 及各軸的負載輪胎半徑 `Car.load_Radius` 計算初始幾何。3D 座標中，X 為車身縱向、Y 向車輛左方、Z 向上；前軸平面位於 `X = 0`，後軸平面位於 `X = -Car.l`。每個 FV 平面原有的橫向 X 映射為 3D 的 `Y = -X_FV`，原高度 Y 映射為 `Z = Y_FV`。

合成 3D 視圖依幾何所屬群組統一配色：前 FV 與前輪使用藍色 `#0072b2`，後 FV 與後輪使用橘色 `#d55e00`，中央 SV 使用綠色 `#009e73`。各群組的輪胎／輪框投影、主銷、A 臂、干涉圓、交點、車體接點與對應標示沿用同一色系；3D 輪胎圓柱、車身及跨前後軸的共用參考使用灰色。圖例配合三組顏色，並以線型、標記形狀及文字區分幾何類型。

中央 SV 平面位於車身中心線 `Y = 0`，與兩個 FV 平面垂直，顯示側視輪胎參考圈、主銷、Au／Al 構造線與切線，以及俯仰中心（PC）。幾何重用 `sv/sus_sv_v4.py` 的 `build_side_view_geometry(...)`，將既有 SV 的縱向座標整體鏡射為 `X = settings.wheel_center_x_sv - Car.l - x_SV`、高度映射為 `Z = y_SV`，使前後軸對齊 FV，並保留 caster、trail、IC 與 PC 的相對幾何。SV 瞬心的真實座標保留，三角平面的整體視圖會納入這些遠端 IC；車體局部預覽只裁切顯示範圍。

四輪同時保留灰色半透明的封閉 3D 輪胎圓柱與前視矩形正投影。圓柱軸沿車輛 Y 方向，半徑取自 `Car.free_radius`、寬度取自 `Car.tire_width`，以胎面與兩側封閉側壁呈現。矩形位於各自的 FV 平面，前輪全部頂點的 `X = 0`，後輪為 `X = -Car.l`。輪胎矩形以實線繪製，寬度取自 `Car.tire_width`（目前 190.5 mm）、高度為 `2 * Car.free_radius`（411.48 mm）；圓柱、輪胎與輪框矩形及原始干涉圓共用各軸的負載輪心高度：前輪 `Z = Car.load_Radius[0]`，後輪 `Z = Car.load_Radius[1]`，與 FV 輪心虛線及 SV 輪心一致。自由半徑外形的最低點為 `Z = Car.load_Radius[axle] - Car.free_radius`，因此部分輪胎會位於地面下方，不裁切或壓平外形；接地參考點仍在 `Z = 0`。

輪框以同中心的虛線矩形顯示內孔投影，高度取自 `Car.rim_inner_diameter`（目前為 `9.625 * 25.4 = 244.475 mm`），上、下邊各距負載輪心半個內徑。這是已知內徑的輪廓，沒有推定輪框外徑。寬度優先使用投影建構函式的 `rim_width` 參數，其次使用 `Car.rim_width`；兩者皆未指定時，暫以輪胎寬度 190.5 mm 作示意，並在投影資料中記錄寬度來源，這個預設值不代表實測輪框寬度。

可由 `figure.wheel_projection_geometry`、`figure.wheel_projection_artists` 讀取四輪矩形座標與圖上物件，各 FV 平面也保留對應的 `wheel_projection_geometry`、`wheel_projection_artists`。`figure.tire_artists` 與各 FV 平面的 `tire_artists` 按輪位保存 3D 輪胎的 `tread`、`sidewalls` 與 `outlines`。矩形投影與 3D 輪胎同時顯示，輪框干涉邊界圓、交點及 A 臂車體接點沿用既有 3D 幾何。

輪框干涉邊界圓位於各輪胎靠近車身的內側平面。`Car.gap = 20 / 1000`，長度單位為 m；`Car.steer = 36.0`，角度單位為 deg。邊界圓直徑為 `Car.rim_inner_diameter - 2 * Car.gap`，目前為 `0.204475 m`（204.475 mm），半徑為 102.2375 mm。四輪各保留一個原始圓，前輪各另畫 `+steer` 與 `-steer` 的兩個圓，共八個邊界圓。

前輪的旋轉以各自合成 3D 主銷的 `ground` 至 `axis_top` 為向上軸線，用含軸位置平移的 Rodrigues 旋轉矩陣計算。正、負角度依該向上主銷軸的右手定則；`steer` 表示繞主銷的旋轉角。各角度的圓以固定參考疊圖顯示，座標可由 `figure.interference_geometry` 取得。

中央 SV 平面的輪胎圓圈保留為側視投影參考，各輪的前視輪廓顯示於前、後 FV 平面，並與左右兩側的四個 3D 輪胎圓柱一同呈現。

車身以八個 FV 車身側等效接點作為頂點：前／後、左／右各自的上 A 臂 `cpb_ru` 與下 A 臂 `cpb_rl`，共八點。這些等效點代表各 A 臂車身側兩個樞軸的平均位置；上、下臂點各自保留，不再互相平均成四點。八點連成前、後、左、右、上、下六個四邊面，構成封閉半透明六面體，取代原本固定尺寸的橘色矩形。頂點完全隨前後懸吊參數及軸距變化；可由 `figure.chassis_geometry` 讀取 `vertices`、`faces` 與 `edges`，面透明度由 `AnalysisSettings.chassis_face_alpha` 設定。

FV、SV 的其他元素均保留，另在 FL、FR、RL、RR 四個輪位合成 3D 主銷。每個主銷點在同一高度 `Z` 取 SV 轉換後的縱向 `X` 與 FV 的橫向 `Y = -X_FV`，同時保留 caster 與 KPI；左右輪各自建立。上 A 臂外端球接頭標為 `UBJ`，下 A 臂外端球接頭標為 `LBJ`，兩點均位於合成主銷軸線上。

`fv/kingpin_geometry.py` 的 `build_kingpin_geometry(...)` 回傳四個輪位的 `UBJ`、`LBJ`、主銷地面交點 `ground`、軸線頂端 `axis_top` 與對應 FV／SV 投影座標。建立圖形後，可由 `figure.kingpins` 取得這些靜態座標。

四個輪位的上、下 A 臂各有一個三角參考面，共八面。每面的三個頂點為同輪位的 FV 瞬心、同軸的 SV 瞬心，以及主銷上的接點；上臂使用 `UBJ`，下臂使用 `LBJ`，依前、後軸統一配色，並以標記形狀、線型與文字辨識上、下臂。SV 瞬心沿用中央 `Y = 0` 平面的真實座標。這些面是指定三點構成的靜態幾何參考面；八面的座標可由 `figure.arm_planes` 取得，透明度由 `AnalysisSettings.arm_plane_alpha` 設定。

`fv/sus_fv_v5.py` 檔案上方的 `SHOW_REFERENCE_PLANES` 統一控制八個 A 臂、前 FV、後 FV 與中央 SV 的透明填面，共十一個面。預設為 `False`；改成 `True` 並重新執行腳本即可顯示：

```python
SHOW_REFERENCE_PLANES = False  # 關閉透明填面；True 顯示透明填面
```

此開關只影響透明填面。線條、平面框線、瞬心標記、幾何計算、3D 輪胎、FV 矩形投影及車身均保留。

同一檔案上方另有三個獨立的投影視圖開關，預設皆為 `True`。將個別變數改為 `False`，再重新執行腳本，即可隱藏該視圖：

```python
SHOW_SV = True        # 中央 SV 投影視圖
SHOW_FRONT_FV = True  # 前輪 FV 投影視圖
SHOW_REAR_FV = True   # 後輪 FV 投影視圖
```

這三個開關分別控制各視圖原有的懸吊與構造線、投影點、平面框線、視圖名稱、RC／PC 標記及對應的 FV／SV 瞬心標示；前、後 FV 開關也各自控制該軸的輪胎與輪框矩形投影。關閉視圖時，其透明填面亦隱藏，即使 `SHOW_REFERENCE_PLANES = True` 也不顯示該視圖的填面；八個 A 臂透明填面則仍由 `SHOW_REFERENCE_PLANES` 控制。

投影視圖的顯示開關不影響完整幾何計算。四個實體 3D 輪胎、3D 主銷、A 臂、干涉圓及交點、車體接點與共用車身、CG、前後 RC 連線均持續顯示；下述前輪下臂交點檢視模式則會另行暫停該下臂的自動選點標記與構造端點顯示。

前輪 `+steer`、`-steer` 的兩個邊界圓，分別與該輪上、下 A 臂的延伸幾何平面解析求交。每個圓與臂平面最多產生兩個離散交點，兩圈合計最多四個候選點。選點以轉向姿態分開處理：在 `+steer` 圓的交點中取 `abs(Y)` 最小的一點，再在 `-steer` 圓的交點中取 `abs(Y)` 最小的一點；`abs(Y)` 即該點到車輛 XZ 平面（`Y = 0`）的距離。前輪上、下 A 臂均使用此規則，不將兩圈交點混在一起取最近兩點。

求交使用三角形所定義的無限延伸平面，不受三角形邊界或預覽裁切影響。無交點、相切或共面情況會保留其狀態；某個姿態沒有離散交點時，不拿另一圈的第二個點補足，因此每臂可能少於兩個選點。這些狀態描述圓與平面的幾何關係，不是實體碰撞判定。可由 `figure.circle_plane_intersections` 讀取各前輪上／下臂的 `by_pose` 求交狀態及交點、`candidates` 唯一候選點，以及 `selected` 各有效姿態的最近點與來源角度。`selected` 按 `positive`、`negative` 順序儲存；`steer = 0` 時，即使兩姿態選到相同位置，仍分別保留兩筆姿態選取資料。

預設關閉前輪下 A 臂交點檢視模式，主圖依下述靜態選點與延伸規則繪製四輪上、下 A 臂，共八組、十六支臂腿：

```python
INSPECT_FRONT_LOWER_INTERSECTIONS = False
```

將此變數改為 `True` 可切換到交點檢視模式。開啟後，直接執行 `sus_fv_v5.py` 會同時開啟主圖與左右前輪下臂特寫圖。特寫固定顯示局部下臂平面、兩個轉向圓及兩平面的截線，方便直接比對原始交點。

每個前輪獨立編號：`+steer` 圓固定使用 `a`、`b`，`-steer` 圓固定使用 `c`、`d`；各圓內按車輛 X 由大至小排序，相同 X 時按 Z 由大至小。字母前加 `FL`／`FR` 輪位前綴。標記直接來自 `by_pose` 的解析交點，不經最近點篩選或跨姿態去重；某圓沒有交點時保留對應字母空缺，相切時只標該姿態的第一個字母。交點數量隨輪心高度與懸吊參數變化，不補造缺失點。

模式開啟時，主圖不畫原本前輪下臂的 `L1`／`L2` 自選標記、自動延伸腿與內接點，其他輪位及上臂展示保留。將變數改為 `False` 並重新執行，可恢復下臂的自動選點與延伸展示；既有 `selected` 與 `lower_arm_mount_geometry` 資料始終保留。可由 `figure.front_lower_intersection_candidates`、`figure.front_lower_intersection_candidate_artists` 取得原始字母標記資料與圖上物件。另可呼叫 `front_lower_inspection_plot.py` 的 `create_front_lower_inspection_figure(arm_planes, interference_geometry, candidates)`，分開查看左右前輪下臂的局部平面、兩個圓、全部交點及各姿態求交狀態。

後輪使用未旋轉的中立干涉邊界圓，分別與左右後輪的上、下 A 臂延伸平面求交。每個臂平面目前各有前、後兩點，因此上／下 × 左／右 × 前／後共標記八個極限干涉交點。按車輛縱向 X 由大至小排序，上臂標為 `UF`（前）／`UR`（後），下臂標為 `LF`（前）／`LR`（後），並加輪位前綴 `RL` 或 `RR`。求交同樣使用無限延伸平面，座標不受顯示裁切影響；可由 `figure.rear_circle_plane_intersections` 取得各臂的狀態、交點及標記資料，由 `figure.rear_circle_plane_intersection_artists` 取得圖上標記。

前輪上 A 臂由主銷上的 `UBJ` 分別經過選定的 `U1`、`U2` 延伸，直到車輛座標系的 Y 等於該側 FV 上 A 臂車身接點的投影位置：`Y_target = -cpb_ru.x`。目前左前為 `+0.255 m`、右前為 `-0.255 m`。每支連線使用 `P = UBJ + t * (U - UBJ)`，其中 `t = (Y_target - UBJ_Y) / (U_Y - UBJ_Y)`，終點的 X、Z 由同一線方程決定；兩個轉向姿態都有選點且能求得延伸終點時，每輪得到兩個車體端點。

兩端點按車身縱向 X 命名：X 較大的是上臂前側內接點 `UFI`，較小的是後側內接點 `URI`。可由 `figure.upper_arm_mount_geometry` 讀取各輪的 `target_y`、`legs` 與 `mounts`。平行、退化或尚未經過選定點的情況會保留狀態，不補造延伸端點。這些接點依各側車身接點的 Y 座標條件構造，既有 `cpb_ru`／`cpb_rl` 與六面體車身仍保留。

一般顯示模式下，前輪下 A 臂同樣由主銷上的 `LBJ` 分別經過已選定的 `L1`、`L2` 延伸，終止於該側 FV 下 A 臂車身接點的車輛 Y 座標：`Y_target = -cpb_rl.x`，目前左前為 `+0.210 m`、右前為 `-0.210 m`。使用 `P = LBJ + t * (L - LBJ)`，其中 `t = (Y_target - LBJ_Y) / (L_Y - LBJ_Y)`；X、Z 由延伸線自然決定。終點按 X 由大至小命名為下臂前側內接點 `LFI`、後側內接點 `LRI`，可由 `figure.lower_arm_mount_geometry` 讀取 `LBJ`、`target_y`、`FV_mount`、`legs` 與 `mounts`；開啟交點檢視模式時才暫停繪製這些自動構造端點。

左右後輪的上、下 A 臂也採用相同的車輛 Y 終止條件：上臂從 `UBJ` 經該輪的 `UF`／`UR` 干涉交點，延伸到 `Y_target = -cpb_ru.x`（目前左後 `+0.250 m`、右後 `-0.250 m`）；下臂從 `LBJ` 經 `LF`／`LR`，延伸到 `Y_target = -cpb_rl.x`（左後 `+0.205 m`、右後 `-0.205 m`）。每支線使用 `P = J + t * (Q - J)` 與 `t = (Y_target - J_Y) / (Q_Y - J_Y)`，其中 `J` 為主銷接點、`Q` 為干涉交點；終點的 X、Z 保留線方程所得的值。

後輪共構造八個車體接點，各輪上臂按 X 由大至小標為 `UFI`／`URI`，下臂為 `LFI`／`LRI`，並加上 `RL`／`RR` 輪位前綴。可由 `figure.rear_upper_arm_mount_geometry`、`figure.rear_lower_arm_mount_geometry` 讀取各側的主銷接點、`target_y`、`FV_mount`、`legs` 與 `mounts`，對應標記由 `figure.rear_upper_arm_mount_artists`、`figure.rear_lower_arm_mount_artists` 取得。顯示範圍納入前後輪全部構造端點；原始 FV 車身側等效點及六面體車身仍保留。

預設整體視角納入八個三角形的完整頂點。呼叫 `create_figure(fit_arm_planes=False)` 可近看車體，僅裁切顯示用面片與邊線，原始瞬心及三角形座標保持不變。

這個 3D 視圖呈現 FV、SV 投影參考、合成的 3D 主銷及指定轉向角度的干涉邊界圓，沒有滑桿、動態運動學求解或 CSV 匯出。

## 參數與共用工具

- `car.py`：`Car`、`SuspensionF`、`SuspensionR` 的車輛、輪胎與前後懸吊設計參數。
- `analysis_settings.py`：FV/SV 的靜態圖窗、座標範圍與參考線設定；FV 另有 3D 初始視角設定。
- `geometry.py`、`Geometric_Analysis.py`、`visualization.py`：幾何物件、懸吊幾何計算與繪圖工具。
- `fv/kingpin_geometry.py`：依同一高度的 FV／SV 座標合成四輪 3D 主銷與 `UBJ`／`LBJ`。
- `fv/chassis_geometry.py`：以八個 A 臂車身側等效接點建立六面體車身。
- `fv/arm_plane_geometry.py`：以 FV 瞬心、SV 瞬心及主銷接點建立八個 A 臂三角參考面。
- `fv/arm_plane_plot.py`：繪製上／下臂參考面與瞬心標示，並處理局部視圖的顯示裁切。
- `fv/wheel_projection_geometry.py`：將四輪輪胎與輪框內孔建立為各軸 FV 平面的矩形投影，保留尺寸及輪框寬度來源。
- `fv/interference_geometry.py`：建立輪框內側干涉邊界圓與前輪繞實際主銷旋轉後的參考圓。
- `fv/circle_plane_intersections.py`：解析計算圓與 A 臂延伸平面的交點；前輪從正、負轉向圓各自選取一個 `abs(Y)` 最小的交點，後輪中立圓的交點按 X 排序並標為前、後極限干涉交點。
- `fv/front_lower_inspection_plot.py`：以左右前輪下臂特寫顯示原始 `a`／`b`／`c`／`d` 交點、轉向圓及求交狀態，供人工比較。
- `fv/arm_mount_geometry.py`：共用的 A 臂射線與固定車輛 Y 平面求交、延伸狀態及接點排序。
- `fv/upper_arm_mount_geometry.py`：將前輪上 A 臂由 `UBJ` 經選定交點延伸至該側的車輛 Y 座標 `-cpb_ru.x`，建立 `UFI`／`URI` 車體接點。
- `fv/lower_arm_mount_geometry.py`：將前輪下 A 臂由 `LBJ` 經選定交點延伸至該側的車輛 Y 座標 `-cpb_rl.x`，建立 `LFI`／`LRI` 車體接點。
- `fv/rear_arm_mount_geometry.py`：將左右後輪的上、下 A 臂由 `UBJ`／`LBJ` 經干涉交點延伸至各自 FV 車身接點的車輛 Y 座標，建立八個後輪車體接點。
- `tv/sus_tv_v2.py`：檔案上方另有視圖參數。
- `we3D/sus_3Dwe_v3.py`：檔案上方另有靜態輪胎外形及視圖參數。
- `we3D/wheel_end_geometry.py`：3D 輪端共用的主銷軸與初始輪心計算。

修改 `static/car.py` 或對應視圖參數後，重新執行腳本即可更新圖形。
