# 3D 輪端靜態視圖

從專案根目錄執行：

```powershell
python -m static.we3D.sus_3Dwe_v3
```

也可直接執行 `python static/we3D/sus_3Dwe_v3.py`。需要 `numpy` 與 `matplotlib`。

`sus_3Dwe_v3.py` 以單一 3D 圖顯示初始輪胎、主銷軸、輪心、輪軸、輪胎底部參考點及其對主銷的靜態力臂，並顯示固定姿態的幾何讀值。

座標與長度計算使用 m，圖中文字的輪心座標與力臂讀值使用 mm。輪胎底部參考點為初始輪胎最低點的幾何參考。

車輛與懸吊設計參數來自 `static/car.py` 的 `Car`、`SuspensionF`。修改參數後重新執行即可更新靜態圖形。

`wheel_end_geometry.py` 提供 `kingpin_axis(...)` 與 `initial_wheel_center(...)`。主銷穿過固定原點，初始輪心偏移依 KPI、caster、mechanical trail 與 scrub radius 計算。
