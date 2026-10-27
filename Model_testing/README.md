# Batch results viewer

在專案根目錄執行：

```powershell
.\Suspension_sys\.venv\Scripts\python.exe Model_testing/batch_viewer.py
```

上方使用專案現有的虛擬環境。若自己的 Python 已有下列套件，也可以在
`Model_testing` 內執行 `python batch_viewer.py`。預設讀取程式旁的
`batch_results`，不會重新執行模擬。

- 選擇 **Family → Scenario → Model** 切換案例。
- **Signals**：選擇一個訊號，勾選 **Compare models** 疊圖比較同 family、同 scenario
  的模型。下方顯示訊號最小／最大值，工具列可縮放、平移及儲存圖片。
- **Animation**：播放該案例已有的 `response.gif`；沒有 GIF 仍能看數據。
- **Metadata**：查看該案例儲存的模擬參數與來源資訊。
- **Refresh**：重新讀取結果；**Open folder**：切換其他批次結果目錄。

指定其他結果目錄：

```powershell
python Model_testing/batch_viewer.py --results "D:\path\to\batch_results"
```

使用既有的 NumPy、Matplotlib、Pillow，以及 Python 內建的 Tkinter。
若環境缺少繪圖套件：

```powershell
python -m pip install numpy matplotlib pillow
```

PKL 以現有 `SimulationLogger` 格式讀取，請只開啟自己產生或可信任的結果。
曲線使用 PKL 內的原始數值，不額外換算或推導 roll/pitch。
