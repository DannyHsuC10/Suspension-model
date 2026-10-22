"""唯一批次啟動介面。修改下方設定後執行此檔案。"""

from batch_runner import main


# ===== 使用者設定 =====
# 工況清單由 batch_cases.py 的 FAMILIES 管理，可執行 --list 查看目前保留的配對。
# 輸出類型："all"（quarter + LR + FR）、"quarter"（單輪）、"LR"（左右雙輪）、"FR"（前後雙輪）
FAMILY = "all"

# 輸出內容："both"（PKL + GIF）或 "pkl"（只產生 PKL）
# GIF 需要 PKL 數據作為來源，因此沒有 GIF-only 選項。
OUTPUT = "both"

# 動畫輸出設定（可用 --fps / --dpi 覆寫）
FPS = 8
DPI = 70
# ======================


FAMILY_ALIASES = {
    "all": None,
    "quarter": ["quarter"],
    "LR": ["LR"],
    "FR": ["FR"],
}


if __name__ == "__main__":
    if FAMILY not in FAMILY_ALIASES:
        raise ValueError(f"FAMILY 請設定為以下其中之一：{', '.join(FAMILY_ALIASES)}")
    if OUTPUT not in {"both", "pkl"}:
        raise ValueError('OUTPUT 請設定為 "both" 或 "pkl"')

    import sys

    if OUTPUT == "pkl" and "--no-gif" not in sys.argv:
        sys.argv.append("--no-gif")
    raise SystemExit(main(FAMILY_ALIASES[FAMILY], default_fps=FPS, default_dpi=DPI))
