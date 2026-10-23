from pathlib import Path
"""
這個py檔案在的地方就是data標記地點

路徑這樣指定 : "data/檔案名稱.csv"

"""

class DataPath:

    ROOT = Path(__file__).parent


    @staticmethod
    def resolve(path):

        path = Path(path)

        # 已經是絕對路徑
        if path.is_absolute():
            return path


        # 相對於 Data 資料夾
        return DataPath.ROOT / path