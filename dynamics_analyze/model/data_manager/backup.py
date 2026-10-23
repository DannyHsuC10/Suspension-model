from pathlib import Path
import shutil
from datetime import datetime

from data_path import DataPath



class DataBackup:


    def __init__(
        self,
        source_dir,
        backup_dir="backup",
        max_backup=5
    ):

        self.source_dir = DataPath.resolve(
            source_dir
        )

        self.backup_dir = DataPath.resolve(
            backup_dir
        )

        self.max_backup = max_backup



    def create_backup(self):

        if not self.source_dir.exists():

            raise FileNotFoundError(
                f"{self.source_dir} not found"
            )


        # 建立backup資料夾
        self.backup_dir.mkdir(
            parents=True,
            exist_ok=True
        )


        timestamp = datetime.now().strftime(
            "%Y-%m-%d_%H-%M-%S"
        )


        target = (
            self.backup_dir /
            timestamp
        )


        target.mkdir(
            parents=True,
            exist_ok=False
        )


        # 複製資料
        for item in self.source_dir.iterdir():

            destination = target / item.name


            if item.is_dir():

                shutil.copytree(
                    item,
                    destination
                )

            else:

                shutil.copy2(
                    item,
                    destination
                )


        # 清理舊版本 開啟之後最多就只會保留5個檔案了
        # self.clean_backup()


        return target



    def clean_backup(self):

        if self.max_backup is None:
            return


        if not self.backup_dir.exists():
            return


        backups = [
            p for p in self.backup_dir.iterdir()
            if p.is_dir()
        ]


        # timestamp格式可以直接排序
        backups.sort()


        while len(backups) > self.max_backup:

            old_backup = backups.pop(0)

            shutil.rmtree(
                old_backup
            )

            print(
                f"Remove old backup: {old_backup.name}"
            )

# ===============================================
if __name__ == "__main__":

    backup = DataBackup(
        source_dir="data",
        backup_dir="backup",
        max_backup = 0# 設定要保留多少個備份檔案
    )

    backup.clean_backup()

    backup_path = backup.create_backup()


    print("Backup created:")
    print(backup_path)