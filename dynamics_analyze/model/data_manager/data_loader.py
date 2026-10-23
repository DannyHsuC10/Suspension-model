import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from data_path import DataPath


class LookupData:

    def __init__(
        self,
        x,
        y,
        name="data"
    ):

        self.x = np.asarray(x)
        self.y = np.asarray(y)

        self.name = name


    def __len__(self):

        return len(self.x)


    def size(self):

        return len(self.x)


    def plot(
        self,
        xlabel="X",
        ylabel="Y",
        title=None
    ):

        plt.figure()

        plt.plot(
            self.x,
            self.y,
            marker="o",
            color="blue"
        )

        plt.xlabel(xlabel)
        plt.ylabel(ylabel)

        if title:
            plt.title(title)
        else:
            plt.title(self.name)

        plt.grid(True)

        plt.show()



class DataLoader:



    @staticmethod
    def load_csv(
        path,
        x_col=0,
        y_col=1,
        name=None
    ):

        path = DataPath.resolve(path)


        if not path.exists():
            raise FileNotFoundError(
                f"Data file not found: {path}"
            )


        data = pd.read_csv(path)


        x = data.iloc[:,x_col].values
        y = data.iloc[:,y_col].values


        return LookupData(
            x,
            y,
            name=name or path.stem
        )

    @staticmethod
    def load_excel(
        path,
        x_col=0,
        y_col=1,
        sheet=0,
        name=None
    ):


        path = DataPath.resolve(path)


        if not path.exists():
            raise FileNotFoundError(
                f"Data file not found: {path}"
            )

        data = pd.read_excel(
            path,
            sheet_name=sheet
        )


        x = data.iloc[:,x_col].values
        y = data.iloc[:,y_col].values


        return LookupData(
            x,
            y,
            name=name or Path(path).stem
        )
