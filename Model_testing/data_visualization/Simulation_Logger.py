import numpy as np
import pickle
from pathlib import Path

class SimulationLogger:


    def __init__(self):

        self.data = {}

        self.metadata = {}



    def add(self, name, value):
        
        if name not in self.data:
            self.data[name] = []

        if isinstance(value, np.ndarray):
            value = value.copy()

        self.data[name].append(value)



    def add_state(self, state, prefix=""):
        
        for key, value in vars(state).items():

            if key == "dt":
                continue

            name = f"{prefix}{key}"

            # 如果還是一個 class
            if hasattr(value, "__dict__"):

                self.add_state(
                    value,
                    prefix=name+"_"
                )

            else:

                self.add(name, value)



    def set_metadata(self, **kwargs):

        self.metadata.update(kwargs)



    def get(self, name):

        return np.asarray(
            self.data[name]
        )



    def result(self):

        return {
            key:np.asarray(value)
            for key,value in self.data.items()
        }



    def save(self, filename):

        package = {

            "data":self.result(),

            "metadata":self.metadata

        }


        filename = Path(filename)


        with open(filename,"wb") as f:

            pickle.dump(
                package,
                f
            )



    @staticmethod
    def load(filename):

        filename = Path(filename)


        with open(filename,"rb") as f:

            package = pickle.load(f)


        logger = SimulationLogger()

        logger.data = package["data"]

        logger.metadata = package["metadata"]


        return logger