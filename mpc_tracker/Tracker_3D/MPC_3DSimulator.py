import inspect
from dataclasses import dataclass, field

global m
@dataclass
class MPC_3DSimulator:
    age: int
    parameters: dict = field()

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}
        self.age = eval("s", self.parameters) + 1

if __name__ == "__main__":
    # m = 1
    test = MPC_3DSimulator(10)
    print(test.age)