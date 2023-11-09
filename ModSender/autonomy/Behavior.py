from abc import abstractmethod


class Behavior:
    def __init__(self):
        pass

    @abstractmethod
    def begin(self):
        pass

    @abstractmethod
    def execute(self, feedback):
        pass