from solution import SOLUTION
import constants as c
import copy

class HILLCLIMBER:
    def __init__(self) -> None:
        self.parent = SOLUTION()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()
        print(self.child.weights)
        print(self.parent.weights)

    def Select(self):
        print(self.parent.fitness, self.child.fitness)
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        print(self.parent.fitness, self.child.fitness)
        self.Select()

    def Show_Best(self):
        self.parent.Evaluate("GUI")

    def Evolve(self):
        self.parent.Evaluate("GUI")
        print(self.parent.fitness)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
        self.Show_Best()