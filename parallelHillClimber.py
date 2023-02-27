from solution import SOLUTION
import constants as c
import copy
import os

class PARALLELHILLCLIMBER:
    def __init__(self) -> None:
        self.parents = {}
        self.nextAvailableID = 0
        self.fitnessPlot = open("fitnessPlot.txt", "w")
        self.generationCount = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        os.system("del body*.urdf")


    def Spawn(self):
        self.children = {}
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()

    def Select(self):
        # print(self.parents.fitness, self.children.fitness)
        for i in self.parents:
            if self.parents[i].fitness > self.children[i].fitness:
                self.parents[i] = self.children[i]

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
        self.Write_Fitness()
        self.generationCount += 1

    def Write_Fitness(self):
        fittest_parent = None
        fittest_score = None
        for parent in self.parents.keys():
            if fittest_parent is None:
                fittest_parent = self.parents[parent]
                fittest_score = self.parents[parent].fitness
            if self.parents[parent].fitness < fittest_score:
                fittest_parent = self.parents[parent]
                fittest_score = self.parents[parent].fitness
        self.fitnessPlot.write(f"{str(self.generationCount)}, {str(fittest_score * -1)} \n")

    def Evaluate(self, results):
        for i in results:
            results[i].Start_Simulation("DIRECT")
        for i in results:
            results[i].Wait_For_Simulation_To_End()

    def Show_Best(self):
        fittest_parent = None
        fittest_score = None
        for parent in self.parents.keys():
            if fittest_parent is None:
                fittest_parent = self.parents[parent]
                fittest_score = self.parents[parent].fitness
            if self.parents[parent].fitness < fittest_score:
                fittest_parent = self.parents[parent]
                fittest_score = self.parents[parent].fitness
        fittest_parent.Start_Simulation('GUI')

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
        self.fitnessPlot.close()

    def Print(self):
        print("\n")
        for i in self.parents:
            print(f"Parent: {self.parents[i].fitness} Child: {self.children[i].fitness}")
        print("\n")
