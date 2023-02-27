from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import pyrosim.pyrosim as pyrosim
import math
import pybullet as p
import constants as c
import numpy
import os

class ROBOT:

    def __init__(self, brainID):
        self.nn = NEURAL_NETWORK(f"brain{brainID}.nndf")
        self.motors = {}
        self.robotId = p.loadURDF("body" + str(brainID) + ".urdf")
        # self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        os.system(f"del brain{brainID}.nndf")
        self.brainID = brainID
        self.fitness = 0
        self.current = 0

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, i):
        for sensor in self.sensors:
            # print(self.sensors[sensor].Get_Value(i))
            self.sensors[sensor].Get_Value(i)
            # self.sensors[sensor] = math.sin(i)
            # self.sensors[sensor].Save_Values(i)

    
    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, i):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = bytes(self.nn.Get_Motor_Neurons_Joint(neuronName), encoding='utf8')
                desiredAngle = self.nn.Get_Value_Of(neuronName)*c.motorJointRange
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)
                # print(f'Motor Neuron: {neuronName}, Joint Name: {jointName}, Value: {desiredAngle}')
        # for motor in self.motors:
        #     self.motors[motor].Set_Value(i, self.robotId)
        #     self.motors[motor].Save_Values(i, self.robotId)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()


    def Get_Fitness(self):
        # stateofLinkZero = p.getLinkState(self.robotId,0)
        # positionofLinkZero = stateofLinkZero[0]
        # xCoordinateofLinkZero = positionofLinkZero[0]
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]
        f = open(f"tmp{self.brainID}.txt", "w")
        f.write(str(xPosition))
        f.close()
        os.rename("tmp"+str(self.brainID)+".txt" , "fitness"+str(self.brainID)+".txt")

        # os.system(f"rename tmp{self.brainID}.txt fitness{self.brainID}.txt")
