# import robot as ROBOT
import numpy
import constants as c
import pybullet as p
import pyrosim.pyrosim as pyrosim

class MOTOR:

    def __init__(self, jointName):
        self.jointName = jointName
        # self.Prepare_To_Act()

    # def Prepare_To_Act(self):
    #     self.amplitude = c.amplitude
    #     self.frequency = c.frequency
    #     self.offset = c.phaseOffset 
    #     self.targetAngles = numpy.linspace(-numpy.pi, numpy.pi, c.vectorSize)
    #     self.motorValues = self.amplitude * numpy.sin(self.frequency * self.targetAngles + self.offset)

    # def Set_Value(self, i, robotId):
    #     pyrosim.Set_Motor_For_Joint(
    #         bodyIndex = robotId,
    #         jointName = self.jointName,
    #         controlMode = p.POSITION_CONTROL,
    #         targetPosition = self.motorValues[i],
    #         maxForce = c.maxForce
    #     )
    def Set_Value(self,robotId,desiredAngle):
		    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = self.jointName,
        controlMode = p.POSITION_CONTROL,
        targetPosition = desiredAngle,
        maxForce = 200)

    # def Save_Values(self, i, robotId):
    #     numpy.save('data\motorvalues.npy', self.motorValues)