import numpy
import random

amplitude = numpy.pi/4
frequency = 5
phaseOffset = 0

amplitude_backleg = numpy.pi/4
frequency_backleg = 5
phaseOffset_backleg = numpy.pi/4

amplitude_frontleg = numpy.pi/4
frequency_frontleg = 5
phaseOffset_frontleg = 0

vectorSize = 1
maxForce = 1

targetAngles = numpy.linspace(-numpy.pi, numpy.pi, vectorSize)

numberOfGenerations = 1

populationSize = 1

linkcount = numpy.random.randint(10)

numSensorNeurons = linkcount + 2
numMotorNeurons = linkcount + 1

numoflinks = random.randint(7,12)


motorJointRange = .3