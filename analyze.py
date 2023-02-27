import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("data/backLegSensorValues.npy", mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', max_header_size=1000)
frontLegSensorValues = numpy.load("data/frontLegSensorValues.npy", mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', max_header_size=1000)
targetAnglesValues = numpy.load("data/targetAnglesValues.npy", mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', max_header_size=1000)
matplotlib.pyplot.plot(targetAnglesValues, numpy.sin(targetAnglesValues))
# matplotlib.pyplot.plot(backLegSensorValues, linewidth=1, label="Back Leg")
# matplotlib.pyplot.plot(frontLegSensorValues, linewidth=2, label="Front Leg")
# matplotlib.pyplot.legend(loc="upper right")
matplotlib.pyplot.show()
