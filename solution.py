import random
import numpy
import pyrosim.pyrosim
import pyrosim.pyrosim as pyrosim
import os
import time
import constants as c

class SOLUTION:
    def __init__(self, myID):
        self.weights = (numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)*2)-1
        self.myID = myID
        random.seed(self.myID)
        self.created = False
        self.fitness = 0
        self.sensors = []
        self.motors = []
        self.links = []
        self.counter = 0

    def Evaluate(self, directOrGUI):
        self.Create_Body()
        self.Create_Brain()
        os.system("start /B python3 simulate.py " + directOrGUI + " " + str(self.myID))
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.05)
        fitnessFile = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()

    def Start_Simulation(self, directOrGUI):
        self.Create_Body()
        while not os.path.exists("body" + str(self.myID) + ".urdf"):
            time.sleep(0.05)
        self.Create_Brain()
        os.system("start /B python3 simulate.py " + directOrGUI + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.05)
        time.sleep(0.05)
        fitnessFile = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.system("del " + "fitness" + str(self.myID) + ".txt")

    def Create_Body(self):
        def randValue(length):
            length = length/2
            randomNum = ((random.random()*2)-1)
            return length * randomNum
        def getjointPos(parent, goal, size):
            x = size[0]
            y = size[1]
            z = size[2]
            randX = randValue(x)
            randY = randValue(y)
            randZ = randValue(z)
            randArray = [randX, randY, randZ]
            match parent:
                case 0: randArray[0] -= x/2
                case 1: randArray[0] += x/2
                case 2: randArray[1] -= y/2
                case 3: randArray[1] += y/2
                case 4: randArray[2] -= z/2
                case 5: randArray[2] += z/2
            match goal:
                case 0: randArray[0] += x/2 - randX
                case 1: randArray[0] += -x/2 - randX
                case 2: randArray[1] += y/2 - randY
                case 3: randArray[1] += -y/2 - randY
                case 4: randArray[2] += z/2 - randZ
                case 5: randArray[2] += -z/2 - randZ
            return randArray

        def findDir(parent_direction):
            position = []
            match parent_direction:
                case 0: position = [-link[0]/2.0, 0, 0]
                case 1: position = [link[0]/2.0, 0, 0]
                case 2: position = [0, -link[1]/2.0, 0]
                case 3: position = [0, link[1]/2.0, 0]
                case 4: position = [0, 0, -link[2]/2.0]
                case 5: position = [0, 0, link[2]/2.0]
            return position

        def getjointPosForRoot(i, x , y, z):
            jointPos = []
            if i == 0: jointPos = [x/2, randValue(y), randValue(z) + (self.findZ/2)]
            elif i == 1: jointPos = [-x/2, randValue(y), randValue(z) + (self.findZ/2)]
            elif i == 2: jointPos = [randValue(x), y/2, randValue(z) + (self.findZ/2)]
            elif i == 3: jointPos = [randValue(x), -y/2, randValue(z) + (self.findZ/2)]
            elif i == 4: jointPos = [randValue(x), randValue(y), (z/2) + (self.findZ/2)]
            elif i == 5: jointPos = [randValue(x), randValue(y), (-z/2) + (self.findZ/2)]
            return jointPos
        if not self.created:
            self.created = True
            self.sensors = []
            self.motors = []
            pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
            self.areaLink = {}
            self.sideGraph = {}
            self.edge_graph = {}
            self.parentDict = {}
            self.labels = {}
            self.joint_parent_child = {}
            self.joint_positions = {}
            self.joint_axis = {}
            self.link_sensor_check = {}
            self.link_positions = {}
            self.link_sizes = {}
            #generate sizes
            for i in range(c.numoflinks):
                rand_x = random.random()
                rand_y = random.random()
                rand_z = random.random()
                area = rand_x*rand_y*rand_z
                self.areaLink[(rand_x, rand_y, rand_z)] = area
                self.sideGraph[(rand_x, rand_y, rand_z)] = [None, None, None, None, None, None] #+x -x +y -y +z -z
                self.edge_graph[(rand_x, rand_y, rand_z)] = []
            self.areaSort = sorted(self.areaLink, key=self.areaLink.get, reverse=True)
            label = 0
            for s in self.areaSort:
                self.labels[s] = label
                label += 1
            for i, link in enumerate(self.areaSort):
                if i == 0:
                    continue
                larger_blocks = self.areaSort[:i]
                while True:
                    choose_block = random.choice(larger_blocks)
                    if len(self.edge_graph[choose_block]) < 6:
                        self.edge_graph[choose_block].append(link)
                        break
                    larger_blocks.remove(choose_block)
            self.findZ = (len(self.areaSort)*3/2)
            for i, link in enumerate(self.areaSort):
                if i == 0:
                    self.parentDict[link] = None
                connectLinks = self.edge_graph[link]
                available_indices = [i for i, x in enumerate(self.sideGraph[link]) if x is None]
                for connectLink in connectLinks:
                    chosen_side = random.choice(available_indices)
                    self.sideGraph[link][chosen_side] = connectLink
                    if chosen_side % 2 == 0:
                        self.sideGraph[connectLink][chosen_side+1] = []
                        self.parentDict[connectLink] = chosen_side+1
                    else:
                        self.sideGraph[connectLink][chosen_side-1] = []
                        self.parentDict[connectLink] = chosen_side-1
                    available_indices.remove(chosen_side)

            for link_i, link in enumerate(self.areaSort):
                if link in self.parentDict:
                    parent_direction = self.parentDict[link]
                else:
                    parent_direction = None
                if link_i == 0:
                    sensorCheck = random.random()
                    if sensorCheck > 0.5:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=[0, 0, self.findZ/2.0], size=list(link), material_name="Green", rgba="0 1 0 1")
                        self.sensors.append(self.labels[link])
                        self.link_sensor_check[self.labels[link]] = True
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = [0, 0, self.findZ/2.0]
                    else:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=[0, 0, self.findZ/2.0], size=list(link))
                        self.link_sensor_check[self.labels[link]] = False
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = [0, 0, self.findZ/2.0]
                else:
                    sensorCheck = random.random()
                    position = findDir(parent_direction)
                    if sensorCheck > 0.5:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=position, size=list(link), material_name="Green", rgba="0 1 0 1")
                        self.sensors.append(self.labels[link])
                        self.link_sensor_check[self.labels[link]] = True
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = position
                    else:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=position,
                                          size=list(link))
                        self.link_sensor_check[self.labels[link]] = False
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = position
                connectLinks = self.sideGraph[link]
                for i, connectLink in enumerate(connectLinks):
                    if connectLink == None or connectLink == []:
                        continue
                    jointName = "Link" + str(self.labels[link]) + "_Link" + str(self.labels[connectLink])
                    randomjoint = random.choice(["0 1 0", "1 0 0", "0 0 1"])
                    jointPos = []
                    x = link[0]
                    y = link[1]
                    z = link[2]
                    if self.labels[link] == 0:
                        jointPos = getjointPosForRoot(i, x, y, z)
                    else:
                        jointPos = getjointPos(parent_direction, i, link)
                    pyrosim.Send_Joint(name=jointName, parent="Link" + str(self.labels[link]),child="Link" + str(self.labels[connectLink]),type="revolute",position=jointPos,jointAxis=randomjoint)
                    self.motors.append(jointName)
                    self.joint_parent_child[jointName] = {'parent':"Link" + str(self.labels[link]),'child':"Link" + str(self.labels[connectLink])}
                    self.joint_positions[jointName] = jointPos
                    self.joint_axis[jointName] = randomjoint
            pyrosim.End()
            
    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        j = 0
        self.numMotors = 0
        self.numSensors = 0
        for i in self.motors:
            pyrosim.Send_Motor_Neuron(name=j, jointName=i)
            self.numMotors += 1
            j += 1
        for i in self.sensors:
            pyrosim.Send_Sensor_Neuron(name=j, linkName="Link"+str(i))
            self.numSensors += 1
            j += 1
        self.weights = (numpy.random.rand(self.numSensors, self.numMotors) * 2) - 1
        for currentRow in range(self.numSensors):
            for currentColumn in range(self.numMotors):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+self.numSensors, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        def randValue(length):
            length = length/2
            randomNum = ((random.random()*2)-1)
            return length * randomNum

        def getjointPos(parent, goal, length):
            x = length[0]
            y = length[1]
            z = length[2]
            randX = randValue(x)
            randY = randValue(y)
            randZ = randValue(z)
            randArray = [randX, randY, randZ]
            match parent:
                case 0: randArray[0] -= x/2
                case 1: randArray[0] += x/2
                case 2: randArray[1] -= y/2
                case 3: randArray[1] += y/2
                case 4: randArray[2] -= z/2
                case 5: randArray[2] += z/2
            match goal:
                case 0: randArray[0] += x/2 - randX
                case 1: randArray[0] += -x/2 - randX
                case 2: randArray[1] += y/2 - randY
                case 3: randArray[1] += -y/2 - randY
                case 4: randArray[2] += z/2 - randZ
                case 5: randArray[2] += -z/2 - randZ
            return randArray

        def findDir(parent_direction, link):
            position = []
            match parent_direction:
                case 0: position = [-link[0]/2.0, 0, 0]
                case 1: position = [link[0]/2.0, 0, 0]
                case 2: position = [0, -link[1]/2.0, 0]
                case 3: position = [0, link[1]/2.0, 0]
                case 4: position = [0, 0, -link[2]/2.0]
                case 5: position = [0, 0, link[2]/2.0]
            return position

        def getjointPosForRoot(i, x , y, z):
            jointPos = []
            if i == 0:jointPos = [x/2, randValue(y), randValue(z) + (self.findZ/2)]
            elif i == 1:jointPos = [-x/2, randValue(y), randValue(z) + (self.findZ/2)]
            elif i == 2:jointPos = [randValue(x), y/2, randValue(z) + (self.findZ/2)]
            elif i == 3:jointPos = [randValue(x), -y/2, randValue(z) + (self.findZ/2)]
            elif i == 4:jointPos = [randValue(x), randValue(y), (z/2) + (self.findZ/2)]
            elif i == 5:jointPos = [randValue(x), randValue(y), (-z/2) + (self.findZ/2)]
            return jointPos
        evolve_option = random.randint(3, 4)
        if evolve_option == 3:
            os.system("del body" + str(self.myID) + ".urdf")
            addorsubtract = random.random()
            if addorsubtract < 0.5 and len(self.areaSort) > 2:
                link_to_remove = random.choice(list(self.areaSort))
                self.areaSort.remove(link_to_remove)
                self.areaLink.pop(link_to_remove)
                self.sideGraph.pop(link_to_remove)
                self.edge_graph.pop(link_to_remove)
            else:
                rand_x = random.random()
                rand_y = random.random()
                rand_z = random.random()
                area = rand_x * rand_y * rand_z

                new_link = (rand_x, rand_y, rand_z)
                self.areaLink[new_link] = area
                self.sideGraph[new_link] = [None] * 6 
                self.edge_graph[new_link] = []
            self.sensors = []
            self.motors = []
            pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
            self.parentDict = {}
            self.labels = {}
            self.joint_parent_child = {}
            self.joint_positions = {}
            self.joint_axis = {}
            self.link_sensor_check = {}
            self.link_positions = {}
            self.link_sizes = {}
            for link in self.areaSort:
                self.sideGraph[link] = [None] * 6
                self.edge_graph[link] = []

            self.areaSort = sorted(self.areaLink, key=self.areaLink.get, reverse=True)
            label = 0
            for s in self.areaSort:
                self.labels[s] = label
                label += 1
            for i, link in enumerate(self.areaSort):
                if i == 0:
                    continue
                larger_blocks = self.areaSort[:i]
                while True:
                    choose_block = random.choice(larger_blocks)
                    if len(self.edge_graph[choose_block]) < 6:
                        self.edge_graph[choose_block].append(link)
                        break
                    larger_blocks.remove(choose_block)

            self.findZ = (len(self.areaSort)/2) + 1
            for i, link in enumerate(self.areaSort):
                if i == 0:
                    self.parentDict[link] = None
                connectLinks = self.edge_graph[link]
                available_indices = [i for i, x in enumerate(self.sideGraph[link]) if x is None]
                for connectLink in connectLinks:
                    chosen_side = random.choice(available_indices)
                    self.sideGraph[link][chosen_side] = connectLink
                    if chosen_side % 2 == 0:
                        self.sideGraph[connectLink][chosen_side + 1] = []
                        self.parentDict[connectLink] = chosen_side + 1
                    else:
                        self.sideGraph[connectLink][chosen_side - 1] = []
                        self.parentDict[connectLink] = chosen_side - 1
                    available_indices.remove(chosen_side)

            for link_i, link in enumerate(self.areaSort):
                if link in self.parentDict:
                    parent_direction = self.parentDict[link]
                else:
                    parent_direction = None
                if link_i == 0:
                    sensorCheck = random.random()
                    if sensorCheck > 0.5:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]),pos=[0, 0, self.findZ/2.0],size=list(link),material_name="Green",rgba="0 1 0 1")
                        self.sensors.append(self.labels[link])
                        self.link_sensor_check[self.labels[link]] = True
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = [0, 0, self.findZ/2.0]
                    else:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]),pos=[0, 0, self.findZ/2.0],size=list(link))
                        self.link_sensor_check[self.labels[link]] = False
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = [0, 0, self.findZ/2.0]
                else:
                    randomChoice = random.randint(0,1)
                    position = findDir(parent_direction, link)
                    if randomChoice == 0:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=position,size=list(link),material_name="Green",rgba="0 1 0 1")
                        self.sensors.append(self.labels[link])
                        self.link_sensor_check[self.labels[link]] = True
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = position
                    else:
                        pyrosim.Send_Cube(name="Link" + str(self.labels[link]), pos=position,size=list(link))
                        self.link_sensor_check[self.labels[link]] = False
                        self.link_sizes[self.labels[link]] = list(link)
                        self.link_positions[self.labels[link]] = position
                connectLinks = self.sideGraph[link]

                for i, connectLink in enumerate(connectLinks):
                    if connectLink == None or connectLink == []:
                        continue
                    jointName = "Link" + str(self.labels[link]) + "_Link" + str(self.labels[connectLink])
                    randomjoint =  random.choice(["0 1 0", "1 0 0", "0 0 1"])
                    jointPos = []
                    x = link[0]
                    y = link[1]
                    z = link[2]
                    if self.labels[link] == 0:
                        jointPos = getjointPosForRoot(i, x, y, z)
                    else:
                        jointPos = getjointPos(parent_direction, i, link)
                    pyrosim.Send_Joint(name=jointName, parent="Link" + str(self.labels[link]),child="Link" + str(self.labels[connectLink]),type="revolute",position=jointPos,jointAxis=randomjoint)
                    self.motors.append(jointName)
                    self.joint_parent_child[jointName] = {'parent': "Link" + str(self.labels[link]),'child': "Link" + str(self.labels[connectLink])}
                    self.joint_positions[jointName] = jointPos
                    self.joint_axis[jointName] = randomjoint
            pyrosim.End()
            os.system("del brain" + str(self.myID) + ".nndf")
            self.Create_Brain()
        else:
            os.system("del body" + str(self.myID) + ".urdf")
            pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
            for link in self.areaSort:
                if self.link_sensor_check[self.labels[link]]:
                    pyrosim.Send_Cube(name="Link" + str(self.labels[link]),pos=self.link_positions[self.labels[link]],size=self.link_sizes[self.labels[link]],material_name="Green",rgba="0 1 0 1")
                    self.sensors.append(self.labels[link])
                else:
                    pyrosim.Send_Cube(name="Link" + str(self.labels[link]),pos=self.link_positions[self.labels[link]],size=self.link_sizes[self.labels[link]])
            for current_joint in self.motors:
                pyrosim.Send_Joint(name=current_joint, parent=self.joint_parent_child[current_joint]['parent'],child=self.joint_parent_child[current_joint]['child'],type="revolute",position=self.joint_positions[current_joint],jointAxis=self.joint_axis[current_joint])
            pyrosim.End()
            os.system("del brain" + str(self.myID) + ".nndf")
            self.Create_Brain()
            if self.numSensors == 0:
                return
            randomRow = random.randint(0, self.numSensors-1)
            randomColumn = random.randint(0, self.numMotors-1)
            self.weights[randomRow][randomColumn] = random.random()*2 - 1

    def Set_ID(self, newID):
        self.myID = newID
