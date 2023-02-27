from pyrosim.commonFunctions import Save_Whitespace

class MATERIAL: 

    def __init__(self, material_name="Cyan", rgba="0 1.0 1.0 1.0"):

        self.depth  = 3

        self.string1 = '<material name="' + material_name + '">'

        self.string2 = '<color rgba="' + rgba + '"/>'

        self.string3 = '</material>'

    def Save(self,f):

        Save_Whitespace(self.depth,f)

        f.write( self.string1 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string2 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string3 + '\n' )