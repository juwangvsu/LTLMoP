""" 
    ===============================================
    createJTLVinput.py - LTL Pre-Processor Routines
    ===============================================
    
    Module that creates the input files for the JTLV based synthesis algorithm.
    Its functions create the skeleton .smv file and the .ltl file which
    includes the topological relations and the given spec.
"""
import numpy
import parseEnglishToLTL
import textwrap

def createSMVfile(fileName, numRegions, sensorList, robotPropList):
    ''' This function writes the skeleton SMV file.
    It takes as input a filename, the number of regions, the list of the
    sensor propositions and the list of robot propositions (without the regions).
    '''

    fileName = fileName + '.smv'
    smvFile = open(fileName, 'w')

    # Write the header
    smvFile.write(textwrap.dedent("""
    -- Skeleton SMV file
    -- (Generated by the LTLMoP toolkit)


    MODULE main
        VAR
            e : env();
            s : sys();
    """));

    # Define sensor propositions
    smvFile.write(textwrap.dedent("""
    MODULE env -- inputs
        VAR
    """));
    for sensor in sensorList:
        smvFile.write('\t\t')
        smvFile.write(sensor)
        smvFile.write(' : boolean;\n')

    smvFile.write(textwrap.dedent("""
    MODULE sys -- outputs
        VAR
    """));

    # Define the number of bits needed to encode the regions
    numBits = int(numpy.ceil(numpy.log2(numRegions)))
    for bitNum in range(numBits):
        smvFile.write('\t\tbit')
        smvFile.write(str(bitNum))
        smvFile.write(' : boolean;\n')

    # Define robot propositions
    for robotProp in robotPropList:
        smvFile.write('\t\t')
        smvFile.write(robotProp)
        smvFile.write(' : boolean;\n')

    # close the file
    smvFile.close()
    

def createLTLfile(fileName, sensorList, robotPropList, adjData, spec):
    ''' This function writes the LTL file. It encodes the specification and 
    topological relation. 
    It takes as input a filename, the list of the
    sensor propositions, the list of robot propositions (without the regions),
    the adjacency data (transition data structure) and
    a dictionary containing the specification strings.
    '''

    fileName = fileName + '.ltl'
    ltlFile = open(fileName, 'w')

    numBits = int(numpy.ceil(numpy.log2(len(adjData))))
    bitEncode = parseEnglishToLTL.bitEncoding(len(adjData), numBits)
    currBitEnc = bitEncode['current']
    nextBitEnc = bitEncode['next']
    
    # Write the header and begining of the formula
    ltlFile.write(textwrap.dedent("""
    -- LTL specification file
    -- (Generated by the LTLMoP toolkit)

    """))
    ltlFile.write('LTLSPEC -- Assumptions\n')
    ltlFile.write('\t(\n')

    # Write the environment assumptions
    # from the 'spec' input 
    ltlFile.write(spec['EnvInit'])
    ltlFile.write(spec['EnvTrans'])
    ltlFile.write(spec['EnvGoals'])
    ltlFile.write('\n\t);\n\n')

    ltlFile.write('LTLSPEC -- Guarantees\n')
    ltlFile.write('\t(\n')

    # Write the desired robot behavior
    ltlFile.write(spec['SysInit'])

    # The topological relation (adjacency)
    for Origin in range(len(adjData)):
        # from region i we can stay in region i
        ltlFile.write('\t\t\t []( (')
        ltlFile.write(currBitEnc[Origin])
        ltlFile.write(') -> ( (')
        ltlFile.write(nextBitEnc[Origin])
        ltlFile.write(')')
        
        for dest in range(len(adjData)):
            if adjData[Origin][dest]:
                # not empty, hence there is a transition
                ltlFile.write('\n\t\t\t\t\t\t\t\t\t| (')
                ltlFile.write(nextBitEnc[dest])
                ltlFile.write(') ')

        # closing this region
        ltlFile.write(' ) ) & \n ')
    

    # The rest of the spec
    ltlFile.write(spec['SysTrans'])
    ltlFile.write(spec['SysGoals'])
    # Close the LTL formula
    ltlFile.write('\n\t);\n')

    # close the file
    ltlFile.close()


