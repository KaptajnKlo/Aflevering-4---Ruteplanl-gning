import pyomo.environ as pyomo  # Used for modelling the IP
import matplotlib.pyplot as plt  # Used to plot the instance
import math  # Used to get access to sqrt() function
import readAndWriteJson as rwJson  # Used to read data from Json file

def makeEuclideanDistanceMatrix(data: dict) -> list:
    dist = []
    nrPoints = len(data['x'])
    for i in range(0, nrPoints):
        dist.append([])
        for j in range(0, nrPoints):
            tmpDist = math.sqrt((data['x'][i] - data['x'][j]) ** 2 + (data['y'][i] - data['y'][j]) ** 2)
            dist[i].append(tmpDist)
    return dist

def readData(clusterData: str) -> dict():
    data = rwJson.readJsonFileToDictionary(clusterData)
    data['nrPoints'] = len(data['x'])
    data['dist'] = makeEuclideanDistanceMatrix(data)
    return data

def buildModel(data: dict) -> pyomo.ConcreteModel():
    # Create model
    model = pyomo.ConcreteModel()
    # Copy data to model object
    model.nrPoints = data['nrPoints']
    model.points = range(0, data['nrPoints'])
    model.xCoordinates = data['x']
    model.yCoordinates = data['y']
    model.dist = data['dist']
    model.k = data['k']

    # Define variables
    model.x = pyomo.Var(model.points, model.points, within=pyomo.Binary)
    model.y = pyomo.Var(model.points, within=pyomo.Binary)
    model.rhoMax = pyomo.Var(within=pyomo.NonNegativeReals)

    # Add objective function
    model.obj = pyomo.Objective(expr=model.rhoMax)

    # Add "all represented" constraints
    model.allRep = pyomo.ConstraintList()
    for j in model.points:
        model.allRep.add(expr=sum(model.x[i, j] for i in model.points) == 1)

    # Add only represent if y[i]=1
    model.GUB = pyomo.ConstraintList()
    for i in model.points:
        for j in model.points:
            model.GUB.add(expr=model.x[i, j] <= model.y[i])

    # Add cardinality constraint on number of groups
    model.cardinality = pyomo.Constraint(expr=sum(model.y[i] for i in model.points) == model.k)

    # Add correct definition of the rhoMax variable
    model.rhoMaxDef = pyomo.ConstraintList()
    for j in model.points:
        model.rhoMaxDef.add(expr=sum(model.dist[i][j]*model.x[i, j] for i in model.points) <= model.rhoMax)

    # Fix x[i][i] == y[i]
    model.fixXandY = pyomo.ConstraintList()
    for i in model.points:
        model.fixXandY.add(expr=model.x[i,i]==model.y[i])

    #Fix Arms to be our 'Representatives'
    model.FixArm1 =pyomo.Constraint(expr=model.y[0] == 1)
    model.FixArm2 = pyomo.Constraint(expr=model.y[1] == 1)
    model.FixArm3 = pyomo.Constraint(expr=model.y[2] == 1)
    model.FixArm4 = pyomo.Constraint(expr=model.y[3] == 1)
    return model


def solveModel(model: pyomo.ConcreteModel()):
    # Set the solver
    solver = pyomo.SolverFactory('gurobi')
    # Solve the model
    solver.solve(model, tee=True)

def displaySolution(model: pyomo.ConcreteModel()):
    print('Optimal minimum of the maximal length: ', pyomo.value(model.obj))
    labels = [0]*model.nrPoints
    ptNumber = list(model.points)
    # Print the groups to promt and save coordinates for plotting
    for i in model.points:
        if pyomo.value(model.y[i]) == 1:
            print('Arm', i+1, 'represents Svejsepunkter:')
            for j in model.points:
                if pyomo.value(model.x[i, j]) == 1:
                    print (j+1,",",end='')
                    labels[j] = i
            print('\n')
    # Plot with different colors
    plt.scatter(model.xCoordinates, model.yCoordinates, c=labels)
    for i, label in enumerate(ptNumber):
        plt.annotate(ptNumber[i]+1, (model.xCoordinates[i], model.yCoordinates[i]))
    plt.show()



def main(clusterDataFile: str):
    data = readData(clusterDataFile)
    model = buildModel(data)
    solveModel(model)
    displaySolution(model)


if __name__ == '__main__':
    theDataFile = "Data"
    main(theDataFile)