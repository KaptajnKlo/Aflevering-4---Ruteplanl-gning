import pyomo.environ as pyomo  # Used for modelling the IP
import matplotlib.pyplot as plt  # Used to plot the instance
import math  # Used to get access to sqrt() function
import readAndWriteJson as rwJson  # Used to read data from Json file

def makeEuclideanDistanceMatrix(data: dict) -> list:
    dist = []
    nrPoints = len(data['xCord'])
    for i in range(0, nrPoints):
        dist.append([])
        for j in range(0, nrPoints):
            tmpDist = math.sqrt((data['xCord'][i] - data['xCord'][j]) ** 2 + (data['yCord'][i] - data['yCord'][j]) ** 2)
            dist[i].append(tmpDist)
    return dist

def readData(clusterData: str) -> dict():
    data = rwJson.readJsonFileToDictionary(clusterData)
    data['nrPoints'] = len(data['xCord'])
    data['dist'] = makeEuclideanDistanceMatrix(data)
    return data

def buildModel(data: dict) -> pyomo.ConcreteModel():
    model = pyomo.ConcreteModel()
    model.numOfNodes = data['n']
    # Add descriptive comments here
    model.nodes = range(0, model.numOfNodes)
    model.customers = range(1, model.numOfNodes)
    model.x = pyomo.Var(model.nodes, model.nodes, within=pyomo.Binary)
    model.f = pyomo.Var(model.nodes, model.nodes, within=pyomo.NonNegativeReals)
    # Add descriptive comments here
    for i in model.nodes:
        model.x[i, i].fix(0)
    # Add descriptive comments here
    model.obj = pyomo.Objective(
        expr=sum(data['dist'][i][j]*model.x[i, j] for i in model.nodes for j in model.nodes if i != j)
    )
    # Add descriptive comments here
    model.sumToOne = pyomo.ConstraintList()
    for i in model.nodes:
        # Out of node i
        model.sumToOne.add(expr=sum(model.x[i, j] for j in model.nodes if i != j) == 1)
        # Into node i
        model.sumToOne.add(expr=sum(model.x[j, i] for j in model.nodes if i != j) == 1)
    # Add descriptive comments here
    model.GeneralizedBounds = pyomo.ConstraintList()
    for i in model.nodes:
        for j in model.nodes:
            model.GeneralizedBounds.add(expr=model.f[i, j] <= data['n']*model.x[i, j])
            model.GeneralizedBounds.add(expr=model.f[i, j] >= min(i, 1)*model.x[i, j])
    # Add descriptive comments here
    model.flowConservation = pyomo.ConstraintList()
    for i in model.customers:
        model.flowConservation.add(
            expr=sum(model.f[i, j] for j in model.nodes) == sum(model.f[j, i] for j in model.nodes) + 1
        )
    return model

def solveModel(model: pyomo.ConcreteModel()):
    # Set the solver
    solver = pyomo.SolverFactory('gurobi')
    # Solve the model
    solver.solve(model, tee=True)

def displaySolution(model: pyomo.ConcreteModel(), data: dict):
    print('Solution value is:', pyomo.value(model.obj))
    # Print solution information to prompt
    print('Objective function value =', pyomo.value(model.obj))
    print('Optimal tour is')
    curNode = 0
    print(curNode, '->', end='')
    KeepOn = True
    counter = 0
    # flag for testing if coordinates are present in the data
    coordinatesPresent = 'xCord' in data and 'yCord' in data
    if coordinatesPresent:
        displayX = [data['xCord'][0]]
        displayY = [data['yCord'][0]]
        labels = [0]
    # Find the route from the x[i,j] values
    while KeepOn:
        counter = counter + 1
        # Find next on route
        for i in model.nodes:
            if i != curNode and pyomo.value(model.x[curNode, i]) >= 0.98:
                if coordinatesPresent:
                    displayX.append(data['xCord'][i])
                    displayY.append(data['yCord'][i])
                if i > 0:
                    print(i, '->', end='')
                    if coordinatesPresent:
                        labels.append(i)
                else:
                    print(i, end='')
                tmpCurNode = i
        curNode = tmpCurNode
        if curNode < 1:
            break
    # Start plotting the solution to a coordinate system
    if coordinatesPresent:
        plt.plot(displayX, displayY, '-o')
        for i, label in enumerate(labels):
            plt.annotate(label, (displayX[i], displayY[i]))
        plt.show()

def main(clusterDataFile: str):
    data = readData(clusterDataFile)
    model = buildModel(data)
    solveModel(model)
    displaySolution(model, data)

if __name__ == '__main__':
    theDataFile = "Arm_4"
    main(theDataFile)