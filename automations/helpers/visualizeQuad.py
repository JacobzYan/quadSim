from matplotlib import pyplot as plot
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import animation
import time
from pathlib import Path as path
import sys
import pandas as pd

class quadVisual:
    def __init__(self, params:dict):
        self.bodyL = params.get('l', .1)
        self.bodyW = params.get('w', .05)
        self.bodyH = params.get('h', .05)
        self.propLocs = params.get('proplocs', [
                                                    np.array([.21, -.21]),
                                                    np.array([.21, .21]),
                                                    np.array([.21, .21]),
                                                    np.array([.21, -.21])
                                                ])

        self.frontColor = params.get('frontColor', 'b')
        self.bodyColor = params.get('bodyColor', 'k')
        self.propColor = params.get('propColor', 'r')


class quadVisualizer:
    # Class that holds a matplotlib figure, 3d projected
    '''
    fig = plt.figure()
    ax = Axes3D(fig)

    track verticies of rectangle from origin of quad, set size, then add them to origin coordinate to define World frame location of verticies
    Also add primitive stalks with fake props on them
    
    Make a main function that continuously passes info to this class and updates it
    Make another version that just reads output

    stretch goals:
    read params file to size the rectangle/props
    maybe track phase to make props rotate?


    # Draw3d polygons
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    
    '''
    def __init__(self, params={}, videoName=''):
        
        # Get Params
        self.simName = params.get('simName', "Unnamed Quad Visualize")
        self.dt = params.get('dt', .05)
        self.viewOrientation = params.get('viewOrientation', [1,1,1])
        self.viewLimits = params.get('viewLimits', [2,2,2])
        self.quadVisual = quadVisual(params)


        # Create figure, set up paramaters (axes equal, set orientation, view limits, interactive mode on)
        self.fig = plot.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.set_xlim3d([0,self.viewLimits[0]])
        self.ax.set_ylim3d([0,self.viewLimits[1]])
        self.ax.set_zlim3d([0,self.viewLimits[2]])
        self.ax.set_aspect('equal')
        
        self.ax.set_title(self.simName)
        self.ax.set_xlabel('x[m]')
        self.ax.set_ylabel('y[m]')
        self.ax.set_zlabel('z[m]')


        # Plot floor
        FloorSquare = Poly3DCollection([[(0,0,0),(self.viewLimits[0],0,0),(self.viewLimits[0],self.viewLimits[1],0),(0,self.viewLimits[1],0),(0,0,0)]], 
                                       alpha=1, color='k') # Can also use .set_color()
        floor = self.ax.add_collection(FloorSquare)


        # Create quad body ref coordinates
        self.xcom = [0,0,0]
        self.RBI = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        lx = self.quadVisual.bodyL
        ly = self.quadVisual.bodyW
        lz = self.quadVisual.bodyH

        self.verticies = [ # N, W, T = +, S, E, B = -
                        np.matrix([ lx, ly, lz]), # NWT
                        np.matrix([ lx, ly,-lz]), # NWB
                        np.matrix([ lx,-ly, lz]), # NET
                        np.matrix([ lx,-ly,-lz]), # NEB
                        np.matrix([-lx,-ly, lz]), # SET
                        np.matrix([-lx,-ly,-lz]), # SEB
                        np.matrix([-lx, ly, lz]), # SWT
                        np.matrix([-lx, ly,-lz])  # SWB
                    ]
        
        self.outputVerticies = [v.tolist()[0].copy() for v in self.verticies]

        self.bodyLineIndexList = [
            (0,1), # NW
            (2,3), # NE
            (4,5), # SE
            (6,7), # SW
            (0,2), # NT
            (1,3), # NB
            (4,6), # ST
            (5,7), # SB
            (0,6), # WT
            (1,7), # WB
            (2,4), # ET
            (3,5)  # EB
        ]


        # Create wireframe plotting objects
        self.bodyLinesPlot = [self.ax.plot3D([0,1],[0,1],[0,1], '-k')[0] for _ in self.bodyLineIndexList] # wireframe body
        # print(f'self.bodyLinesPlot: {self.bodyLinesPlot}')# DEBUG
        self.axisLines = [ # axis arrows
            self.ax.quiver(0,0,0,1,0,0,color='r'), # X
            self.ax.quiver(0,0,0,0,1,0,color='g'), # Y
            self.ax.quiver(0,0,0,0,0,1,color='b') # Z
        ]

        self.updateBodyLines()
        self.RBIList = []
        self.xcomList = []



    # Re-assign plotting object data of body and axes
    def updateBodyLines(self):
        for i, line in enumerate(self.bodyLinesPlot): # Re-plot body lines
            
            x, y, z = zip(*[self.outputVerticies[x] for x in self.bodyLineIndexList[i]])
            line.set_data_3d(x,y,z)

        for i, arrow in enumerate(self.axisLines): # Re-plot axes
            # print(f'self.RBI[i]: \n{self.RBI[i]}')#DEBUG
            arrow._segments3d = [[self.xcom, (self.RBI.transpose()[i] + np.matrix(self.xcom)).tolist()[0]]]
        
    
    def locateVerticies(self, xcom: np.matrix, RBI: np.matrix):
        
        self.xcom = xcom
        self.RBI = RBI
        # Calculate new pose
        for i, v in enumerate(self.verticies):
            # print(f'calculating vertex {i}:\n{RBI}\n{v.transpose()}') # DEBUG
            self.outputVerticies[i] = (xcom + np.dot(RBI,v.transpose()).transpose()).tolist()[0] # may need to transpose RBI depending on frame/point rotation
            # print(f'result: {self.outputVerticies[i]}\nexpected result: {(xcom + np.dot(RBI,v.transpose()).transpose()).tolist()[0]}\n deviation from COM: {np.dot(RBI,v.transpose()).transpose().tolist()}') # DEBUG

        


    def updateQuad(self, xcom, RBI):

        # Calculate Body points
        self.locateVerticies(xcom, RBI)
        self.updateBodyLines()

    # Update for animation
    def update(self, frame):
            self.updateQuad(self.xcomList[frame], self.RBIList[frame])
            return *self.bodyLinesPlot, *self.axisLines


    def exportAnimation(self, xcomList, RBIList, vidName='results/quadAnimationTest.mp4'):

        self.xcomList = xcomList
        self.RBIList = RBIList

        self.dt = .005
        
        print(f'Exporting Video')
        t0 = time.time()
        writer = animation.FFMpegWriter(fps=1/self.dt)
        ani = animation.FuncAnimation(self.fig, self.update, interval=self.dt*1000, frames=len(self.xcomList))
        ani.save(vidName, writer=writer)
        print(f'time elapsed: {time.time()-t0}')


def importTrajData(inputFilepath):
    
    if(not inputFilepath.is_file()):
        print('Provided input filepath is invalid')
        exit()

    # Grouping columns to read in
    dataTypeList = [0,1,1,1,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,5,5,5,5]
    dataTypeNames = ['t', 'pos', 'vel', 'RBI', 'omegab', 'propOmegaB']
    currentDataNum = 1

    data = pd.read_csv(inputFilepath)
    dataOut = data[['t']]
    dataOut['pos'] = data[[f'stateVector_{x}' for x in range(0,3)]].values.tolist()
    dataOut['vel'] = data[[f'stateVector_{x}' for x in range(3,6)]].values.tolist()
    dataOut['RBI'] = data[[f'stateVector_{x}' for x in range(6,15)]].apply(lambda row: row.to_numpy().reshape(3,3), axis=1)
    dataOut['omegab'] = data[[f'stateVector_{x}' for x in range(15,18)]].values.tolist()
    dataOut['propOmega'] = data[[f'stateVector_{x}' for x in range(18,22)]].values.tolist()
    dataOut['propVoltages'] = data[[f'V{x}' for x in range(1,5)]].values.tolist()
    
    print(f'dataOut:\n{dataOut.head()}\n.\n.\n.\n{dataOut.tail()}') # DEBUG
    for c in dataOut.columns:
        print(f'{c}:\n{dataOut[c][1]}') # DEBUG

    return dataOut


# def importQuadData(filepath):
#     data = {}
#     with open(filepath) as file:
#         for line in file:
#             if line[0] == '#':
#                 continue
#             elif 

    





def rotz(theta):
    return np.matrix([[np.cos(theta), np.sin(theta), 0],
                      [-np.sin(theta),np.cos(theta),0],
                      [0,0,1]])

def rotx(theta):
    return np.matrix([[1,0,0],
                      [0,np.cos(theta), np.sin(theta)],
                      [0,-np.sin(theta), np.cos(theta)]])



if __name__ == '__main__':
    
    try:
        quadParamsFilepath = path(sys.argv[1])
        trajFielpath = path(sys.argv[2])
        outputFilepath = path(sys.argv[3])
    except:
        print('Please pass a quadParams file, a trajectory filepath, and an output filepath')
        exit()


    # Test Data import
    df = importTrajData(trajFielpath)
    # Test code

    # qv = quadVisualizer()
    q = quadVisualizer()

    # df = pd.DataFrame([[1,2]])
    xcomList = df['pos'].to_list()
    RBIList = df['RBI'].tolist()

    fig = plot.figure()
    ax  = fig.add_subplot(1,1,1)

    propOmegaPlot = zip([[x[0],x[1],x[2],x[3]] for x in df['propOmega'] ])

    # ax.plot(df['t'], df['propOmega']) # For later to plot everything

    q.exportAnimation(xcomList, RBIList, vidName=outputFilepath)


    if False: # Basic Visualization test
        xcomList = [
            [0,0,1],
            [0.5,0,1],
            [0.5,.5,1]
        ]
        RBIList = [
            np.array([[1,0,0],[0,1,0],[0,0,1]]),
            rotz(np.pi/4),
            rotz(np.pi/2)
        ]
        counter = 1



        for i in range(len(xcomList)):
            xcom = xcomList[i]
            RBI = RBIList[i]
            q.updateQuad(xcom, RBI)
            q.fig.savefig(f'results/quad{counter}.png')
            
            counter += 1

        t = 10
        thetaList = np.linspace(0,2*np.pi, int(t/.005))
        RBIList = [np.matmul(rotz(x),rotx(x/2)) for x in thetaList]
        xcomList = [[np.cos(x)/2+1,np.sin(x)/2+1,np.sin(x)/2+1] for x in thetaList ]

        #DEBUG
        q.RBIList = RBIList
        q.xcomList = xcomList
        # q.update(1)

        q.exportAnimation(xcomList, RBIList)










    ## EXAMPLE CODE
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import random

# x_data, y_data = [], []

# fig, ax = plt.subplots()
# line, = ax.plot([], [])

# def update(frame):
#     x_data.append(frame)
#     y_data.append(random.random())

#     line.set_data(x_data, y_data)
#     ax.relim()
#     ax.autoscale_view()

#     return line,

# ani = FuncAnimation(fig, update, interval=200)
# plt.show()