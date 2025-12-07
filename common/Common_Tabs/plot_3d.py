import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PySide2.QtGui import QPixmap
from PySide2.QtGui import QImage

from graph_utilities import eulerRot, getBoxArcs, getBoxArcs2D ,getBoxLines, getSquareLines

# Different methods to color the points 
COLOR_MODE_SNR = 'SNR'
COLOR_MODE_HEIGHT = 'Height'
COLOR_MODE_DOPPLER = 'Doppler'
COLOR_MODE_TRACK = 'Associated Track'

class Plot3D():
    def __init__(self, demo=None):
        # Create plot
        self.plot_3d = gl.GLViewWidget()
        # Sets background to a pastel grey
        self.plot_3d.setBackgroundColor(70, 72, 79)
        # Create the background grid
        gz = gl.GLGridItem()
        self.plot_3d.addItem(gz)

        # Create scatter plot for point cloud
        self.scatter = gl.GLScatterPlotItem(size=5)
        self.scatter.setData(pos=np.zeros((1,3)))
        self.plot_3d.addItem(self.scatter)
        self.boundaryBoxList = []

        # Create scatter plot for clusters
        self.scatterClusters = gl.GLScatterPlotItem(size=10)
        self.scatterClusters.setData(pos=np.zeros((1,3)))
        self.plot_3d.addItem(self.scatterClusters)

        # demo specific
        self.demo = demo
        
        # Sensor position (primary radar - backward compatibility)
        self.xOffset = 0
        self.yOffset = 0
        self.sensorHeight = 0
        self.az_tilt = 0
        self.elev_tilt = 0
        
        # === MULTI-RADAR SUPPORT: Storage for multiple radar sensors ===
        self.sensorPositions = {
            0: {'x': 0, 'y': 0, 'z': 0, 'az': 0, 'elev': 0},  # Primary radar
            1: {'x': 0, 'y': 0, 'z': 0, 'az': 0, 'elev': 0},  # Radar 1
            2: {'x': 0, 'y': 0, 'z': 0, 'az': 0, 'elev': 0}   # Radar 2
        }
        self.radarMarkers = {}  # GLScatterPlotItem for each radar dot
        self.radarBoxes = {}    # EVM boxes for additional radars
        # === END MULTI-RADAR SUPPORT ===
    
        # Create box to represent EVM (primary radar)
        evmSizeX = 0.0625
        evmSizeZ = 0.125
        verts = np.empty((2,3,3))
        verts[0,0,:] = [-evmSizeX, 0, evmSizeZ]
        verts[0,1,:] = [-evmSizeX,0,-evmSizeZ]
        verts[0,2,:] = [evmSizeX,0,-evmSizeZ]
        verts[1,0,:] = [-evmSizeX, 0, evmSizeZ]
        verts[1,1,:] = [evmSizeX, 0, evmSizeZ]
        verts[1,2,:] = [evmSizeX, 0, -evmSizeZ]
        self.evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
        self.plot_3d.addItem(self.evmBox)

        # Initialize other elements
        self.boundaryBoxViz = []
        self.coordStr = []
        self.classifierStr = []
        self.ellipsoids = []
        self.plotComplete = 1

        self.zRange = [-3, 3]

        # Persistent point cloud
        self.previousClouds = []
        if self.demo == None:
            self.numPersistentFrames = int(3)
        elif self.demo == "LPD":
            self.numPersistentFrames = int(20)
            self.plot_3d.pan(0, 1, 0)

        self.mpdZoneType = None
        self.snapTo2D = None
        self.modeSwitchLabel = None

    def updatePointCloud(self, outputDict):
        if ('pointCloud' in outputDict and 'numDetectedPoints' in outputDict):
            pointCloud = outputDict['pointCloud']
            pointCloud = np.asarray(pointCloud)

            # Rotate point cloud and tracks to account for elevation and azimuth tilt
            # if (self.elev_tilt != 0 or self.az_tilt != 0):
            #     for i in range(outputDict['numDetectedPoints']):
            #         rotX, rotY, rotZ = eulerRot (pointCloud[i,0], pointCloud[i,1], pointCloud[i,2], self.elev_tilt, self.az_tilt)
            #         pointCloud[i,0] = rotX
            #         pointCloud[i,1] = rotY
            #         pointCloud[i,2] = rotZ

            # Shift points to account for sensor height
            # if (self.sensorHeight != 0):
                # pointCloud[:,2] = pointCloud[:,2] + self.sensorHeight

            if self.demo == "LPD":
                outputDict['pointCloud'] = self.filterPointCloud(outputDict['pointCloud'])
            # Add current point cloud to the cumulative cloud if it's not empty
            self.previousClouds.append(outputDict['pointCloud'])
        else:
            # if there is no point cloud, append an empty array
            self.previousClouds.append([])

        # If we have more point clouds than needed, stated by numPersistentFrames, delete the oldest ones 
        while(len(self.previousClouds) > self.numPersistentFrames):
            self.previousClouds.pop(0)
            
    # Add a boundary box to the boundary boxes tab
    def addBoundBox(self, name, minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0, color='b'):
        newBox = gl.GLLinePlotItem()
        newBox.setVisible(True)
        self.plot_3d.addItem(newBox)     

        if ('mpdBoundaryArc' in name):
            try:
                if(self.snapTo2D.checkState() == 0):
                    boxLines = getBoxArcs(minX,minY,minZ,maxX,maxY,maxZ)
                elif(self.snapTo2D.checkState() == 2):
                    boxLines = getBoxArcs2D(minX,minY,0,maxX,maxY,0)
                
                
                boxColor = pg.glColor('b')
                newBox.setData(pos=boxLines,color=boxColor,width=2,antialias=True,mode='lines')

                # TODO add point boundary back into visualizer

                boundaryBoxItem = {
                    'plot': newBox,
                    'name': name,
                    'boxLines': boxLines,
                    'minX': float(minX),
                    'maxX': float(maxX),
                    'minY': float(minY),
                    'maxY': float(maxY),
                    'minZ': float(minZ),
                    'maxZ': float(maxZ)
                }   

                self.boundaryBoxViz.append(boundaryBoxItem) 
                self.plot_3d.addItem(newBox)
                self.boundaryBoxList.append(newBox)
            except:
                # You get here if you enter an invalid number
                # When you enter a minus sign for a negative value, you will end up here before you type the full number
                pass
        else:
            try:
                if(self.snapTo2D.checkState() == 0):
                    boxLines = getBoxLines(minX,minY,minZ,maxX,maxY,maxZ)
                elif(self.snapTo2D.checkState() == 2):
                    boxLines = getSquareLines(minX,minY,0,maxX,maxY,0)
 
                boxColor = pg.glColor(color)
                newBox.setData(pos=boxLines,color=boxColor,width=2,antialias=True,mode='lines')

                # TODO add point boundary back into visualizer

                boundaryBoxItem = {
                    'plot': newBox,
                    'name': name,
                    'boxLines': boxLines,
                    'minX': float(minX),
                    'maxX': float(maxX),
                    'minY': float(minY),
                    'maxY': float(maxY),
                    'minZ': float(minZ),
                    'maxZ': float(maxZ)
                }   

                self.boundaryBoxViz.append(boundaryBoxItem) 
                self.plot_3d.addItem(newBox)
                self.boundaryBoxList.append(newBox)
            except:
                # You get here if you enter an invalid number
                # When you enter a minus sign for a negative value, you will end up here before you type the full number
                pass

    def removeAllBoundBoxes(self):
        # === MULTI-RADAR SUPPORT: Hide radar markers ===
        for marker in self.radarMarkers.values():
            marker.setVisible(False)
        for box in self.radarBoxes.values():
            box.setVisible(False)
        # === END MULTI-RADAR SUPPORT ===
        
        for item in self.boundaryBoxList:
            item.setVisible(False)
        if(self.snapTo2D is not None):
            self.snapTo2D.setEnabled(1)
        if(self.modeSwitchLabel is not None):
            self.modeSwitchLabel.setText('Two Pass Mode Disabled')
            self.modeSwitchLabel.setStyleSheet("background-color: lightgrey; border: 1px solid black;")
        self.boundaryBoxList.clear()

    def changeBoundaryBoxColor(self, box, color):
        box['plot'].setData(pos=box['boxLines'], color=pg.glColor(color),width=2,antialias=True,mode='lines')

    def changeBoundaryBoxBold(self, box, bold, strip):
        if bold:
            box['plot'].setData(width=8)
        else:
            box['plot'].setData(width=2)
        
        if strip:
            box['plot'].setData(mode='line_strip')
        else:
            box['plot'].setData(mode='lines')


    def parseTrackingCfg(self, args):
        self.maxTracks = int(args[4])

    def parseBoundaryBox(self, args):
        self.snapTo2D.setEnabled(0)

        if (args[0] == 'SceneryParam' or args[0] == 'boundaryBox'):
            leftX = float(args[1])
            rightX = float(args[2])
            nearY = float(args[3])
            farY = float(args[4])
            bottomZ = float(args[5])
            topZ = float(args[6])
                        
            self.addBoundBox('trackerBounds', leftX, rightX, nearY, farY, bottomZ, topZ)
        elif (args[0] == 'zoneDef'):
            zoneIdx = int(args[1])
            minX = float(args[2])
            maxX = float(args[3])
            minY = float(args[4])
            maxY = float(args[5])
            # Offset by 3 so it is in center of screen
            minZ = float(args[6]) + self.sensorHeight
            maxZ = float(args[7]) + self.sensorHeight

            name = 'occZone' + str(zoneIdx)
            self.addBoundBox(name, minX, maxX, minY, maxY, minZ, maxZ)
        elif (args[0] == 'mpdBoundaryBox'):
            zoneIdx = int(args[1])
            minX = float(args[2])
            maxX = float(args[3])
            minY = float(args[4])
            maxY = float(args[5])
            minZ = float(args[6])
            maxZ = float(args[7])
            name = 'mpdBoundaryBox' + str(zoneIdx)
            self.addBoundBox(name, minX, maxX, minY, maxY, minZ, maxZ)
        elif (args[0] == 'mpdBoundaryArc'):
            zoneIdx = int(args[1])
            minR = float(args[2])
            maxR = float(args[3])
            minTheta = float(args[4])
            maxTheta = float(args[5])
            minZ = float(args[6])
            maxZ = float(args[7])
            name = 'mpdBoundaryArc' + str(zoneIdx)
            self.addBoundBox(name, minR, maxR, minTheta, maxTheta, minZ, maxZ)

        # TODO print out somewhere these boundary boxes

    def parseSensorPosition(self, args, is_x843, radar_index=0):
        """
        Parse sensor position for multiple radars
        
        Args:
            args: Configuration arguments from cfg file
            is_x843: Boolean indicating if device is x843 family
            radar_index: Which radar (0=primary, 1=second, 2=third)
        """
        print(f"[Multi-Radar] Parsing sensor position for radar {radar_index}")
        
        # Parse position based on device type
        if is_x843:
            # x843 format: sensorPosition <height> <azimuthTilt> <elevationTilt>
            self.sensorPositions[radar_index]['x'] = 0
            self.sensorPositions[radar_index]['y'] = 0
            self.sensorPositions[radar_index]['z'] = float(args[1])
            self.sensorPositions[radar_index]['az'] = float(args[2])
            self.sensorPositions[radar_index]['elev'] = float(args[3])
        else:
            # x432/x844 format: sensorPosition <x> <y> <z> <azimuthTilt> <elevationTilt>
            self.sensorPositions[radar_index]['x'] = float(args[1])
            self.sensorPositions[radar_index]['y'] = float(args[2])
            self.sensorPositions[radar_index]['z'] = float(args[3])
            self.sensorPositions[radar_index]['az'] = float(args[4])
            self.sensorPositions[radar_index]['elev'] = float(args[5])
        
        # Update primary radar variables for backward compatibility
        if radar_index == 0:
            self.xOffset = self.sensorPositions[0]['x']
            self.yOffset = self.sensorPositions[0]['y']
            self.sensorHeight = self.sensorPositions[0]['z']
            self.az_tilt = self.sensorPositions[0]['az']
            self.elev_tilt = self.sensorPositions[0]['elev']
            
            # Update primary EVM box
            self.evmBox.resetTransform()
            if self.demo == "LPD":
                self.elev_tilt = -1 * self.elev_tilt
                self.az_tilt = -1 * self.az_tilt
            self.evmBox.rotate(-1 * self.elev_tilt, 1, 0, 0)
            self.evmBox.rotate(-1 * self.az_tilt, 0, 0, 1)
            self.evmBox.translate(0, 0, self.sensorHeight)
        
        # Create or update radar marker
        self.updateRadarMarker(radar_index)
        
        print(f"[Multi-Radar] Radar {radar_index} position: "
              f"X={self.sensorPositions[radar_index]['x']:.2f}m, "
              f"Y={self.sensorPositions[radar_index]['y']:.2f}m, "
              f"Z={self.sensorPositions[radar_index]['z']:.2f}m, "
              f"Az={self.sensorPositions[radar_index]['az']:.1f}°, "
              f"Elev={self.sensorPositions[radar_index]['elev']:.1f}°")

    def updateRadarMarker(self, radar_index):
        """
        Create or update the visual marker (dot) for a radar sensor
        
        Args:
            radar_index: Which radar (0, 1, or 2)
        """
        # Define colors for each radar
        radar_colors = [
            (1.0, 0.0, 0.0, 1.0),  # Red for primary radar (radar 0)
            (0.0, 0.0, 1.0, 1.0),  # Blue for radar 1
            (0.0, 1.0, 0.0, 1.0)   # Green for radar 2
        ]
        
        # Get position from stored values
        pos = self.sensorPositions[radar_index]
        position = np.array([[pos['x'], pos['y'], pos['z']]])
        
        # Create new marker if it doesn't exist
        if radar_index not in self.radarMarkers:
            marker = gl.GLScatterPlotItem()
            marker.setGLOptions('opaque')
            self.plot_3d.addItem(marker)
            self.radarMarkers[radar_index] = marker
            print(f"[Multi-Radar] Created new marker for radar {radar_index}")
        
        # Update marker position and appearance
        self.radarMarkers[radar_index].setData(
            pos=position,
            color=radar_colors[radar_index],
            size=12
        )
        self.radarMarkers[radar_index].setVisible(True)
        
        # Create EVM box for additional radars (radar 1 and 2)
        if radar_index > 0:
            if radar_index not in self.radarBoxes:
                # Create box to represent EVM
                evmSizeX = 0.0625
                evmSizeZ = 0.125
                verts = np.empty((2,3,3))
                verts[0,0,:] = [-evmSizeX, 0, evmSizeZ]
                verts[0,1,:] = [-evmSizeX, 0, -evmSizeZ]
                verts[0,2,:] = [evmSizeX, 0, -evmSizeZ]
                verts[1,0,:] = [-evmSizeX, 0, evmSizeZ]
                verts[1,1,:] = [evmSizeX, 0, evmSizeZ]
                verts[1,2,:] = [evmSizeX, 0, -evmSizeZ]
                
                # Use the radar's color for the box edge
                color_tuple = radar_colors[radar_index][:3]  # Get RGB without alpha
                
                evmBox = gl.GLMeshItem(
                    vertexes=verts,
                    smooth=False,
                    drawEdges=True,
                    edgeColor=pg.glColor(color_tuple),
                    drawFaces=False
                )
                self.plot_3d.addItem(evmBox)
                self.radarBoxes[radar_index] = evmBox
                print(f"[Multi-Radar] Created EVM box for radar {radar_index}")
            
            # Update box transform to match radar position and orientation
            box = self.radarBoxes[radar_index]
            box.resetTransform()
            
            elev = pos['elev']
            az = pos['az']
            if self.demo == "LPD":
                elev = -1 * elev
                az = -1 * az
            
            box.rotate(-1 * elev, 1, 0, 0)
            box.rotate(-1 * az, 0, 0, 1)
            box.translate(pos['x'], pos['y'], pos['z'])

    def filterPointCloud(self, pointCloud):
        # filter the point cloud to only include points within the bounds of the boundary boxes
        newPointCloud = []
        #print("Empty new point cloud? : " + str(newPointCloud))
        for point in pointCloud:
            for boxNum in range(len(self.boundaryBoxViz)):
                # check if the point is within the bounds of the box
                if (self.boundaryBoxViz[boxNum]['minX'] <= point[0] and point[0] <= self.boundaryBoxViz[boxNum]['maxX'] and
                    self.boundaryBoxViz[boxNum]['minY'] <= point[1] and point[1] <= self.boundaryBoxViz[boxNum]['maxY'] and
                    self.boundaryBoxViz[boxNum]['minZ'] <= point[2] and point[2] <= self.boundaryBoxViz[boxNum]['maxZ']):
                    newPointCloud.append(point)
                    # TODO STORE WHICH BOX THIS PIN IS INSIDE OF AND USE IT TO COLOR POINT TO THE BOX AND PROB LINES
                    break # if within a box then move on to next point
        return np.asarray(newPointCloud)