from collections import deque
import copy

from PySide2 import QtCore, QtWidgets


# QSlider extended to make the mouse click event move the slider to the location of the mouse
class FallDetectionSliderClass(QtWidgets.QSlider):
    def mousePressEvent(self, event):
        super(FallDetectionSliderClass, self).mousePressEvent(event)
        if event.button() == QtCore.Qt.LeftButton:
            val = self.pixelPosToRangeValue(event.pos())
            self.setValue(val)

    def pixelPosToRangeValue(self, pos):
        opt = QtWidgets.QStyleOptionSlider()
        self.initStyleOption(opt)
        gr = self.style().subControlRect(QtWidgets.QStyle.CC_Slider, opt, QtWidgets.QStyle.SC_SliderGroove, self)
        sr = self.style().subControlRect(QtWidgets.QStyle.CC_Slider, opt, QtWidgets.QStyle.SC_SliderHandle, self)
        sliderLength = sr.width()
        sliderMin = gr.x()
        sliderMax = gr.right() - sliderLength + 1
        pr = pos - sr.center() + sr.topLeft()
        p = pr.x()
        return QtWidgets.QStyle.sliderValueFromPosition(self.minimum(), self.maximum(), p - sliderMin,
                                               sliderMax - sliderMin, opt.upsideDown)

class FallDetection:

    # Initialize the class with the default parameters (tested empirically)
    def __init__(self, maxNumTracks = 10, frameTime = 55, fallingThresholdProportion = 0.6, secondsInFallBuffer = 2.5):
        self.fallingThresholdProportion = fallingThresholdProportion
        self.secondsInFallBuffer = secondsInFallBuffer
        self.heightHistoryLen = int(round(self.secondsInFallBuffer * frameTime))
        
        # Changed from list to dictionary to handle arbitrary track IDs
        self.heightBuffer = {}
        self.tracksIDsInPreviousFrame = []
        
        # Changed from list to dictionary to handle arbitrary track IDs
        self.fallBufferDisplay = {}
        self.numFramesToDisplayFall = 100 # How many frames do you want to display a fall on the screen for

    # Sensitivity as given by the FallDetectionSliderClass instance
    def setFallSensitivity(self, fallingThresholdProportion):
        self.fallingThresholdProportion = fallingThresholdProportion

    # Update the fall detection results for every track in the frame
    # Update the fall detection results for every track in the frame
    def step(self, heights, tracks):
        # Decrement results for fall detection display
        for tid in list(self.fallBufferDisplay.keys()):
            self.fallBufferDisplay[tid] = max(self.fallBufferDisplay[tid] - 1, 0)
            # Clean up entries that have expired
            if self.fallBufferDisplay[tid] == 0:
                del self.fallBufferDisplay[tid]

        trackIDsInCurrFrame = []
        # Populate heights for current tracks
        for height in heights:
            # Find track with correct TID
            for track in tracks:
                # Found correct track
                if (int(track[0]) == int(height[0])):
                    tid = int(height[0])
                    
                    # Initialize buffer for new track IDs
                    if tid not in self.heightBuffer:
                        self.heightBuffer[tid] = deque([-5] * self.heightHistoryLen, maxlen=self.heightHistoryLen)
                    
                    # Initialize fall display for new track IDs
                    if tid not in self.fallBufferDisplay:
                        self.fallBufferDisplay[tid] = 0
                    
                    self.heightBuffer[tid].appendleft(height[1])
                    trackIDsInCurrFrame.append(tid)
                    
                    # Check if fallen
                    if(self.heightBuffer[tid][0] < self.fallingThresholdProportion * self.heightBuffer[tid][-1]):
                        self.fallBufferDisplay[tid] = self.numFramesToDisplayFall


        # Reset the buffer for tracks that were detected in the previous frame but not the current frame
        tracksToReset = set(self.tracksIDsInPreviousFrame) - set(trackIDsInCurrFrame) 
        for tid in tracksToReset:
            # Reset the height buffer
            if tid in self.heightBuffer:
                for frame in range(self.heightHistoryLen):
                    self.heightBuffer[tid].appendleft(-5) # Fill the buffer with -5's to remove any history for the track
        
        self.tracksIDsInPreviousFrame = copy.deepcopy(trackIDsInCurrFrame)
        
        return self.fallBufferDisplay
             

# TODO This stuff was never used in original implementation?
#     def resetFallText(self):
#         self.fallAlert.setText('Standing')
#         self.fallPic.setPixmap(self.standingPicture)
#         self.fallResetTimerOn = 0


#     def updateFallThresh(self):
#         try:
#             newThresh = float(self.fallThreshInput.text())
#             self.fallThresh = newThresh
#             self.fallThreshMarker.setPos(self.fallThresh)
#         except:
#             print('No numerical threshold')
