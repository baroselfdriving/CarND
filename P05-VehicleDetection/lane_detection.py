# Code from P4-Lane Detection

import pickle
import numpy as np
import cv2

#=====================================================================
# apply color and gradient threshold to extract lane pixels

def threshold(image, gradThreshMin=20, gradThreshMax=100, satThreshMin = 170, satThreshMax = 255):
    
    # For color threshold we need the s-channel from HLS space 
    # For gradient threshold we need grayscale image
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    satChn = hls[:,:,2]
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    # find x gradient
    sobelX = cv2.Sobel(gray, cv2.CV_64F, 1, 0) # Take the derivative in x
    absSobelX = np.absolute(sobelX) # Absolute x derivative to accentuate lines away from horizontal
    scaledSobel = np.uint8(255*absSobelX/np.max(absSobelX))
    
    # Threshold x gradient
    sxBinary = np.zeros_like(scaledSobel)
    sxBinary[(scaledSobel >= gradThreshMin) & (scaledSobel <= gradThreshMax)] = 1
    
    # Threshold color channel
    satBinary = np.zeros_like(satChn)
    satBinary[(satChn >= satThreshMin) & (satChn <= satThreshMax)] = 1
    
    # Combine the two binary thresholds
    combinedBinary = np.zeros_like(sxBinary)
    combinedBinary[(satBinary == 1) | (sxBinary == 1)] = 1
    
    return combinedBinary

#=====================================================================
# Given a binary top-view image with lane pixels identified, return pixel coordinates list
# for left and right lanes and a debug image
# nWindows - number of sliding windows
# searchDx - horizontal search half width
# bDebug - set to True to create a debug image showing sliding windows and lane pixels
def coarse_lane_search(binaryTopView, nWindows=9, searchDx=100, bDebug=False):
    
    # Take a histogram of the bottom half of the image. Find peaks in the left and right halves.
    # Use them as starting point for left and right lane search 
    histogram = np.sum(binaryTopView[binaryTopView.shape[0]/2:,:], axis=0)
    midpoint = np.int(histogram.shape[0]/2)
    xLeftBase = np.argmax(histogram[:midpoint])
    xRightBase = np.argmax(histogram[midpoint:]) + midpoint

    # Identify the x and y positions of all nonZero pixels in the image
    nonZero = binaryTopView.nonzero()
    nonZeroY = np.array(nonZero[0])
    nonZeroX = np.array(nonZero[1])
    
    # Current positions to be updated for each window
    xleftCurrent = xLeftBase
    xRightCurrent = xRightBase
    
    # Create empty lists to receive left and right lane pixel indices
    leftLaneInds = []
    rightLaneInds = []

    # create an image to show the sliding windows
    debugImage = None
    if bDebug:
        debugImage = np.dstack((binaryTopView, binaryTopView, binaryTopView))*255

    winHt = np.int(binaryTopView.shape[0]/nWindows)
    
    # Set minimum number of pixels found to recenter window
    minPix = (winHt * searchDx)/200

    # Step through the windows one by one
    for window in range(nWindows):
        
        # Identify window boundaries in x and y (and right and left)
        yLow = binaryTopView.shape[0] - (window+1)*winHt
        yHigh = binaryTopView.shape[0] - window*winHt
        xLeftLow = xleftCurrent - searchDx
        xLeftHigh = xleftCurrent + searchDx
        xRightLow = xRightCurrent - searchDx
        xRightHigh = xRightCurrent + searchDx
        
        # Identify the nonZero pixels in x and y within the window
        leftIndsGood = ((nonZeroY >= yLow) & (nonZeroY < yHigh) & (nonZeroX >= xLeftLow) \
                        & (nonZeroX < xLeftHigh)).nonzero()[0]
        rightIndsGood = ((nonZeroY >= yLow) & (nonZeroY < yHigh) & (nonZeroX >= xRightLow) \
                        & (nonZeroX < xRightHigh)).nonzero()[0]
        
        # Append these indices to the lists
        leftLaneInds.append(leftIndsGood)
        rightLaneInds.append(rightIndsGood)

        if bDebug:
            # Draw the windows on the visualization image
            cv2.rectangle(debugImage,(xLeftLow,yLow),(xLeftHigh,yHigh),(0,255,0), 2) 
            cv2.rectangle(debugImage,(xRightLow,yLow),(xRightHigh,yHigh),(0,255,0), 2) 
        
        # If you found > minPix pixels, recenter next window on their mean position
        if len(leftIndsGood) > minPix:
            xleftCurrent = np.int(np.mean(nonZeroX[leftIndsGood]))
        if len(rightIndsGood) > minPix:        
            xRightCurrent = np.int(np.mean(nonZeroX[rightIndsGood]))

    # Concatenate the arrays of indices
    leftLaneInds = np.concatenate(leftLaneInds)
    rightLaneInds = np.concatenate(rightLaneInds)

    # Extract left and right line pixel positions
    leftX = nonZeroX[leftLaneInds]
    leftY = nonZeroY[leftLaneInds] 
    rightX = nonZeroX[rightLaneInds]
    rightY = nonZeroY[rightLaneInds] 

    if bDebug:
        debugImage[leftY, leftX] = [255, 0, 0]
        debugImage[rightY, rightX] = [0, 0, 255]
    
    return leftX, leftY, rightX, rightY, debugImage

#=====================================================================
# Curvature and vehicle offset

# Return curvature evaluated at yEval given curve fit params
def get_curvature(leftFit, rightFit, yEval):
    leftRadius = ((1 + (2*leftFit[0]*yEval + leftFit[1])**2)**1.5) / np.absolute(2*leftFit[0])
    rightRadius = ((1 + (2*rightFit[0]*yEval + rightFit[1])**2)**1.5) / np.absolute(2*rightFit[0])
    return (leftRadius + rightRadius)/2

# Return vehicle offset from center. > 0 if vehicle is offset to the right of center
def get_offset_from_center(leftFit, rightFit, xMid, yEval):
    leftX = leftFit[0]*yEval**2 + leftFit[1]*yEval + leftFit[2]
    rightX = rightFit[0]*yEval**2 + rightFit[1]*yEval + rightFit[2]
    x = (leftX + rightX)/2
    return xMid - x

#=====================================================================
# Given a previous set of lane fit parameters in pixel space, search in the neighbourhood 
# and return updated lane pixel vectors
def fine_lane_search(binaryTopView, leftFit, rightFit, searchDx=100, bDebug = False):
    
    debugImage = None
    if bDebug:
        debugImage = np.dstack((binaryTopView, binaryTopView, binaryTopView))*255
        
    # Identify the x and y positions of all nonZero pixels in the image
    nonZero = binaryTopView.nonzero()
    nonZeroY = np.array(nonZero[0])
    nonZeroX = np.array(nonZero[1])
    
    leftLaneInds = ((nonZeroX > (leftFit[0]*(nonZeroY**2) + leftFit[1]*nonZeroY + leftFit[2] - searchDx)) \
                    & (nonZeroX < (leftFit[0]*(nonZeroY**2) + leftFit[1]*nonZeroY + leftFit[2] + searchDx))) 
    rightLaneInds = ((nonZeroX > (rightFit[0]*(nonZeroY**2) + rightFit[1]*nonZeroY + rightFit[2] - searchDx)) \
                    & (nonZeroX < (rightFit[0]*(nonZeroY**2) + rightFit[1]*nonZeroY + rightFit[2] + searchDx)))  

    # Extract left and right line pixel positions
    leftX = nonZeroX[leftLaneInds]
    leftY = nonZeroY[leftLaneInds]
    rightX = nonZeroX[rightLaneInds]
    rightY = nonZeroY[rightLaneInds]
    
    # return with reinitialise flag set if we didnt find any pixels
    if leftX.size is 0 or rightX.size is 0:
        return True, leftX, leftY, rightX, rightY, debugImage   

    if bDebug:
        leftFit = np.polyfit(leftY, leftX, 2)
        rightFit = np.polyfit(rightY, rightX, 2)        
        plotY = np.linspace(0, binaryTopView.shape[0]-1, binaryTopView.shape[0] )
        leftFitX = leftFit[0]*plotY**2 + leftFit[1]*plotY + leftFit[2]
        rightFitX = rightFit[0]*plotY**2 + rightFit[1]*plotY + rightFit[2]

        windowImg = np.zeros_like(debugImage)
        debugImage[leftY, leftX] = [255, 0, 0]
        debugImage[rightY, rightX] = [0, 0, 255]
        leftLineWindow1 = np.array([np.transpose(np.vstack([leftFitX-searchDx, plotY]))])
        leftLineWindow2 = np.array([np.flipud(np.transpose(np.vstack([leftFitX+searchDx, plotY])))])
        leftLinePts = np.hstack((leftLineWindow1, leftLineWindow2))
        rightLineWindow1 = np.array([np.transpose(np.vstack([rightFitX-searchDx, plotY]))])
        rightLineWindow2 = np.array([np.flipud(np.transpose(np.vstack([rightFitX+searchDx, plotY])))])
        rightLinePts = np.hstack((rightLineWindow1, rightLineWindow2))
        
        cv2.fillPoly(windowImg, np.int_([leftLinePts]), (0,255, 0))
        cv2.fillPoly(windowImg, np.int_([rightLinePts]), (0,255, 0))
        debugImage = cv2.addWeighted(debugImage, 1, windowImg, 0.3, 0)

    return False, leftX, leftY, rightX, rightY, debugImage

#=====================================================================
# Take the lane curvature parameters and back project the detected lane 
# on to undistorted image
def back_project(binaryTopView, leftFit, rightFit, invPersMat):
    plotY = np.linspace(0, binaryTopView.shape[0]-1, binaryTopView.shape[0] )
    leftFitX = leftFit[0]*plotY**2 + leftFit[1]*plotY + leftFit[2]
    rightFitX = rightFit[0]*plotY**2 + rightFit[1]*plotY + rightFit[2]

    # Create an image to draw the lines on
    colorWarped = np.dstack((binaryTopView, binaryTopView, binaryTopView))
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    ptsLeft = np.array([np.transpose(np.vstack([leftFitX, plotY]))])
    ptsRight = np.array([np.flipud(np.transpose(np.vstack([rightFitX, plotY])))])
    pts = np.hstack((ptsLeft, ptsRight))
    
    # Draw the lane onto the warped blank image
    cv2.fillPoly(colorWarped, np.int_([pts]), (0,255, 0))
    
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    return cv2.warpPerspective(colorWarped, invPersMat, (binaryTopView.shape[1], binaryTopView.shape[0]))

#=====================================================================
# utility functions

# read in image as RGB
def read_image(filename):
    image = cv2.imread(filename)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# apply distortion correction to the image
def undistort(image, camMat, distParams):
    return cv2.undistort(image, camMat, distParams, None, camMat)

# apply perspective transform to generate top view
def generate_top_view(image, persMat):
    imgSz = (image.shape[1], image.shape[0])
    transformed = cv2.warpPerspective(image, persMat, imgSz, flags=cv2.INTER_LINEAR)
    return transformed

#=====================================================================
# The processing pipeline

class LaneDetector:
    
    def __init__(self):
        self._initSearch = True
        self._leftFit = np.array([0, 0, 0])
        self._rightFit = np.array([0, 0, 0])
        self._debugFlag = False

        # Load camera calibration, perpective projection and scaling parameters
        # that were saved from project 4 (lane detection)
        pickleDict = pickle.load( open("p4_camera_pickle.p", "rb" ) )
        self._camMat = pickleDict["camMat"]
        self._distParams = pickleDict["distParams"]
        self._persMat = pickleDict["persMat"]
        self._invPersMat = pickleDict["invPersMat"]
        self._metersPerPixelY = pickleDict["metersPerPixelY"]
        self._metersPerPixelX = pickleDict["metersPerPixelX"]

    def set_debug(self,bFlag):
        self._debugFlag = bFlag

    def process_image(self,image):
        # NOTE: The output you return should be a color image (3 channel) for processing video below
        
        # apply distortion correction, find lane pixels and transform to top view
        undistorted = undistort(image, self._camMat, self._distParams)
        thresholded = threshold(undistorted, gradThreshMin=30, gradThreshMax=255, 
                                satThreshMin = 100, satThreshMax = 255)
        binary_plan = generate_top_view(thresholded, self._persMat)
        
        # if this is the first frame or if we lost lanes, re-initialise with a coarse search for lanes
        if self._initSearch is True:
            leftX, leftY, rightX, rightY, debugImage = coarse_lane_search(binary_plan, nWindows=9,
                                                                          searchDx=70, 
                                                                          bDebug=self._debugFlag)
            self._initSearch = False
            self._leftFit = np.polyfit(leftY, leftX, 2)
            self._rightFit = np.polyfit(rightY, rightX, 2)
            
        # already know roughly where lanes are. do fine search
        else:
            self._initSearch, leftX, leftY, rightX, rightY, debugImage = fine_lane_search(binary_plan,
                                                                                          self._leftFit,
                                                                                          self._rightFit,
                                                                                          bDebug=self._debugFlag)
        
        # if search succeeded, update curvature parameters
        curvatureInfo = "Road curvature radius: unknown"
        vehiclePosInfo = "Vehicle position unknown"
        if self._initSearch is False:
            leftFit = np.polyfit(leftY, leftX, 2)
            rightFit = np.polyfit(rightY, rightX, 2)
            
            # apply temporal smoothing with exponential filter
            alpha = 0.8
            self._leftFit = self._leftFit * alpha + leftFit * (1 - alpha)
            self._rightFit = self._rightFit * alpha + rightFit * (1 - alpha)
            
            # Fit a second order polynomial to lane pixels after scaling to real world units (meters)
            leftFit_m = np.polyfit(leftY*self._metersPerPixelY, leftX*self._metersPerPixelX, 2)
            rightFit_m = np.polyfit(rightY*self._metersPerPixelY, rightX*self._metersPerPixelX, 2)
            
            # Find the radius of curvature and vehicle offset from lane center
            radius = get_curvature(leftFit_m, rightFit_m, binary_plan.shape[0]*self._metersPerPixelY)
            offset = get_offset_from_center(leftFit, rightFit, binary_plan.shape[1]/2, binary_plan.shape[0])
            
            curvatureInfo = "Road curvature radius: %05.1f m." % (radius)
            
            if offset < 0:
                lr = "left"
            else:
                lr = "right"
            vehiclePosInfo = "Vehicle %05.2f m. %s of center" % (np.fabs(offset)*self._metersPerPixelX, lr)

        
        # overlay lane and vehicle information on output image
        if self._debugFlag is True:
            cv2.putText(debugImage, curvatureInfo, (10,30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, [255,255,255], 1) 
            cv2.putText(debugImage, vehiclePosInfo, (10,60), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, [255,255,255], 1) 
            return debugImage
        else:
            overlay = back_project(binary_plan, self._leftFit, self._rightFit, self._invPersMat)
            resultImage = cv2.addWeighted(undistorted, 1, overlay, 0.3, 0)
            cv2.putText(resultImage, curvatureInfo, (10,30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, [255,255,255], 1) 
            cv2.putText(resultImage, vehiclePosInfo, (10,60), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, [255,255,255], 1) 
            return resultImage
