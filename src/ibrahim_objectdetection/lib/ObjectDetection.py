from __future__ import print_function

import sys

import cv2 as cv
import imutils
# import matplotlib.pyplot as plt
import numpy as np

import lib.rect
# import region
from lib.rect import Rect
# from region import Region

defaultThresh=30

# processes the image by converting to grayscale, dilating, and finding canny edges
def getProcessed(image,threshold1=15,threshold2=20,sigma=12):
    image = imutils.resize(image.copy(),width=750)
    image = cv.GaussianBlur(imutils.resize(imutils.resize(imutils.resize(image.copy(),width=1000),width=100),width=750),(11,11),0)
    img=image.copy()
    imgBlur = cv.GaussianBlur(img, (7, 7), 1)
    imgGray = cv.cvtColor(imgBlur, cv.COLOR_BGR2GRAY)
    imgGray = cv.bilateralFilter(imgGray, d = 7, sigmaSpace = 75, sigmaColor =75)
    dilationKernal = np.ones((3, 3))
    dilationIterations = 3
    auto=False
    if(not auto):
        imgCanny = cv.Canny(imgGray, threshold1,threshold2) 
        imgDilated = cv.dilate(imgCanny, dilationKernal, iterations=dilationIterations)
        return imgDilated
    else:
        v = np.mean(imgGray)
        sigma = cv.getTrackbarPos("Sigma", "Final")/100
        lower = int(max(0, (1.0 - sigma) * v))/4
        upper = int(min(255, (1.0 + sigma) * v))/4
        print(upper,lower)
        edged = cv.Canny(imgGray, lower, upper)
        dilated = cv.dilate(edged, dilationKernal, iterations=dilationIterations)
        return dilated
# returns found contours from processed image
# param processed: a processed image from getProcesssed()
# param maxContours: contour cap
def getContours(processed,maxContours=None):
    imgProcessed = processed.copy()#getProcessed(image,threshold1=thresh1,threshold2=thresh2)
    cnt, hierarchy = cv.findContours(imgProcessed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) 
    temp  = cnt
    cnt=[]
    for c in temp:
        if cv.contourArea(c)>350: # filter out
            cnt.append(c)
    cnt.sort(key=lambda contour:cv.contourArea(contour))
    cnt.reverse()
    maxContours = len(cnt) if maxContours==None else maxContours
    if len(cnt) > 0:
        cnt=cnt[0:min(maxContours,len(cnt))]
    return cnt
# gets boundingbox rect for each contour and sorts by area
# paraam cnt: contours from getContours()
def getTargets(cnt,maxTargets=None):
    targets = []
    for c in cnt:
        rect = cv.minAreaRect(c)
        # c = cv.approxPolyDP(c, 0.04 * cv.arcLength(c, True), True)
        x,y,w,h = cv.boundingRect(c)
        # region = Region(rect)
        region = Rect(x,y,w,h)
        if True:#(region.area>500):
            targets.append(region)

    # temp=targets.copy()
    # targets=[]
    # for t in temp:
    #     targets.append(t)

    targets.sort(key=lambda target:target.area)
    targets.reverse()
    maxTargets = len(targets) if maxTargets==None else maxTargets
    if len(targets) > 0:
        targets=targets[0:min(maxTargets,len(targets))]
    return targets
# this step is redundant for now because I removed min area rect, but incase it is used, this will bet the bounding rectangle around a minarea rect
def getBoxes(targets):
    boxes=[]
    if len(targets) > 0:
        for target in targets:
            boxes.append(target.getRect())
    boxes.sort(key=lambda target:target.getArea())
    boxes.reverse()
    return boxes
# converts bounding boxes into possible objects
# param boxes: boxes of type Rect determined by either getTargets() or getBoxes()
def getObjects(boxes):
    objects=[]
    if(len(boxes)>0):
        if(type(boxes[0])==Rect):
            objects = lib.rect.getClusters(boxes)
        objects.sort(key=lambda target:target.getArea())
        objects.reverse()
    else:
        objects = boxes

    temp = objects.copy()
    objects=[]
    for o in temp:
        if o.area>500:#o.isEligible(heightthresh=20,widththresh=100):
            objects.append(o)
    
    for i in range(len(objects)):
        objects[i].num= i

    return objects
# automatically combines all of the steps above and gives predicted objects based on an image
def getObjectsFinal(frame,threshval1=None,threshval2=None,minContours=None,maxContours=None,maxTargets=None):
    return getObjects(getBoxes(getTargets(getContours(getProcessed(frame,threshold1=threshval1,threshold2=threshval2),maxContours=maxContours),maxTargets=maxTargets)))








# depricated theshold tuning, might be useful later
# def tuneThresh(image,thresh,lower,upper,recursionDepth=0,minThresh=defaultThresh):
#     def getNumContours(image,threshold):
#         src_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
#         src_gray = cv.blur(src_gray, (3,3))
#         m=np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
#         ret, thresh = cv.threshold(src_gray, 50, 255, cv.THRESH_BINARY)
#         canny_output = cv.Canny(src_gray, threshold, threshold * 2)
#         cnt, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#         return len(cnt)
#     contours = getNumContours(image,thresh)
#     if(thresh <= minThresh):
#         thresh=minThresh
#     print("thresh: ",thresh,", contours: ",contours,", recursion depth: ",recursionDepth)
#     if(recursionDepth>150):#sys.getrecursionlimit()*1/4):
#         return thresh
#     if(not (contours>=lower and contours<=upper) and thresh>=minThresh):
#         contours = getNumContours(image,thresh)
#         if(contours<=lower and thresh>=minThresh):
#             if(thresh <= minThresh):
#                 return minThresh
#             return tuneThresh(image,thresh-.2,lower,upper,recursionDepth+1)
#         elif(contours>=upper):
#             return tuneThresh(image,thresh+5,lower,upper,recursionDepth+1)
#     else:
#         if(thresh<=minThresh):
#             return minThresh    
#         return (thresh)