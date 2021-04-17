import cv2 as cv
import imutils
import lib.ObjectDetection
import numpy as np
import random
import argparse
import lib.utils
import time

from objectClassification.Classifier import Classifier
classifier = Classifier()
classifier.initYoloDetector()


### this section determines what file to analyze
file = ""

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image",required=False,help="path to input image")
ap.add_argument("-v", "--video",required=False,help="path to input video")
args = vars(ap.parse_args())
image = (args["image"])
video = (args["video"])
isVideo = not video == None
if(isVideo):
    file=video
    if(video.isnumeric()):
        video = int(video)
        file="webcam"
else:
    file=image

if(isVideo):
    webcam = cv.VideoCapture(video)
    webcam.read()
windowName = "Reading "+file
###


key=cv.waitKey(1)
def empty(a):
    pass

###############
#  CONSTANTS  #
###############
AreaThresh = 30000
thresh1 = 10
thresh2 = 13
isGUI = True;

while(not (key & 0xFF == ord('q'))): # main loop
    timer = cv.getTickCount()
    key=cv.waitKey(1)

    if(isVideo):
        ret, frame = webcam.read()
    else:
        frame=cv.imread(image)
    if(not(type(frame)==np.ndarray)):
        print("\n    finished\n")
        break;

    real = imutils.resize(frame,width=750) # resizes the image (WIDTH MUST BE 750 OTHERWISE ALL OTHER MAGIC NUMBERS WON'T WORK)
    frame = real#cv.GaussianBlur(imutils.resize(imutils.resize(imutils.resize(real,width=1000),width=100),width=750),(11,11),0)

    ### processes image and get objects
    gameObjects = classifier.getObjects(lib.ObjectDetection.getObjectsFinal(frame, threshval1 = thresh1, threshval2 = thresh2)) # Pre DNN detections
    dnnDetections = []
    useDnn = len(gameObjects)>0 and gameObjects[0].getArea()>AreaThresh
    if(useDnn): dnnDetections = classifier.getYoloObjects(frame) # DNN detections
    preferredObjects = [] # the final object to be outputted

    if(len(dnnDetections)>0): 
        preferredObjects = dnnDetections
    elif(len(gameObjects)>0):
        preferredObjects.append(gameObjects[0])
    if(not preferredObjects == []): 
        for o in preferredObjects:
            print(o)
    else: print("Nothing detected")
    if(isGUI):
        processed = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
        if(not preferredObjects == []):
            for o in preferredObjects:
                o.setRegion(frame,processed)
                o.drawColor(processed,(0,255,0) if gameObjects[0].getArea()>AreaThresh else (0,0,255),thickness=3,drawDescriptions=True)
        cv.imshow(windowName,lib.utils.stackImages(1,([frame,processed])))
    print()
if(isVideo):
    webcam.release()
cv.destroyAllWindows()

