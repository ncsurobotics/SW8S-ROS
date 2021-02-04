import cv2 as cv
import imutils
import ObjectDetection
import numpy as np
import random
import argparse
import utils
import time
print("q: to quit\nt: to toggle automatically detecting when there is action(current state shown by color of border)\nd: to force a detection unconditionally")

classifying=True;

if(classifying): # initialize classsifier(s)
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
###



### creating trackbars for tuning
def empty(a):
    pass

cv.namedWindow("Final")

cv.createTrackbar("Threshold1","Final",15,255,empty)#30
cv.createTrackbar("Threshold2","Final",20,255,empty)#10
# cv.createTrackbar("Sigma","Final",12,100,empty)#10



###
global fps
detectedObjects=[]
detected=None
fps = 60
isClassifying=False # if automatic contour detection to neural network detection is on
key=cv.waitKey(1)
lastTime = int(time.time())

while(not (key & 0xFF == ord('q'))): # main loop
    timer = cv.getTickCount()
    key=cv.waitKey(1)

    if (key & 0xFF == ord('t')):
        isClassifying=not isClassifying

    randColor=(random.randint(0,256),random.randint(0,256),random.randint(0,256))

    if(isVideo):
        ret, frame = webcam.read()
    else:
        frame=cv.imread(image)
    if(not(type(frame)==np.ndarray)):
        print("\n    finished\n")
        break;

    # frame = frame[:,:,0]
    real = imutils.resize(frame,width=750) # resizes the image (WIDTH MUST BE 750 OTHERWISE ALL OTHER MAGIC NUMBERS WON'T WORK)
    originalFrame = real.copy() # a copy of the original frame
    frame = real#cv.GaussianBlur(imutils.resize(imutils.resize(imutils.resize(real,width=1000),width=100),width=750),(11,11),0)

    ### processes image and get objects
    thresh1 = cv.getTrackbarPos("Threshold1", "Final")# if fps>20 else 255
    thresh2 = cv.getTrackbarPos("Threshold2", "Final")# if fps>20 else 255
    processed = ObjectDetection.getProcessed(frame,threshold1=thresh1,threshold2=thresh2)
    contours = ObjectDetection.getContours(processed)
    targets = ObjectDetection.getTargets(contours)
    boxes = ObjectDetection.getBoxes(targets)
    objects = ObjectDetection.getObjectsFinal(frame, threshval1 = thresh1, threshval2 = thresh2)#ObjectDetection.getObjects(boxes)
    gameObjects = classifier.getObjects(objects) # DNN detections
    ### image initialization for each frame
    detectingColor = (0,255,0) if isClassifying else (0,0,255)
    empty = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
    cv.rectangle(empty,(0,0),(real.shape[1], real.shape[0]),detectingColor,thickness=5)
    blank = empty.copy()
    screen = empty.copy()
    targetFrame = empty.copy()
    cv.rectangle(real,(0,0),(real.shape[1], real.shape[0]),detectingColor,thickness=5)
    if(type(detected)==type(None)):
        detected=empty.copy()

    # draw contours
    cv.drawContours(blank, contours, -1, (255, 255, 255), 1)
    # draw targets
    for t in targets:
        t.drawColor(blank,(0,255,0),thickness=1)
    # draw objects
    counter = 0
    for o in objects:
        counter+=1
        if(counter<=4):
            o.setRegion(real,screen)
            objectColor = o.getColor(real)
            o.drawColor(screen,objectColor)
        o.drawColor(frame,randColor)
    # custom classification
    if classifying:
        for o in gameObjects:
            if not type(o)==type(None):
                o.setRegion(originalFrame,targetFrame)
                o.drawColor(targetFrame,randColor,drawDescriptions=True)
        # classify with manual user input
        if (key & 0xFF == ord('d')):
            detectedObjects=classifier.getYoloObjects(originalFrame)
            detected = empty.copy()#originalFrame.copy()
            detectedFrame = originalFrame.copy()
        # DNN detection
        elif isClassifying and (not gameObjects[0]==None) and abs(int(time.time())-lastTime)>10:
            lastTime = int(time.time())
            if (gameObjects[0].area>50000):
                detectedObjects=classifier.getYoloObjects(originalFrame) 
                detected = empty.copy()#originalFrame.copy() 
                detectedFrame = originalFrame.copy()
    # writing the result of DNN in a .jpg file
    if(len(detectedObjects)>0):
        for o in detectedObjects:
            o.setRegion(detectedFrame,detected)
            o.drawColor(detected,randColor,thickness=3,drawDescriptions=True)
        # cv.imwrite("result.jpg",detected)
    
    # stitching images and writing output
    stacked = utils.stackImages(1,([blank,processed,targetFrame],[screen,frame,detected]))
    fps = cv.getTickFrequency() / (cv.getTickCount() - timer)
    cv.putText(stacked,file + " fps: "+str(int(fps)), (75, 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, (20,230,20) if fps>60 else ((230,20,20) if fps>20 else (20,20,230)), 2)
    stacked = imutils.resize(stacked,height=800)#cv.resize(stacked,(1200,850))
    cv.imshow("Final",stacked)
if(isVideo):
    webcam.release()
cv.destroyAllWindows()

