import random
import sys
import os

import cv2 as cv
import imutils
import numpy as np

sys.path.append("../lib")
import lib.rect
from lib.rect import Rect

class Classifier:

    # the yolo Detector DNN
    yoloDetector = 0 #cv.dnn.readNet("custom_cfg/yolov3_testing.cfg", "weights/yolov3_training_final.weights")
    # a list of which types of objects there are
    classes = []
    
    # initializes the yolo Detector
    def initYoloDetector(self,modelConfig="objectClassification/custom_cfg/yolov3_testing.cfg",modelWeights="objectClassification/weights/yolov3_training_final.weights"):
        self.yoloDetector = cv.dnn.readNet(modelConfig, modelWeights)
        self.yoloDetector.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        self.yoloDetector.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
        with open("objectClassification/classes.txt", "r") as f:
            self.classes = f.read().splitlines()
        self.classes.reverse()
    # uses the DNN to get objects
    def getYoloObjects(self,img):
        if(not type(self.yoloDetector)==cv.dnn_Net):
            self.initYoloDetector()
        image=img.copy()
        result=[]
        height, width, _ = img.shape
        blob = cv.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
        self.yoloDetector.setInput(blob)
        output_layers_names = self.yoloDetector.getUnconnectedOutLayersNames()
        layerOutputs = self.yoloDetector.forward(output_layers_names)
        boxes = []
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.2:
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)

                    x = int(center_x - w/2)
                    y = int(center_y - h/2)

                    boxes.append([x, y, w, h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)
        indexes = cv.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)
        if len(indexes)>0:
            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                # if(label=="gate"):
                #     label="dice"
                # elif(label=="dice"):
                #     label="gate"
                confidence = float(round(confidences[i],2))
                result.append(Rect(x,y,w,h,proposedObject=label,confidence=confidence))
            # for r in result:
            #     cv.rectangle(image, (x,y), (x+w, y+h), (random.randint(0,256),random.randint(0,256),random.randint(0,256)), 2)
            #     cv.putText(image, label + " " + confidence, (x, y+20), cv.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2)
            #     r.drawColor(img,(random.randint(0,256),random.randint(0,256),random.randint(0,256)),thickness=3,drawDescriptions=True)
            # cv.imwrite("result.jpg",image)
            result.sort(key = lambda obj:obj.confidence)
            result.reverse()
        return result
    # custom analyzing (pretty bad but could be improved)
    def getObjects(self,objects):
        finalObjects=[]
        gatePosts=[]
        for o in objects:
            if(o.area>100 and o.proposedType=="taller"):#gatepost conditions
                o.proposedObject = "gatePost?"
                o.confidence=-1
                gatePosts.append(o)
            elif(o.area>15000 and o.proposedType=="wider"):#gate condition
                o.proposedObject = "gate?"
                o.confidence=-1
                finalObjects.append(o)
            elif((o.area>5000 and o.proposedType=="squarish")):#dice condition
                o.proposedObject = "dice?"
                o.confidence=-1
                finalObjects.append(o)
        gate = None
        if(len(gatePosts)>=2):#gatepost to gate conversion conditions
            gate = lib.rect.boudingBox(gatePosts)
            gate.proposedObject = "gate?"
            gate.confidence=-1
        if(not gate==None):
            if(not gate.w>gate.h):
                gate=None

        finalObjects.append(gate)
        for o in finalObjects:
            if(type(o)==type(None)):
                finalObjects.remove(o)

        finalObjects.sort(key = lambda obj:obj.getArea())
        finalObjects.reverse()
        return finalObjects
