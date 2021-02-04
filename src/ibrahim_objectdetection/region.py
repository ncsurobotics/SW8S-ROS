from __future__ import print_function
import cv2 as cv
import numpy as np
import copy, operator
from enum import Enum
import imutils
import matplotlib.pyplot as plt
from rect import Rect
import random

class Region:
    x = 0
    y = 0
    width = 0
    height = 0
    angle = 0
    area = 0
    boxpoints = 0
    left=0
    right=0
    up=0
    down=0
    num=0
    def __init__(self, target):
        self.x = target[0][0]
        self.y = target[0][1]
        self.angle = target[2]
        if abs(self.angle) < 45:
            self.width = target[1][0]
            self.height = target[1][1]
        else:
            self.width = target[1][1]
            self.height = target[1][0]
        self.area = self.width*self.height
        self.boxpoints=np.int0(cv.boxPoints(target))
        xs = [i[0] for i in self.boxpoints]
        ys = [i[1] for i in self.boxpoints]
        self.left=int(min(xs))
        self.right=int(max(xs))
        self.up=int(min(ys))
        self.down=int(max(ys))
        self.num=random.randint(0,1000)
    def drawColor(self,image, color,thickness=1):
        # xs = [i[0] for i in self.boxpoints]
        # ys = [i[1] for i in self.boxpoints]
        # left=int(min(xs))
        # right=int(max(xs))
        # up=int(min(ys))
        # down=int(max(ys))
        # cv.rectangle( frame , (left,up) , (right,down) , (255,255,255) , 1)
        cv.drawContours(image,[self.boxpoints],0,color,thickness)
        #cv.putText(image,"{}".format(int(self.num)),(int(self.x),int(self.y)),cv.FONT_HERSHEY_SIMPLEX, 0.7, color,2)
    def getRect(self):
        return Rect(self.left,self.up,self.right-self.left,self.down-self.up)
    def isParticle(self,heighthresh,widththresh):
        return self.down-self.up<heighthresh and self.right-self.left<widththresh
    def isEligible(self,heightthresh=None,widththresh=None):#particleheightThresh=20,particlewidthThresh=100):
        return not(self.isParticle(heightthresh,widththresh))
    def isIn(self, region):
        def between(first, lower, upper):
            return first>=lower and first<=upper
        return(between(self.left,region.left,region.right) or between(self.right,region.left,region.right)) and (between(self.up,region.up,region.down) or between(self.down,region.up,region.down))

def conflictsRemain(l):
        for group in range(len(l)):
            for rect in range(len(l)):
                    for r in range(len(l)):
                        #print(l[group][rect].getNum() , ((l[group][rect].isIn(l[g][r]) or l[group][rect].getTooClose(l[g][r])) and (group==g and rect==r)))
                        if(((l[rect].isIn(l[r]) and not(rect==r)))):#not(l[group][rect].isEqual(l[g][r])))):
                            return True
        return False;


def boudingBox(regions):
    lefts=[i.left for i in regions]
    rights=[i.right for i in regions]
    ups=[i.up for i in regions]
    downs=[i.down for i in regions]
    left=int(min(lefts))
    right=int(max(rights))
    up=int(min(ups))
    down=int(max(downs))
    return Rect(left,up,right-left,down-up)

def conflictsRemain(l):
        for group in range(len(l)):
            for rect in range(len(l)):
                    for r in range(len(l)):
                        #print(l[group][rect].getNum() , ((l[group][rect].isIn(l[g][r]) or l[group][rect].getTooClose(l[g][r])) and (group==g and rect==r)))
                        if(((l[rect].isIn(l[r]) and not(rect==r)))):#not(l[group][rect].isEqual(l[g][r])))):
                            return True
        return False;
def getClusters(rects):
    if(conflictsRemain(rects)):
        for rect in range(len(rects)):
            for other in range(len(rects)):
                if(not(rect==other)) and rects[rect].isIn(rects[other]):
                    bounding = boudingBox([rects[rect],rects[other]])
                    a = rects[rect]
                    b = rects[other]
                    rects.remove(a)
                    rects.remove(b)
                    rects.append(bounding)
                    return getClusters(rects)

    else:
        rects.sort(key=lambda target:target.getArea())
        rects.reverse()
        return rects[0:2]