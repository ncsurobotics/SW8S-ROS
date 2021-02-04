from __future__ import print_function
import cv2 as cv
import random
import numpy as np
# from sklearn.cluster import KMeans


class Rect:
    x,y,w,h = 0,0,0,0
    thresh = 50 # the threshold for how close another box needs to be to combine
    num = 0 # id for a box to differentiate between boxes
    area = 0
    color = (255,0,0)#(rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    confidence = 0
    proposedType="" # taller, wider, squarish
    proposedObject="" # gate, dice
    region=None
    image=None
    objects=[]

    def __init__(self,x,y,w,h,number=None,proposedObject="",confidence=0):
        from objectClassification.Classifier import Classifier
        self.x = x if x>0 else 0
        self.y = y if y>0 else 0
        self.w = w if w>0 else 0
        self.h = h if h>0 else 0
        self.area = self.w*self.h
        self.confidence=confidence

        if(self.h/self.w>1.3):
            self.proposedType="taller"
        elif(self.w/self.h>1.3):
            self.proposedType="wider"
        else:
            self.proposedType="squarish"
        self.proposedObject=proposedObject
        self.num = number if not number==None else random.randint(0,1000)
    # getters, basically useless
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getW(self):
        return self.w
    def getH(self):
        return self.h
    def getNum(self):
        return self.num
    def getArea(self):
        return self.w*self.h
    # prints the box's imformation
    def show(self):
        print("X:{} , Y:{}, W:{}, H:{}, NUM:{}".format(self.x,self.y,self.w,self.h,self.num))
    # returns a list of boxes that are near or colliding with it
    def getCandidates(self, rects,threshold=thresh):
        cand = []
        for r in rects:
            if(self.getDistance(r)<=threshold or self.isIn(r)):
                cand.append(r)
        return cand
    # returns itself, basically useless
    def getRect(self):
        return self
    # returns a list of boxes inside of itself
    def getIn(self, rects):
        cand = []
        for r in rects:
            if(self.isIn(r)):
                cand.append(r)
        return cand
    # returns center x
    def getCenterX(self):
        return self.x+self.w//2
    # returns center y
    def getCenterY(self):
        return self.y+self.h//2   
    # gets distance between the center of this box and another one 
    def getDistance(self,rect):
        #return self.thresh+1
        #return min( [ abs(self.x-rect.getX()),abs(self.x-(rect.getX()+rect.getW())),abs(self.x+self.w-(rect.getX()+rect.getW())),abs(self.x+self.w-(rect.getX())),abs(self.y-rect.getY()),abs(self.y-(rect.getY()+rect.getH())),abs(self.y+self.h-(rect.getY()+rect.getH())),abs(self.y+self.h-(rect.getY())) ])
        return ((rect.getCenterY()-self.getCenterY())**2+(rect.getCenterX()-self.getCenterX())**2)**.5
    # returns if the box is too close based on the threshold, this doesn't compare the distance of the centers, rather it looks at the sides and how close they are
    def getTooClose(self,rect,threshold=thresh):
        #return self.getDistance(rect)<=self.thresh
        def between(first, lower, upper):
            return first>=lower and first<=upper
        if(between(self.x,rect.x,rect.x+rect.w) or between(self.x+self.w,rect.x,rect.x+rect.w)):
            return abs((self.y)-(rect.y))<=threshold or abs((self.y+self.h)-(rect.y))<=threshold or abs((self.y)-(rect.y+rect.h))<=threshold or abs((self.y+self.h)-(rect.y+self.h))<=threshold
        elif(between(self.y,rect.y,rect.y+rect.h) or between(self.y+self.h,rect.y,rect.y+rect.h)):
            return abs((self.x)-(rect.x))<=threshold or abs((self.x+self.w)-(rect.x))<=threshold or abs((self.x)-(rect.x+rect.w))<=threshold or abs((self.x+self.w)-(rect.x+self.w))<=threshold
        return False
    # draws red if too small, otherwise green
    def drawSpecial(self,image):
        temp=not self.eligibleContour()
        cv.rectangle(image,(self.x,self.y),(self.x+self.w,self.y+self.h),(0,255*(not(temp)),255*temp),2)
        #cv.rectangle(image,(self.getCenterX(),self.getCenterY()),(self.getCenterX()+1,self.getCenterY()+1),(0,255*(not(temp)),255*temp),2)
        #cv.putText(image,"{}".format(int(self.num)),(self.getCenterX(),self.getCenterY()),cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255*(not(temp)),255*temp),2)
    # draws it with it's color
    def draw(self,image):
        cv.rectangle(image,(self.x,self.y),(self.x+self.w,self.y+self.h),self.color,2)    
    # draws it a predetermined color
    def drawColor(self,image,c,thickness=2,drawDescriptions=False): 
        cv.rectangle(image,(self.x,self.y),(self.x+self.w,self.y+self.h),c,thickness)
        # cv.putText(image,"{}".format(int(self.getArea())),(self.getCenterX(),self.getCenterY()),cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255),1)
        if drawDescriptions:
            cv.putText(image,"{}".format(str(self.proposedObject)+str(self.confidence)),(self.getCenterX(),self.getCenterY()),cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),1)
    # returns if there is a collision between self and other
    def isIn(self, rect):
        x2 = rect.getX()
        y2 = rect.getY()
        w2 = rect.getW()
        h2 = rect.getH()
        return (((x2 + w2 >= self.x) and (x2 <= self.x + self.w)) and ((y2 <= self.y + self.h) and (y2 + h2 >= self.y))) or self.isEqual(rect)
    # returns a cropped image of itself, from the original image
    def getRegion(self, image):
        return image[self.y:self.y+self.h,self.x:self.x+self.w]
    # "pastes" this cropped imaged on another one
    def setRegion(self, source, destination):
        destination[self.y:self.y+self.h, self.x:self.x+self.w] = self.getRegion(source)
    # returns whether this box is eligible to be a contour (a really inefficient way to detect noise)
    def eligibleContour(self):
        return (self.w>10 and self.h>10) or (self.w>50 or self.h>50)
    # retuns whether self perfectly fits around other
    def encompasses(self,rect):
        return self.x<rect.getX() and self.w+self.x>rect.getX()+rect.getW() and self.y<rect.getY() and self.h+self.y>rect.getY()+rect.getW()
    # returns if self is the same as other
    def isEqual(self,rect):
        return self == rect or self.getArea==rect.getArea() or (self.x==rect.getX() and self.y==rect.getY() and self.w==rect.getW() and self.h==rect.getH())
    # returns if self is eligible (height and width are larger than a certain value)
    def isEligible(self,heightthresh=None,widththresh=None):
        return not (self.h<heightthresh and self.w<widththresh)
    # uses DNN to set self's object type
    def setObjectType(self,image,classifier):
        img = self.getRegion(image.copy()) 
        # cv.imshow("image",img)
        obj = classifier.getYoloObjects(img)
        if(len(obj)>0):
            obj=obj[0]
            self.confidence = obj.confidence
            self.proposedObject = obj.proposedObject
    # predicts dominant color
    def getColor(self,image):
        screen = self.getRegion(image).copy()
        # img = cv.cvtColor(screen, cv.COLOR_BGR2RGB)
        # img = screen.reshape((screen.shape[0] * screen.shape[1], 3))
        # cluster = KMeans(n_clusters=5).fit(img)
        # labels = np.arange(0, len(np.unique(cluster.labels_)) + 1)
        # (hist, _) = np.histogram(cluster.labels_, bins = labels)
        # hist = hist.astype("float")
        # hist /= hist.sum()
        # colors = ([(percent, color) for (percent, color) in list(zip(hist,cluster.cluster_centers_))])
        # colors.sort(key=lambda element:element[0])
        # # colors.reverse()
        # return colors[0][1]

        # colors, count = np.unique(screen.reshape(-1,screen.shape[-1]), axis=0, return_counts=True)
        # colors = colors[count.argmax()]
        # color =  (int(colors[0]),int(colors[1]),int(colors[2]))
        # return color

        color = screen.mean(axis=0).mean(axis=0)
        # mix = color[0]/255
        # color[0] = 0
        # color[1] = color[1]*(1-mix)+255*mix
        # color[2]= color[2]*(1-mix)+255*mix
        # print(color)
        return color
# returns the bounding box around multiple rects
def boudingBox(rectangles,n=None):
    minX=[]
    minY=[]
    maxX=[]
    maxY=[]
    if(type(rectangles)==Rect):
        rectangles=[rectangles]
    if(not rectangles==None and len(rectangles)>0):
        for r in rectangles:
            minX.append(r.getX())
            maxX.append(r.getX()+r.getW())
            minY.append(r.getY())
            maxY.append(r.getY()+r.getH())
        return Rect(min(minX),min(minY),max(maxX)-min(minX),max(maxY)-min(minY),n)
# the condition on which 2 boxes will combine
def isConflicting(l, index1,index2):
    return (not(index1==index2)) and ((l[index1].isIn(l[index2]) or l[index1].getTooClose(l[index2])))
# returns if there are still any conflicts remaining
def conflictsRemain(l):
        for group in range(len(l)):
            for rect in range(len(l)):
                    for r in range(len(l)):
                        if(isConflicting(l,rect,r)):
                            return True
        return False;
# recursively combines rects until a big supposed object is created
def getClusters(rectangles):
    rects = rectangles.copy()
    if(conflictsRemain(rects)):
        for rect in range(len(rects)):
            for other in range(len(rects)):
                if(isConflicting(rects,rect,other)):
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
        return rects