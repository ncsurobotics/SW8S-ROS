# detecting underwater gate shown in forward3.avi
import cv2
import timeit
from matplotlib import pyplot as plt
### Initializes
def nothing(x):
        pass

def contourProcess(img):
    ### processing img to contours
    # apply a blur to reduce # of contours
    blur = cv2.bilateralFilter(img,5,100,100)
    # Canny edge detection
    edge = cv2.Canny(blur,5,15,apertureSize=3,L2gradient=True)
    # produce the contours and place contour over original image
    _, contours, _ = cv2.findContours(edge,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    out = cv2.drawContours(img.copy(),contours,-1,(255,0,0),2)
    # contour display
    cv2.imshow('output',out)

    ### filter contours
    location = []
    # loop through all of the contours
    for i in range(len(contours)):
        # current contour properties
        (x,y),radius = cv2.minEnclosingCircle(contours[i])
        _,_,w_rect,h_rect = cv2.boundingRect(contours[i])

        # more important properties
        aspect_ratio = float(h_rect)/w_rect
        radius = int(radius)
        
        # avoid crash when the contours[i] doesn't have enough for fitEllipse()
        if(len(contours[i])>4):
            #find angle of the contour
            _,_,angle = cv2.fitEllipse(contours[i])
        else:
            angle=-1
        
        # filter (contour close to a vertical line)
        if radius > 20 and (angle > 155 or angle < 25) and aspect_ratio > 3:
            # add the center of contours to location
            location.append([int(x),int(y)])

    
    ### adding labels
    # simplify the contours of interest
    amount_identified = len(location)
    points_of_interest = []
    if amount_identified != 0:
        # if there is only 1 filtered contour just label it as "pole" (used to be here)

        if amount_identified != 1:
            # find similar y-value pairs (if height of contour center is less than 30 pixels apart)
            # get the contour centers into pairs 
            y_range=[]
            for i in range(amount_identified):
                for j in range(i+1,amount_identified):
                    if abs(location[i][1]-location[j][1]) < 30:
                        y_range.append([i,j])
            # find whether the horizontal distance between the pairs is far enough for possible gate poles
            distGate = None
            for obj in y_range:
                x_1 = location[obj[0]][0]
                x_2 = location[obj[1]][0]
                diff = x_1 - x_2
                # attaching labels for "right" and "left" poles
                if abs(diff) > 100 and diff > 0:
                    #put center
                    center_x = (location[obj[0]][0] + location[obj[1]][0])/2
                    center_y = (location[obj[0]][1] + location[obj[1]][1])/2
                    points_of_interest.append([center_x,center_y])
                    distGate = diff
                elif abs(diff) > 100 and diff < 0:
                    #put center
                    center_x = (location[obj[0]][0] + location[obj[1]][0])/2
                    center_y = (location[obj[0]][1] + location[obj[1]][1])/2
                    points_of_interest.append([center_x,center_y])
                    distGate = diff
            points_of_interest.append(distGate) #store length as the last element
    
    return points_of_interest

# average of the center (points of interest)
# input: list of center points. 
# output: list of averaged x and y
def averageCenter(centers):
    x = 0
    y = 0
    
    if len(centers)!=0:
        for center in centers:
            x += center[0]
            y += center[1]
        return [x/len(centers),y/len(centers)]
    return []

# calculate confidence by, x_conf/y_conf = 100 - difference between center_x and center_y, then average of the two confidence
# input: current center points, past center point
# output: confidence value below 100 (can go negative)
def confidence(points_of_interest, past_point):
    x_conf = 0
    y_conf = 0
    
    if len(points_of_interest)==2 and len(past_point)==2:
        for curr_point in points_of_interest:
            x_diff = past_point[0] - curr_point[0]
            if (100 - abs(x_diff)) > x_conf:
                x_conf = 100-abs(x_diff)
            y_diff = past_point[1] - curr_point[1]
            if (100 - abs(y_diff)) > y_conf:
                y_conf = 100-abs(x_diff)
            return (x_conf+y_conf)/2
# offset from the center of the image, in pixels
def offset(origin,target):
    x = target[0] - origin[0]
    y = origin[1] - target[1]
    return [x,y]

###
# Starting part
###
# Video paths
#/home/xing/TesterCodes/OpenCV/GateProject/ff.avi
#/home/xing/TesterCodes/OpenCV/GateProject/gateCube1.avi
#/home/xing/TesterCodes/OpenCV/GateProject/gateCube2.avi
#/home/xing/TesterCodes/OpenCV/GateProject/gate2.avi
#/home/xing/TesterCodes/OpenCV/GateProject/forward2.avi
#/home/xing/TesterCodes/OpenCV/GateProject/forward3.avi
#/home/xing/TesterCodes/OpenCV/GateProject/forward12.avi

# files titled forward were better than those start with gateCube
vid = cv2.VideoCapture('gate2.avi')
ret, frame = vid.read()
height, width, _ = frame.shape
past_point = []
focus_point = []
confidence_level = 0
secondary_confidence_level = 0

#plotting
timesPlot = []
foundPlot = []
gateLengthPlot = []

while(vid.isOpened()):
    ret, frame = vid.read()
    cv2.imshow('gate_original',frame)
    #Timer
    start = timeit.default_timer()
    
    # start processing
    points = contourProcess(frame)
    if len(points) != 0:
        gateLength = points.pop(-1) #pop out the length of the gate
        if gateLength != None:
            gateLength = abs(gateLength)
            gateLengthPlot.append(gateLength/500.0)
    current_point = points      #rest are center points
    current_confidence = confidence(current_point,past_point)
    secondary_confidence = confidence(current_point,focus_point)

    # Show Confidence Levels
    past_point = averageCenter(current_point)
    if len(past_point) == 2:
        focus_point = past_point

    if current_confidence > 70:
        confidence_level = current_confidence
    else:
        confidence_level -= 1

    if secondary_confidence > 50:
        secondary_confidence_level = secondary_confidence
    else:
        secondary_confidence_level -= 1
    
    final = frame.copy()
    if len(focus_point) == 2 and confidence_level > 20 or secondary_confidence_level > 20:
        # Final center point here
        cv2.putText(final,"focus",(focus_point[0],focus_point[1]),cv2.FONT_HERSHEY_PLAIN,fontScale=1,color=(0,0,255))
        cv2.circle(final,(focus_point[0],focus_point[1]),radius=10,color=(0,0,255),thickness=1)
        cv2.circle(final,(width/2,height/2),radius=5,color=(0,0,255),thickness=1)
        targetoffset = offset([width/2,height/2],focus_point)
        foundPlot.append(.1)
        # output offset from frame center
        print (targetoffset)
        #draw edge points
        if gateLength != None:
            cv2.circle(final,(focus_point[0]+gateLength/2,focus_point[1]),radius=5,color=(0,255,0),thickness=2)
    else:
        foundPlot.append(0)
    cv2.imshow("final",final)

    stop = timeit.default_timer()
    timesPlot.append(stop-start)
    # time between frame, larger = slower, smaller = faster 
    key = cv2.waitKey(1) 
    #keyboard controls, p for pause, q for quit, other keys can speed up
    if key == ord('q'):
        break
    if key == ord('p'):
        cv2.waitKey(0)
plt.plot(foundPlot, label = 'found')
plt.plot(timesPlot, label = 'time')
plt.plot(gateLengthPlot, label = 'width')
plt.legend()
plt.show()

vid.release()
cv2.destroyAllWindows()