#!/usr/bin/env python
import rospy
import cv2
import math
import random
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

class gate_detector:
    past_point = []
    focus_point = []
    confidence_level = 0
    secondary_confidence_level = 0

    gate_length = 0
    last_gate_length = 0
    id_type = 0
    last_id_type = 0
    reset_focus = 0

    is_debug = False

    #sim values and real values differ GREATLY
    focal_length = 381.36115

    def __init__(self):

        if self.is_debug:
            self.center_pub = rospy.Publisher("wolf_vision/gate_center", String, queue_size=10)
            self.final_pub = rospy.Publisher("wolf_camera1/image_final", Image, queue_size=10)
            self.contour_pub = rospy.Publisher("wolf_camera1/image_contour", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("wolf_camera1/image_raw", Image, self.frame_callback)
        self.bridge = CvBridge()

        #cv2.namedWindow('gate_original')
        #cv2.namedWindow('final')
        #cv2.namedWindow('output')

    # return empty array or [[center1(x,y)],[center2(x,y)], ..., id_type, gateLength]
    def contourProcess(self, img, width, height):
        ### processing img to contours
        
        # apply a blur to reduce # of contours
        # 11 - clean water, 5 - murky water
        blur = cv2.bilateralFilter(img,11,100,100)
        # Canny edge detection
        edge = cv2.Canny(blur,5,21,apertureSize=3,L2gradient=True)
        # produce the contours and place contour over original image
        contours, _ = cv2.findContours(edge,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        out = img.copy()
        for i in range(len(contours)):
            out = cv2.drawContours(out,contours,i,(255*random.random(),0,255*random.random()),1)
        # contour display
        if self.is_debug:
            self.contour_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        cv2.imshow('output',out)

        ### filter contours
        # [[x,y,radius],[x,y,radius]]
        vert_location = []
        hori_location = []
        
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
            
            # vertical & horizontal filter
            if radius > height/12 and (abs(angle) > 145 or abs(angle) < 25) and aspect_ratio > 2.5:
                #print(["lime circle: ",[int(x),int(y)],radius,angle,aspect_ratio])
                # add the center of contours to location
                vert_location.append([int(x),int(y), radius])
                # highlighting contour to different color
                out_filter = cv2.drawContours(out,contours,i,(0,0,255),2)
                out_circle = cv2.circle(out_filter,(int(x),int(y)),radius,(0,255,0),2)
                cv2.imshow('output_circle',out_circle)
            if radius > width/5 and (abs(angle) > 75 and abs(angle) < 95) and aspect_ratio > .3:
                #print(["dark circle: ", [int(x),int(y)],radius,angle,aspect_ratio])
                hori_location.append([int(x), int(y), radius])
                # filtered output window
                out_filter = cv2.drawContours(out,contours,i,(0,0,125),2)
                out_circle = cv2.circle(out_filter,(int(x),int(y)),radius,(0,125,0),2)
                cv2.imshow('output_circle',out_circle)
        ### adding labels
        # simplify the contours of interest
        amount_identified = len(vert_location)
        points_of_interest = []
        dist_gate = 0
        # if there is only 1 vertial contour, check if it overlaps with a horizontal contour, id = 0
        if amount_identified == 1:
            # loops through all horizontal contours, and choose all overlapped, with gate length being the largest
            for curr_hori_contour in hori_location:
                overlapped = self.contourRadiusOverlap(vert_location[0],curr_hori_contour)
                if(overlapped):
                    print("overlap found: ",curr_hori_contour)
                    points_of_interest.append([curr_hori_contour[0],curr_hori_contour[1]])
                    temp_dist_gate = abs(vert_location[0][0] - curr_hori_contour[0])
                    if (temp_dist_gate > math.sqrt(curr_hori_contour[0])):
                        dist_gate = temp_dist_gate
                    else: 
                        dist_gate = curr_hori_contour[2]
            points_of_interest.append(0)
            points_of_interest.append(dist_gate)

        # if there are multiple vertical contour, id = 1
        elif amount_identified > 1:
            # find similar y-value pairs (if height of contour center is less than 60 pixels apart)
            # get the contour centers into pairs 
            y_range = []
            for i in range(amount_identified):
                for j in range(i + 1,amount_identified):
                    if abs(vert_location[i][1] - vert_location[j][1]) < height / 4:
                        y_range.append([i,j])
            
            # find whether the horizontal distance between the pairs is far enough for possible gate poles
            for obj in y_range:
                x_1 = vert_location[obj[0]][0]
                x_2 = vert_location[obj[1]][0]
                y_1, y_2 = vert_location[obj[0]][1], vert_location[obj[1]][1]
                diff = x_1 - x_2
                # attaching labels for "right" and "left" poles
                if abs(diff) > width/6:
                    # x_1 is right of x_2 
                    #if diff > 0:
                    #put center, gate length defined as the last one
                    center_x = (x_1 + x_2)/2
                    center_y = (y_1 + y_2)/2
                    points_of_interest.append([center_x,center_y])
                    dist_gate = abs(diff)/2
            # no pair of vertical contours can be defined as a gate (gatelength still 0)
            if (dist_gate == 0):
                # find overlapping vert-hori contours
                for curr_vert_contour in vert_location:
                    for curr_hori_contour in hori_location:
                        overlapped = self.contourRadiusOverlap(curr_vert_contour,curr_hori_contour)
                        if(overlapped):
                            print("overlap found: ",curr_hori_contour)
                            points_of_interest.append([curr_hori_contour[0],curr_hori_contour[1]])
                            temp_dist_gate = abs(curr_vert_contour[0] - curr_hori_contour[0])
                            if(temp_dist_gate > dist_gate):
                                if (temp_dist_gate > math.sqrt(curr_hori_contour[0])):
                                    dist_gate = temp_dist_gate
                                else: 
                                    dist_gate = curr_hori_contour[2]
            points_of_interest.append(1) #id type
            points_of_interest.append(dist_gate) #store length as the last element

        # no verticals, find the largest horizontal contour, id=2
        else:
            largestLocation = [0,0]
            largestSize = 0
            for cont in hori_location:
                if cont[2] > largestSize:
                    largestLocation = [cont[0],cont[1]]
                    largestSize = cont[2]
            if largestSize > 0:
                points_of_interest.append(largestLocation)
                points_of_interest.append(2) #id type
                points_of_interest.append(largestSize)
        return points_of_interest

    # average of the center (points of interest)
    # input: list of center points. 
    # output: list of averaged x and y
    def averageCenter(self, centers):
        x = 0
        y = 0
        
        if len(centers)!=0:
            for center in centers:
                x += center[0]
                y += center[1]
            return [x/len(centers),y/len(centers)]
        return []

    # offset from the center of the image, in pixels
    def offset(self, origin,target):
        x = target[0] - origin[0]
        y = origin[1] - target[1]
        return [x,y]

    # check if two processed contours overlap when each circled
    # input: point1 & point2, [x,y,radius]
    # output: true/false
    def contourRadiusOverlap(self,point1,point2):
        center_dist = math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
        overlap_dist = abs(point1[2] + point2[2])
        return overlap_dist > center_dist

    def frame_callback(self,data: Image):
        self.bridge = CvBridge()
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = frame.shape
        totalPxl = height*width
        reductionRatio = 1
        while(totalPxl > 100000):
            height = height/2
            width = width/2
            totalPxl = height*width
            reductionRatio *= 2
        # resize to 1/4 the size if the image is large
        resize_dim = (int(frame.shape[1]/reductionRatio),int(frame.shape[0]/reductionRatio))
        frame = cv2.resize(frame,resize_dim)

        gateLength = None
        
        cv2.imshow('gate_original', frame)
        cv2.waitKey(20)
        
        # start processing
        points = self.contourProcess(frame, resize_dim[0], resize_dim[1])
        if len(points) != 0:
            self.gate_length = abs(points.pop(-1)) #pop out the length of the gate
            self.last_id_type = self.id_type
            self.id_type = points.pop(-1)

        # Discard results - if change in gate length exceeds x (5) pixels, stop updating focus point
        # useful to prevent sudden unwanted edges
        if abs(self.gate_length - self.last_gate_length) < width/12 and len(points): 
            discard_result = False
            # discard if the focus point have not been detected for a while
            if len(self.focus_point) and len(points):
                if abs(self.focus_point[1]-self.averageCenter(points)[1]) > height/10 and abs(self.focus_point[0] - self.averageCenter(points)[0]) > width/15:
                    discard_result = True
            elif self.id_type == 2 and self.last_id_type == 2:
                discard_result = True
        else: 
            discard_result = True
        
        # update focus point
        if not discard_result:
            self.past_point = self.averageCenter(points)
            if len(self.past_point) == 2:
                self.focus_point = self.past_point
                self.focus_point[0] = int(self.focus_point[0])
                self.focus_point[1] = int(self.focus_point[1])
            if (self.reset_focus > 10): self.reset_focus = 0
            elif (self.reset_focus > -100): self.reset_focus -= 5
        # if focus point has not been updated for too long (shaky camera)
        elif self.reset_focus == 20:
            self.reset_focus = 0
            self.focus_point.clear()
        else:
            if (self.reset_focus < 0): self.reset_focus += 10
            else: self.reset_focus += 1

        final = frame.copy()
        #print([discard_result,self.focus_point, points, self.reset_focus])
        if len(self.focus_point) == 2:
            # Final center point here
            cv2.putText(final,"focus",(self.focus_point[0],self.focus_point[1]),cv2.FONT_HERSHEY_PLAIN,fontScale=1,color=(0,0,255))
            cv2.circle(final,(self.focus_point[0],self.focus_point[1]),radius=10,color=(0,0,255),thickness=1)
            cv2.circle(final,(int(width/2),int(height/2)),radius=5,color=(0,0,255),thickness=1)
            targetoffset = self.offset([int(width/2),int(height/2)],self.focus_point)
            #draw edge points
            if gateLength != None:
                cv2.circle(final,(int(self.focus_point[0]+gateLength),int(self.focus_point[1])),radius=5,color=(0,255,0),thickness=2)

            #convert offset into angles to target
            angle_to_gate = [math.atan(targetoffset[0] / self.focal_length), math.atan(targetoffset[1] / self.focal_length)]
            if self.is_debug:
                self.center_pub.publish(str(targetoffset) + str(angle_to_gate))

            #make a TF2 frame for the gate
            gate_transform = TransformStamped()
            gate_transform.header.stamp = rospy.Time.now()
            gate_transform.header.frame_id = "base_link"
            gate_transform.child_frame_id = "gate"
            gate_transform.transform.translation.x = math.cos(angle_to_gate[0])
            gate_transform.transform.translation.y = math.sin(angle_to_gate[0])
            gate_transform.transform.translation.z = 0.0
            gate_transform.transform.rotation.x = 0
            gate_transform.transform.rotation.y = 0
            gate_transform.transform.rotation.z = 0
            gate_transform.transform.rotation.w = 1
            tf2_ros.TransformBroadcaster().sendTransform(gate_transform)

        self.last_gate_length = self.gate_length
        cv2.imshow("final",final)

        if self.is_debug:
            self.final_pub.publish(self.bridge.cv2_to_imgmsg(final, "bgr8"))
        
if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    detector = gate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()