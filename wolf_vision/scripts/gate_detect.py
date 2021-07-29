#!/usr/bin/env python
import rospy
import cv2
import math
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

    def contourProcess(self, img):
        ### processing img to contours
        # apply a blur to reduce # of contours
        blur = cv2.bilateralFilter(img,11,100,100)
        # Canny edge detection
        edge = cv2.Canny(blur,5,21,apertureSize=3,L2gradient=True)
        # produce the contours and place contour over original image
        contours, _ = cv2.findContours(edge,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        out = cv2.drawContours(img.copy(),contours,-1,(255,0,0),2)
        # contour display
        if self.is_debug:
            self.contour_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        #cv2.imshow('output',out)

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

                # highlighting & circle filtered contours to different color
                # out_filter = cv2.drawContours(out,contours,i,(0,0,255),2)
                # out_circle = cv2.circle(out_filter,(int(x),int(y)),radius,(0,255,0),2)
                # cv2.imshow('output_circle',out_circle)

        
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
    def averageCenter(self, centers):
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
    def confidence(self, points_of_interest, ppast_point):
        x_conf = 0
        y_conf = 0
        
        if len(ppast_point)==2:
            for curr_point in points_of_interest:
                x_diff = ppast_point[0] - curr_point[0]
                if (100 - abs(x_diff)) > x_conf:
                    x_conf = 100-abs(x_diff)
                y_diff = ppast_point[1] - curr_point[1]
                if (100 - abs(y_diff)) > y_conf:
                    y_conf = 100-abs(x_diff)
                return (x_conf+y_conf)/2

    # offset from the center of the image, in pixels
    def offset(self, origin,target):
        x = target[0] - origin[0]
        y = origin[1] - target[1]
        return [x,y]


    def frame_callback(self,data: Image):
        self.bridge = CvBridge()
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = frame.shape

        # resize to 1/4 the size if the image is large
        if (width < 200):
            frame = cv2.resize(frame, (int(width/2),int(height/2)))

        gateLength = None
        
        #cv2.imshow('gate_original',frame)
        
        
        # start processing
        points = self.contourProcess(frame)
        if len(points) != 0:
            gateLength = points.pop(-1) #pop out the length of the gate
            if gateLength != None:
                gateLength = abs(gateLength)

        # confidence levels
        current_confidence = self.confidence(points, self.past_point)
        secondary_confidence = self.confidence(points, self.focus_point)

        # Show Confidence Levels
        self.past_point = self.averageCenter(points)
        if len(self.past_point) == 2:
            self.focus_point = self.past_point

        if current_confidence is not None and current_confidence > 70:
            self.confidence_level = current_confidence
        else:
            self.confidence_level -= 1

        if secondary_confidence is not None and secondary_confidence > 50:
            self.secondary_confidence_level = secondary_confidence
        else:
            self.secondary_confidence_level -= 1

        # Discard results - if change in gate length exceeds x (5) pixels, stop updating focus point
        # useful to prevent sudden unwanted edges
        if (self.gate_length - self.last_gate_length) < 5: # only update focus point if the length of gate is consistent
            discard_result = False
            # discard if the focus point have not been detected for a while
            if secondary_confidence is not None and secondary_confidence < 70 and secondary_confidence != 0: 
                discard_result = True
            # discard if the focus point will jump too much
            if not discard_result and len(self.focus_point) and len(points) and abs(self.focus_point[0] - self.averageCenter(points)[0]) > 5:
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
        # if focus point has not been updated for too long (shaky camera)
        elif self.reset_focus == 20:
            self.reset_focus = 0
            self.focus_point.clear()
        else:
            self.reset_focus += 1

        final = frame.copy()
        if len(self.focus_point) == 2 and (self.confidence_level > 20 or self.secondary_confidence_level > 20):

            self.focus_point[0] = int(self.focus_point[0])
            self.focus_point[1] = int(self.focus_point[1])

            # Final center point here
            cv2.putText(final,"focus",(self.focus_point[0],self.focus_point[1]),cv2.FONT_HERSHEY_PLAIN,fontScale=1,color=(0,0,255))
            cv2.circle(final,(self.focus_point[0],self.focus_point[1]),radius=10,color=(0,0,255),thickness=1)
            cv2.circle(final,(int(width/2),int(height/2)),radius=5,color=(0,0,255),thickness=1)
            targetoffset = self.offset([int(width/2),int(height/2)],self.focus_point)

            #draw edge points
            if gateLength != None:
                cv2.circle(final,(int(self.focus_point[0]+gateLength/2),int(self.focus_point[1])),radius=5,color=(0,255,0),thickness=2)

            #convert offset into angles to target
            angle_to_gate = [math.atan(targetoffset[0] / self.focal_length), math.atan(targetoffset[1] / self.focal_length)]
            if self.is_debug:
                self.center_pub.publish(str(targetoffset) + str(angle_to_gate))

            #make a TF2 frame for the gate
            gate_transform = TransformStamped()
            gate_transform.header.stamp = rospy.Time.now()
            gate_transform.header.frame_id = "odom"
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