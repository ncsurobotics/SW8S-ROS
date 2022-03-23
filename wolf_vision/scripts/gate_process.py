#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

import img_prep

class gate_detector:

    # configs
    width_filter = 20       # # of pixel width of one pole 

    is_debug = True
    show_window = True # display imshow() windows
    #sim values and real values differ GREATLY
    focal_length = 381.36115

    def __init__(self,
                ncluster = 5,
                color_threshold_prep = 30):

        self.img_prep = img_prep.ImagePrep()
        # update parameters
        self.img_prep.k_kmean = ncluster
        self.img_prep.dist_thres_color_diff = color_threshold_prep
        # generate block array
        self.color_flags = {"furthest_distance":0,"background":True}

        if self.is_debug:
            self.center_pub = rospy.Publisher("wolf_vision/gate_center", String, queue_size=10)
            self.final_pub = rospy.Publisher("wolf_camera1/image_final", Image, queue_size=10)
            self.contour_pub = rospy.Publisher("wolf_camera1/image_contour", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("wolf_camera1/image_raw", Image, self.frame_callback)
        self.bridge = CvBridge()

    # assign color flags to img after kmean
    def kmeanColor(self,colors,color_thres = 50):
        flags = self.color_flags
        flags["furthest_distance"] = self.img_prep.colorsMaxDist(colors)
        if flags["furthest_distance"] < color_thres:
            flags["background"] = True
        else:
            flags["background"] = False
        return flags
    # mask the image based on flags 
    # process see maskImgBlock()
    # axis: 0-individual block, 1-row, 2-col
    def maskImg(self,img,color_thres = 50,axis=0):
        sliced_blocks = self.img_prep.slice(img)
        full_list = []
        for i,row in enumerate(sliced_blocks):
            block_list = []
            for j,block in enumerate(row):
                block = self.maskImgBlock(block,color_thres)
                block_list.append(block)
            full_list.append(self.img_prep.combineRow(block_list))
        full = self.img_prep.combineCol(full_list)
        return full
    
    # blur (optional) -> kmeans -> filter: color_label -> contours 
    def maskImgBlock(self,block,color_thres = 50):

        #blur = cv2.medianBlur(block,5)             # gate is close
        #label,center = self.img_prep.kmeans(blur)
        label,center = self.img_prep.kmeans(block)  # gate is far
        color_flag = self.kmeanColor(center,color_thres)
        if color_flag["background"] is True:
            block = np.zeros(block.shape,dtype="uint8")
        else:
            block = self.img_prep.drawKmeans(label,center,block.shape)
            contours,hierarchy = self.img_prep.contour(block)
            block = np.full(block.shape,8,dtype="uint8")
            self.img_prep.drawContour(block, contours, random_color=False)
        return block

    # return gate center by using max contour values along axis
    # axis: 1 horizontal, 0 vertical
    def findByMaximum(self, img, axis, w_filter = width_filter):
        total_energy = self.sumOfAxis(img, axis = axis) # sum all pixels along an axis
        sort_index = np.argsort(total_energy)        # sort the pixel index by sum
        check_index = np.flip(sort_index[-22:])      # filter the sorted pixel index array
        
        max_filter = 7500
        center = 0
        saved_i = check_index[0]

        if max(total_energy) < max_filter:
            return center           # no valid peaks
        if axis == 0:               # 2 vertical peaks
            for i in check_index:
                if(abs(i-saved_i) > w_filter):
                    center = (i + saved_i)/2    # middle
                    center = (center + min([i,saved_i])) / 2 # middle of the left part
                    break
        else:                       # 1 horizontal peak + offset
            center = check_index[0] + 20
        return int(center)

    def sumOfAxis(self, img, axis=0):
        first_channel = img[:,:,0]
        total_energy = np.sum(first_channel,axis=axis)
        return total_energy

    def frame_callback(self,data: Image):
        self.bridge = CvBridge()
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        if self.show_window: 
            cv2.imshow('original',frame)
            cv2.waitKey(20)

        HSVFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            # NCSU pool config
        mask1_img = self.maskImg(HSVFrame,50,0)
            
            # identified location (x, y)
            # the height location is not reliable
        gate_location = (self.findByMaximum(mask1_img,0),
                        self.findByMaximum(mask1_img,1))
        valid_location = any(gate_location)

            # image center
        vert_center = int(mask1_img.shape[1] / 2)
        hori_center = int(mask1_img.shape[0] / 2)

            # debugging, visualize results (unnecessary for robot)
            # blue circle: image center
            # green circle: gate location
            # red circle: x: gate location, y: image center
        if valid_location and all(gate_location):
            #################################
            # send the location to robot here
            #################################

            targetoffset = (gate_location[0] - vert_center,1)

            if self.show_window:
                cv2.circle(mask1_img,gate_location,5,(0,255,0))
                cv2.circle(mask1_img,(gate_location[0],hori_center),5,(0,0,255))
                cv2.circle(mask1_img,(vert_center,hori_center),3,(255,0,0))

            
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
        
        
        if self.show_window: cv2.imshow('mask1',mask1_img)

        if self.is_debug:
            self.final_pub.publish(self.bridge.cv2_to_imgmsg(mask1_img, "bgr8"))
        
if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    detector = gate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
