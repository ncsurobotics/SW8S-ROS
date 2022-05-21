#!/usr/bin/env python
import rospy
import rospkg
import cv2
import math
import numpy as np
from functools import cmp_to_key
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

class gate_detector:
    focal_length = 381.36115
    classes = []
    is_debug = False
    confidence_threshold = 0.1
    gates = [(0,0,0),(0,0,0)] #(right_gate, left_gate) where gate = (x,y,confidence)

    def __init__(self):
        self.image_sub = rospy.Subscriber("wolf_camera1/image_raw", Image, self.frame_callback)
        self.bridge = CvBridge()

        #load the YOLO model
        rospack = rospkg.RosPack()
        root_path = rospack.get_path("wolf_vision")
        rospy.logwarn(root_path)
        self.classes = open(root_path + "/models/gate.names").read().strip().split('\n')
        self.net = cv2.dnn.readNetFromDarknet(root_path + "/models/yolov4-tiny-gate.cfg", 
                                        root_path + "/models/yolov4-tiny-gate_1000.weights")
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
        #net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA)

    def offset(self, origin, target):
        x = target[0] - origin[0]
        y = origin[1] - target[1]
        return [x,y]

    def frame_callback(self, data: Image):
        #convert to an openCV image
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = frame.shape
        
        #reset gate data
        self.gates = [(0,0,0),(0,0,0)]
        #convert openCV image to blob (CNN input)
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        
        #get the output layer of the network
        last_layer = self.net.getLayerNames()
        last_layer = [last_layer[i - 1] for i in self.net.getUnconnectedOutLayers()]

        #run the network
        self.net.setInput(blob)
        outputs = self.net.forward(last_layer)

        #filter until we get only at most two targets (left gate and right gate)
        #YOLO has three outputs (large objects, medium, large)
        #and each output has a list of objects it detected
        for output in outputs:
            for detection in output:
                #the actual confidence for each class is everything after the first 5
                #elements of the detection vector, rest is just bounding box info/metadata
                scores =  detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                #ensure that the confidence is past our minimum
                #and is more than any other detected in this frame
                if confidence > self.confidence_threshold and confidence > self.gates[0][2]:
                    box = detection[:4] * np.array([width, height, width, height])
                    (centerX, centerY, bw, bh) = box.astype("int")
                    x = centerX
                    y = centerY
                    self.gates[0] = (x, y, confidence)
                elif confidence > self.confidence_threshold and confidence > self.gates[1][2]:
                    box = detection[:4] * np.array([width, height, width, height])
                    (centerX, centerY, bw, bh) = box.astype("int")
                    x = centerX
                    y = centerY
                    self.gates[1] = (x, y, confidence)
        
        #sort gates found by their position on the x axis
        self.gates = sorted(self.gates, key = cmp_to_key(lambda a,b: a[0] - b[0]))

        #make a TF2 frame for the gate
        if self.gates[0][2] > 0:
            # offset from the center of image
            targetoffset = self.offset([int(width/2),int(height/2)], self.gates[0][:2])
            
            #convert offset into angles to target
            angle_to_gate = [math.atan(targetoffset[0] / self.focal_length), math.atan(targetoffset[1] / self.focal_length)]
            gate_transform = TransformStamped()
            gate_transform.header.stamp = rospy.Time.now()
            gate_transform.header.frame_id = "base_link"
            gate_transform.child_frame_id = "gate"
            gate_transform.transform.translation.x = -math.cos(angle_to_gate[0])
            gate_transform.transform.translation.y = math.sin(angle_to_gate[0])
            gate_transform.transform.translation.z = 0.0
            gate_transform.transform.rotation.x = 0
            gate_transform.transform.rotation.y = 0
            gate_transform.transform.rotation.z = 0
            gate_transform.transform.rotation.w = 1
            tf2_ros.TransformBroadcaster().sendTransform(gate_transform)
        if self.gates[1][2] > 0:
            # offset from the center of image
            targetoffset = self.offset([int(width/2),int(height/2)], self.gates[1][:2])
            
            #convert offset into angles to target
            angle_to_gate = [math.atan(targetoffset[0] / self.focal_length), math.atan(targetoffset[1] / self.focal_length)]
            gate_transform = TransformStamped()
            gate_transform.header.stamp = rospy.Time.now()
            gate_transform.header.frame_id = "base_link"
            gate_transform.child_frame_id = "lgate"
            gate_transform.transform.translation.x = -math.cos(angle_to_gate[0])
            gate_transform.transform.translation.y = math.sin(angle_to_gate[0])
            gate_transform.transform.translation.z = 0.0
            gate_transform.transform.rotation.x = 0
            gate_transform.transform.rotation.y = 0
            gate_transform.transform.rotation.z = 0
            gate_transform.transform.rotation.w = 1
            tf2_ros.TransformBroadcaster().sendTransform(gate_transform)

        if self.is_debug:
            cv2.circle(frame, (self.gates[0][0], self.gates[0][1]), radius=15, color=(0,0,255), thickness=2)
            cv2.circle(frame, (self.gates[1][0], self.gates[1][1]), radius=15, color=(0,0,255), thickness=2)
            cv2.imshow("gate", frame)
            cv2.waitKey(2)


if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    detector = gate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("stopping YOLO")
    cv2.destroyAllWindows()