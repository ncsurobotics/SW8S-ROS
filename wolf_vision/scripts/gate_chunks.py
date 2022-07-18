#!/usr/bin/env python
import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
import itertools
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

kernel_size = 100
target_h_value = 100
number_of_valid_kernels = 16
focal_length = 381.36

def convert_kernel_to_full(coord: int) -> int:
    return int((coord * kernel_size) + 0.5 * kernel_size)


def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([[np.cos(theta1), np.sin(theta1)],
                  [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0 = convert_kernel_to_full(x0)
    y0 = convert_kernel_to_full(y0)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return (x0, y0)


def rt_to_xy(rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = convert_kernel_to_full(int(x0 + 1000 * (-b)))
    y1 = convert_kernel_to_full(int(y0 + 1000 * (a)))
    x2 = convert_kernel_to_full(int(x0 - 1000 * (-b)))
    y2 = convert_kernel_to_full(int(y0 - 1000 * (a)))
    return (x1, y1, x2, y2)


def find_gate(data: Image):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    color_sort = []
    kernel_image = np.zeros(
        (int(frame.shape[0] / kernel_size),
         int(frame.shape[1] / kernel_size), 1),
        np.uint8)

# iterate over every sub image of size kernel_size
    for x in range(0, frame.shape[0] - kernel_size, kernel_size):
        for y in range(0, frame.shape[1] - kernel_size, kernel_size):
            sub = frame[x:x + kernel_size, y:y + kernel_size]

            # find the most important colors in the image (stored in center)
            kmeanFrame = sub.reshape((-1, 3))
            kmeanFrame = np.float32(kmeanFrame)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10,
                        1.0)
            K = 4
            ret, label, center = cv2.kmeans(kmeanFrame, K, None, criteria, 10,
                                            cv2.KMEANS_RANDOM_CENTERS)
            center = np.uint8(center)
            center = cv2.cvtColor(np.array([center]), cv2.COLOR_RGB2HSV)

            # apply some blur filters on it
            kernel = np.ones((5, 5), np.uint8)
            morph = sub
            morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)
            morph = cv2.GaussianBlur(morph, (9, 9), 0)

            # get the edges of the image
            edges = cv2.Canny(morph, 10, 70)

            # get lines out of contours
            lines = cv2.HoughLines(edges, 1, np.pi / 690, 40)

            # plt.imshow(edges)
            # plt.show()
            # if we have lines
            if lines is not None:
                # then find every combination of lines iterate through them
                # figure out how many of them are parallel enough (within a threshold)
                lineCombos = itertools.combinations(lines, 2)
                pairType = 0
                theta_threshold = 0.01
                show = 0
                for linePairs in lineCombos:
                    if abs(linePairs[0][0][1] -
                           linePairs[1][0][1]) < theta_threshold:
                        show += 1
                if show > 0:
                    # if we have parallel lines, add this kernels colors to the list
                    for k in range(K):
                        # a tuple of the color + the kernel grid coord to which it belongs
                        color_sort.append(
                            (center[0][k][2], (int(x / kernel_size),
                                               int(y / kernel_size))))

# pick the n closest colors to the target color, those are the valid kernels
    color_sort = sorted(color_sort, key=lambda x: abs(target_h_value - x[0]))
    for i in range(number_of_valid_kernels + 1):
        cx = color_sort[i][1][0]
        cy = color_sort[i][1][1]
        kernel_image[cx][cy] = 255

# in the image made of the kernels, find lines in that image
    kernel_lines = cv2.HoughLines(kernel_image, 1, np.pi / 4, 1)
    threshold = 0.1

    vert_lines = []
    horz_lines = []
    total_lines = 0

# plot them on the image and display that plot
    for line in kernel_lines:
        for rho, theta in line:
            if abs(theta - np.pi / 2.0) < threshold:
                extreme_points = rt_to_xy(rho, theta)
                horz_lines.append(line)
                cv2.line(frame, (extreme_points[0], extreme_points[1]),
                         (extreme_points[2], extreme_points[3]), (255, 0, 0), 2)
            elif abs(theta - 0.0) < threshold:
                extreme_points = rt_to_xy(rho, theta)
                vert_lines.append(line)
                cv2.line(frame, (extreme_points[0], extreme_points[1]),
                         (extreme_points[2], extreme_points[3]), (255, 0, 0), 2)

# if we only have one line, it has to be 0
    target = (0, 0)
    confidence = 0.0

    total_lines = len(vert_lines) + len(horz_lines)

    def average_from_extremes(line_list):
        return (int((line_list[0] + line_list[1]) / 2.0),
                int((line_list[2] + line_list[3]) / 2.0))


# if we only have two lines on the screen either
# one is horz one is vert or
# two are vert
    if total_lines == 2:
        if len(horz_lines) == 1 and len(vert_lines) == 1:
            point0 = intersection(horz_lines[0], vert_lines[0])
            target = (point0[0] + 150, point0[1] + 150)
    elif total_lines == 3:
        if len(horz_lines) == 1 and len(vert_lines) == 2:
            point0 = intersection(horz_lines[0], vert_lines[0])
            point1 = intersection(horz_lines[0], vert_lines[1])
            target = (int(
                (point0[0] + point1[0]) / 2.0), int(
                    (point0[1] + point1[1]) / 2.0) + 100)
# if we have 4 lines
# it should be 3 vert and 1 horz
    elif total_lines == 4:
        if len(horz_lines) == 1 and len(vert_lines) == 3:
            point0 = intersection(horz_lines[0], vert_lines[0])
            point1 = intersection(horz_lines[0], vert_lines[1])
            point2 = intersection(horz_lines[0], vert_lines[2])
            target0 = (int(
                (point0[0] + point1[0]) / 2.0), int(
                    (point0[1] + point1[1]) / 2.0) + 100)
            target1 = (int(
                (point1[0] + point2[0]) / 2.0), int(
                    (point1[1] + point2[1]) / 2.0) + 100)
            target2 = (int(
                (point0[0] + point2[0]) / 2.0), int(
                    (point0[1] + point2[1]) / 2.0) + 100)
            target = min(target0, target1, target2)

    cv2.circle(frame, target, 20, (255, 0, 0), 10)
    cv2.imshow("window", frame)
    cv2.waitKey(1)

    vert_center = int(frame.shape[1] / 2)
    hori_center = int(frame.shape[0] / 2)

    if target != (0,0):
        #################################
        # send the location to robot here
        #################################

        targetoffset = (target[0] - vert_center,0)
        
        #convert offset into angles to target
        angle_to_gate = [math.atan(targetoffset[0] / focal_length), math.atan(targetoffset[1] / focal_length)]

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

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    cv2.namedWindow('window')
    image_sub = rospy.Subscriber("wolf_camera1/image_raw", Image, find_gate)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

