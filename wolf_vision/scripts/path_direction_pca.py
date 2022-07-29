#!/usr/bin/env python
from TesterCodes.OpenCV.PathProject.path_direction import FORWARD_DEFAULT
from TesterCodes.OpenCV.PathProject.path_direction_video import NUM_OF_COLORS
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros

import img_prep

class path_direction:
    SCALING_FACTOR = 0.5

    NOISE_PROPORTION = 0.005 # threshold of the area in image as path (less means it is noise)
    FORWARD_DEFAULT = [0,-1] # image up is forward
    # thresholds for tuning
    PATH_COLOR_LOW_THRES, PATH_COLOR_UP_THRES = 65, 90
    PATH_WIDTH_LOW_THRES, PATH_WIDTH_UP_THRES = 70, 600      # between means it is path
    NUM_OF_COLORS = 4

    is_debug = False
    show_window = False
    focal_length = 381.36115

    def __init__(self):
        self.img_prep = img_prep.ImagePrep(slice_size = 25)
        if self.is_debug:
            self.property_pub = rospy.Publisher("wolf_vision/path/properties", String, queue_size=10)
            self.final_pub = rospy.Publisher("wolf_camera2/image_final", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("wolf_camera2/image_raw", Image, self.frame_callback)
        self.bridge = CvBridge()
    ### 
    #   Functions
    ###
    # input: binary image
    # output: mean: center of the coordinates, 
    #         pca_vector: [[PC2_x, PC1_x], [PC2_y, PC1_y]]
    def Path_PCA(self, image):                       # definition method
        pca_vector = []
        #image = cv2.resize(image,IMAGE_SIZE)
        coords_data = np.array(cv2.findNonZero(image)).T.reshape((2,-1))            # 2 x n matrix of coords [[x1,x2,...],[y1,y2,...]]
        mean = np.mean(coords_data,axis=1,keepdims=True)                         # center of coords
        cov_mat = np.cov(coords_data - mean, ddof = 1)              # find covariance
        pca_val, pca_vector = np.linalg.eig(cov_mat)                # find eigen vectors (also PCA first and second component)
        return mean, pca_vector, pca_val

    # changes the value above the line in an image
    # input: image_mask, initial_coord[x,y], slope[x,y], value= 0,1 (for binary masking)
    def set_mask(self, image_mask, initial_coord, slope, value):
        # line_y = mx+b
        # b = line_y - mx
        m = slope[1] / slope[0]
        b = initial_coord[1] - m * initial_coord[0]
        for x in range(image_mask.shape[1]):
            # compute y
            line_y = int(round(m*x + b))
            # bound y within image height
            if line_y > image_mask.shape[0]: line_y = image_mask.shape[0]
            if line_y <= 0: line_y = 1
            # change value of under the line (top of the image)
            image_mask[0:line_y, x] = value 
        return image_mask

    # just for finding the place to draw the circle
    def compute_location(self, pca_cent, pca_dir, scale = 10):
        return (int(pca_cent[0] + scale * pca_dir[0]),
                int(pca_cent[1] + scale * pca_dir[1]))

    # compute angle between two vectors
    # arccos((unit_a dot unit_b))
    def compute_angle(self, v_1, v_2):
        unit_v_1 = v_1 / np.linalg.norm(v_1)
        unit_v_2 = v_2 / np.linalg.norm(v_2)
        return np.arccos(np.dot(unit_v_1,unit_v_2))
    
    def compute_slope(self, p_1, p_2):
        return (p_2[0] - p_1[0]), (p_2[1] - p_1[1])

    def frame_callback(self, data: Image):
        # get image
        self.bridge = CvBridge()
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        ####
        #   Filter Image to Binary Image
        ####
        
        # resize
        width = int(frame.shape[1] * self.SCALING_FACTOR)
        height = int(frame.shape[0] * self.SCALING_FACTOR)
        frame = cv2.resize(frame, (width, height))
        # blur to remove grid on black tiles
        frame = cv2.medianBlur(frame, 3)
        #simplify image colors
        slice_imgs = self.img_prep.slice(frame)
        kmeans = slice_imgs.copy()
        comb_row = [i for i in range(len(slice_imgs))]
        for i,row in enumerate(slice_imgs):
            for j,block in enumerate(row):
                kmeans[i][j], _ = self.img_prep.reduce_image_color(block, 2)
            comb_row[i] = (self.img_prep.combineRow(kmeans[i]))
        combined_filter = self.img_prep.combineCol(comb_row)
        #combined_filter[:,:,1:3] = 0    # only blue channel is relevant (clear GR in BGR)
        filter_final, colors = self.img_prep.reduce_image_color(combined_filter, NUM_OF_COLORS)  # reduce colors (background (black & white tiles, waves, random stuff) and path)
        gray = cv2.cvtColor(filter_final, cv2.COLOR_BGR2GRAY) 

        if self.show_window:
            cv2.imshow('original', frame)
            cv2.imshow('testslice',filter_final)
            cv2.imshow('gray',gray)
            cv2.waitKey(10)
        ####
        #   Find Path Directions
        ####
        # adaptively find the path color
        gray_colors, gray_counts = np.unique(gray.flatten(),return_counts=True)                 # find the colors and counts of each color
        
        if len(gray_colors) < 2:
            if self.is_debug:
                self.property_pub.publish("no path in image (only one color)")
            return  # need to stop the current callback

        img_size = sum(gray_counts)
        background_color = gray_colors[np.argsort(gray_counts)[-1]]             # most common color
        gray_counts[gray_counts < img_size * self.NOISE_PROPORTION] = img_size       # mark noise color (takse too little of the image)
        for i,color in enumerate(gray_colors):                                  # mark white and black tiles
            if color < self.PATH_COLOR_LOW_THRES or color > self.PATH_COLOR_UP_THRES:
                gray_counts[i] = img_size
        path_color = gray_colors[np.argsort(gray_counts)[0]]                                    # find the least common color
        path_size = gray_counts[np.argsort(gray_counts)[0]]

        # simple threshold, use the least frequent color (which should not be the background)
        thres = np.uint8(np.where(gray == path_color, 255, 0)) # produce binary image for the path color found
        #thres = cv2.medianBlur(thres,31)                       # remove the edges

        input_shape = thres.shape

        # compute Principle Components
        # to find line between two paths
        center1, pca_vector_1, pca_val = self.Path_PCA(thres)
        
        # check the orientation with overall PCA
        # drawn as red and green arrows if show_window is enabled
        first_pass_path_vector = pca_vector_1[:,np.argmax(pca_val)]
        first_pass_path_variance = pca_vector_1[:,np.argmin(pca_val)]
        if first_pass_path_vector[1] > 0:
            first_pass_path_vector = -first_pass_path_vector
        
        # if this angle is desirable output (not necessary since there is another method)
        # might be useful to compare the two in very unusual situations
        #first_pass_path_angle = self.compute_angle(FORWARD_DEFAULT, first_pass_path_vector)

        slice_dir = np.argmin(pca_val)  # slice the path using PC2
        # Create the masks to separate two paths
        mask_one = np.ones(input_shape, dtype="uint8")                                   # generate mask
        mask_one = self.set_mask(mask_one, center1[:,0], pca_vector_1[:,slice_dir], 0)       # set above 0
        mask_two = np.zeros(input_shape, dtype="uint8")                                  # generate mask
        mask_two = self.set_mask(mask_two, center1[:,0], pca_vector_1[:,slice_dir], 1)       # set above 1
        # generate the two path segments
        bottom_path = cv2.bitwise_and(thres,thres,mask=mask_one)
        top_path = cv2.bitwise_and(thres,thres,mask=mask_two)

        if self.show_window:
            cv2.imshow('mask1_path',bottom_path)
            cv2.imshow('mask2_path',top_path)

        # Compute Principle Components for both path segments (center point(mean), direction vector(eigvec), variance vector(eigval))
        path_center1, path_direction1, pca_val1 = self.Path_PCA(bottom_path)
        path_center2, path_direction2, pca_val2 = self.Path_PCA(top_path)
        
        # select highest variance for each(eigenvalue)
        #bot_dir = path_direction1[:,np.argmax(pca_val1)]
        #top_dir = path_direction2[:,np.argmax(pca_val2)]
        
        # find angle of bottom direction and top direction with respect to up
        # bot_angle and top_angle aren't necessary for straight paths, but useful if the top and bottom point to different directions
        #bot_angle = self.compute_angle(self.FORWARD_DEFAULT, bot_dir)
        #top_angle = self.compute_angle(self.FORWARD_DEFAULT, top_dir)
        
        # overall center
        path_hori_cent, path_vert_cent = int(center1[0][0]), int(center1[1][0])
        # center of two segments (start location)
        bot_hori_cent, bot_vert_cent = int(path_center1[:,0][0]), int(path_center1[:,0][1])
        top_hori_cent, top_vert_cent = int(path_center2[:,0][0]), int(path_center2[:,0][1])

        
        
        # compute end location of two segment directions
        path_direction = self.compute_slope((bot_hori_cent, bot_vert_cent),(top_hori_cent, top_vert_cent))
        path_angle = self.compute_angle(self.FORWARD_DEFAULT,path_direction)

        if self.property_pub:
            # these information might be useful for more complex logic as well
            # [path_color, background_color, theta, size, bot_location, top_location]
            current_path_properties = [path_color, background_color, path_angle, path_size/img_size, path_hori_cent, path_vert_cent, bot_hori_cent, bot_vert_cent, top_hori_cent, top_vert_cent]
            self.property_pub.publish(str(current_path_properties))
        
        if self.show_window:
            first_pass_end_location = self.compute_location((path_hori_cent, path_vert_cent), first_pass_path_vector,scale= np.max(pca_val)/np.sum(pca_val)*30)
            first_pass_variance_end_location = self.compute_location((path_hori_cent, path_vert_cent), first_pass_path_variance, scale = np.min(pca_val)/np.sum(pca_val)*30)
            cv2.arrowedLine(frame,(path_hori_cent, path_vert_cent),first_pass_end_location,
                            color=(0,0,255),thickness=1,tipLength=0.2)
            cv2.arrowedLine(frame,(path_hori_cent, path_vert_cent),first_pass_variance_end_location,
                            color=(0,255,0),thickness=1,tipLength=0.2)
        
        ####
        #   determine movement of the robot
        #
        #   needs more logic and testing in simulation
        ####

        path_width = np.min(pca_val)
        # path found
        if path_width > self.PATH_COLOR_LOW_THRES and path_width < self.PATH_WIDTH_UP_THRES and path_color > self.PATH_COLOR_LOW_THRES and path_color < self.PATH_COLOR_UP_THRES:
            
            turn_direction = top_hori_cent - bot_hori_cent

            path_transform = TransformStamped()
            path_transform.header.stamp = rospy.Time.now()
            path_transform.header.frame_id = "base_link"
            path_transform.child_frame_id = "path"
            # distance between path center and image center, and normalized between [-0.25, 0.25]
            # did NOT consider when the top part of path goes below the image center
            path_transform.transform.translation.x = (path_vert_cent - height/2) / (2 * height)
            path_transform.transform.translation.y = (path_hori_cent - width/2) / (2 * width)
            path_transform.transform.translation.z = 0.0
            path_transform.transform.rotation.x = 0
            path_transform.transform.rotation.y = 0
            # set the rotation in radians, should be between [0, pi/2]
            if (turn_direction > 0):
                # turn right with respect to z axis (?)
                path_transform.transform.rotation.z = path_angle
            else:
                #turn left
                path_transform.transform.rotation.z = -path_angle
            path_transform.transform.rotation.w = 1
            
            tf2_ros.TransformBroadcaster().sendTransform(path_transform)

            if self.show_window:
                cv2.arrowedLine(frame,(bot_hori_cent, bot_vert_cent),(top_hori_cent, top_vert_cent),
                            color=(255,255,255),thickness=2,tipLength=0.2)

                cv2.putText(frame, "found path:", (0,20), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(0,255,0))
                cv2.putText(frame, "color: {diff}    width: {var:.2f}".format(diff = path_color, var = path_width), (0,40), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(0,255,0))
                cv2.arrowedLine(frame,(bot_hori_cent, bot_vert_cent),(top_hori_cent, top_vert_cent),
                        color=(255,255,255),thickness=2,tipLength=0.2)

                if (path_hori_cent < width/2):        
                    cv2.putText(frame, "move left (x pos): {loc}".format(loc = path_hori_cent),
                                (0,60), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255,255,255))
                else:
                    cv2.putText(frame, "move right(x pos): {loc}".format(loc = path_hori_cent),
                                (0,60), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255,255,255))

                if (turn_direction > 0):
                    cv2.putText(frame, "rotate right(turn rad): {theta}".format(theta = path_angle),
                                (0,80), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255,255,255))
                else:
                    cv2.putText(frame, "rotate left(turn rad): {theta}".format(theta = -path_angle),
                                (0,80), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255,255,255))
                cv2.imshow('final', frame)
        # no path found
        else:
            if self.show_window:
                cv2.putText(frame, "no path", (0,20), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(0,0,255))
                cv2.putText(frame, "color: {diff}    width: {var:.2f}".format(diff = path_color, var = path_width), (0,40), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(0,0,255))
                cv2.imshow('final', frame)