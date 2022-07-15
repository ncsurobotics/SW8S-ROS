import cv2
import numpy as np

class ImagePrep:
    KMEANSFILTER = [3,  # num of clusters
                4,  # num of iterations
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0), # criteria
                cv2.KMEANS_PP_CENTERS]  # flag

    def __init__(self, slice_size = 10, kmeans_filter = KMEANSFILTER):
        self.slice_size = slice_size
        self.k, self.iter_num, self.criteria, self.flag = kmeans_filter

    def slice(self, image):
        arr_size = tuple(int(element / self.slice_size) for element in image.shape)
        col_array = np.array_split(image, arr_size[0], axis=0)
        img_array = []
        for col in col_array:
            img_array.append(np.array_split(col,arr_size[1],axis=1))
        return img_array

    def combineRow(self, imgs):
        combined_img = imgs[0]
        for img in imgs[1:]:
            combined_img = np.concatenate((combined_img,img),axis=1)
        return combined_img

    def combineCol(self, imgs):
        combined_img = imgs[0]
        for img in imgs[1:]:
            combined_img = np.concatenate((combined_img,img),axis=0)
        return combined_img

    def reduce_image_color(self, image, ncluster = None):
        img_kmean = image.reshape(-1,3)
        img_kmean = np.float32(img_kmean)
        if ncluster is not None:
            ret,label,center = cv2.kmeans(img_kmean,ncluster,None,self.criteria,self.iter_num,self.flag)
        else:
            ret,label,center = cv2.kmeans(img_kmean,self.k,None,self.criteria,self.iter_num,self.flag)
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((image.shape))
        return res2, center

# This file provides some utilities needed to process an image
# before labelling the features

#class ImagePrep:
#    ### Below are parameters that may be necessary to change 
#    ### to suit for creating images best for labelling
#    
#    # kmean - parameters to run cv2.kmeans()
#    k_kmean = 5
#    criteria_kmean = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
#    flag_kmean = cv2.KMEANS_PP_CENTERS
#    filter_kmean = True
#
#    # canny - parameters for cv2.Canny()
#    t1_canny = 7
#    t2_canny = 21
#    aperture_canny = 3
#    L2gradient_canny = True
#
#    # colors - parameters for color manipulation
#    dist_thres_color_diff = 20.5    # 3d space distance for colors, higher/lower depending on visibility
#                            #10 for GateCcomp.mp4
#    def __init__(self):
#        # slice - parameter defines how image should be sliced
#        self.img_dim_slice = [210,210] # approximate dimension for image
#        self.block_dim_slice = 21 # dimension for each square slices
#        self.num_seg = np.array([ int(self.img_dim_slice[0] / self.block_dim_slice),
#                                int(self.img_dim_slice[1] / self.block_dim_slice)]) 
#    
#    # slice the image into segments each of square blocks
#    # automatically adjust the input image dimension  
#    # returns a 2d matrix of image blocks, row x col
#    def slice(self,img):
#        seg_dim = self.num_seg
#        block_dim = self.block_dim_slice
#        # adjust image dimension
#        if (seg_dim is not None):
#            resize_dim = seg_dim * block_dim
#            img = cv2.resize(img, tuple(self.img_dim_slice))
#        # prepare 2d matrix to hold images
#        blocks = np.empty([seg_dim[1], seg_dim[0]], dtype=np.ndarray)
#        # extract blocks
#        for i in range(seg_dim[0]):
#            x1 = i * block_dim
#            x2 = x1 + block_dim
#            for j in range(seg_dim[1]):
#                y1 = j * block_dim
#                y2 = y1 + block_dim
#                block = img[y1:y2, x1:x2]
#                blocks[j][i] = block
#        return blocks
#    # combine an numpy array of imgs into 1 row, or 1 column
#    def combineRow(self,imgs):
#        combined_img = imgs[0]
#        for img in imgs[1:]:
#            combined_img = np.concatenate((combined_img,img),axis=1)
#        return combined_img
#    def combineCol(self,imgs):
#        combined_img = imgs[0]
#        for img in imgs[1:]:
#            combined_img = np.concatenate((combined_img,img),axis=0)
#        return combined_img
#
#    # k-means, the same one in the OpenCV documentation
#    # default parameters limit the image into 3 colors
#    # works better with smaller images
#
#    # return the label(pixels) and corresponding colors(center)
#    def kmeans(self,img,k = k_kmean,criteria = criteria_kmean,flag = flag_kmean,filter = filter_kmean):
#        img_kmean = img.reshape(-1,3)
#        img_kmean = np.float32(img_kmean)
#        ret,label,center = cv2.kmeans(img_kmean,k,None,criteria,4,flag)
#        if filter is True:
#            label,center = self.combineColors(center,label)
#        return label,center
#    def drawKmeans(self,label,center,imgShape):
#        center = np.uint8(center)
#        res = center[label.flatten()]
#        res2 = res.reshape((imgShape))
#        return res2
#
#    # modify the labels by combining similar colors using a color list
#    # if color 1 and 2 are similar, then all pixels labeled 2 becomes 1
#    def combineColors(self, colors, labels):
#        labels = labels.T
#        for i,colori in enumerate(colors):
#            for j,colorj in enumerate(colors[i+1:]):
#                j += 1
#                if self.compareColorDiff(np.vstack((colori,colorj))) is False:
#                    labels[0][labels[0]==j] = i # replace all j with i in labels
#                    colors[j] = colors[i]
#                if len(np.unique(labels[0])) == 1:
#                    break
#        labels = labels.T
#        return labels,colors
#
#    # true if colors are different, compare the distance between colors
#    def compareColorDiff(self, colors, dist_threshold = dist_thres_color_diff):
#        dist = np.linalg.norm(colors[0]-colors[1])
#        if dist > dist_threshold:
#            return True
#        return False
#    
#    # return the biggest diameter between two colors
#    def colorsMaxDist(self,colors, skip = False):
#        max_dist = 0
#        for i,c1 in enumerate(colors):
#            for c2 in colors[i+1:]:
#                dist = np.linalg.norm(c2-c1)
#                if dist > max_dist:
#                    max_dist = dist
#        return max_dist
#
#    # finding contours using cv2.Canny()
#    def contour(self, img):
#        img_edge = cv2.Canny(img, self.t1_canny, self.t2_canny,
#                                apertureSize=self.aperture_canny, L2gradient=self.L2gradient_canny)
#        contours, hierarchy = cv2.findContours(img_edge,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#        return contours,hierarchy
#
#    # create the images
#    def drawContour(self, img, contours, random_color = True):
#        for i in range(len(contours)):
#            color = self.contourColor(contours[i])
#            img = cv2.drawContours(img,contours,i,color,1)
#        return img
#
#    # return the rotated_rectangle, upright_rectangle and area of the contour
#    def contourProperty(self,contour):
#        rot_rect = cv2.minAreaRect(contour)
#        upright_rect = cv2.boundingRect(contour)
#        area = cv2.contourArea(contour)
#        return rot_rect, upright_rect, area
#
#    # determine a custom color for filtered contours
#    def contourColor(self,contour):
#        rot_rect, up_rect, area = self.contourProperty(contour)
#        angle = abs(rot_rect[2]) % 90 
#
#        gray_scale = 64
#        color = (gray_scale,gray_scale,gray_scale) #gray, normal contour
#        # filters
#        if ( # check dimension of the bounding box based on block dimension
#             any(size >= self.block_dim_slice / 1 for size in up_rect[2:])  
#             and area < 5 
#             and (angle < 20 or angle > 70) ):
#            color = (255,255,255) # white, special contour
#        return color
#