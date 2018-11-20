import numpy as np
import cv2
import matplotlib as plt


# Define a class to receive the characteristics of each line detection
class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = []
        # coefficient values of the last n fits of the line
        self.recent_coef = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        #polynomial coefficients for the most recent fit
        self.current_fit = None
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = None  
        #y values for detected line pixels
        self.ally = None
        
    
    def find_line(self, binary_warped):
        # Choose the number of sliding windows
        nwindows = 7
        # Set height of windows
        window_height = np.int(binary_warped.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        x_current = self.line_base_pos
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 70
        # Create empty lists to receive line pixel indices
        line_inds = []
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y 
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_x_low) &  (nonzerox < win_x_high)).nonzero()[0]
            # Append these indices to the lists
            line_inds.append(good_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_inds) > minpix:
                x_current = np.int(np.mean(nonzerox[good_inds]))
        # Concatenate the arrays of indices
        line_inds = np.concatenate(line_inds)
        # Extract pixel positions
        self.allx = nonzerox[line_inds]
        self.ally = nonzeroy[line_inds] 
        # Fit a second order polynomial to each
        fit = np.polyfit(self.ally, self.allx, 2)
        return fit[:3]
    
    def fitx(self, y, fit):
        return fit[0]*y**2 + fit[1]*y + fit[2]
    
    def set_current(self, dist_diff, coef_diff, fit, ploty): 
        
        if (coef_diff < 60) & (dist_diff < 30):
            #Append coefs 
            self.recent_coef.append(fit)
            if len(self.recent_coef) > 5:
                del self.recent_coef[0]
            #Calculate coef average and set as current
            self.best_fit = np.average(np.array(self.recent_coef), axis=0,
                                       weights=np.exp(np.array(range(len(self.recent_coef),0,-1))))
            self.current_fit = self.best_fit
        elif self.current_fit is None:
            self.current_fit = fit  
        
        # finding x values for current coef
        fitx = self.fitx(ploty, self.current_fit)
        
        #Append x values to queue
        self.recent_xfitted.append(fitx) 
        if len(self.recent_xfitted) > 5:
            del self.recent_xfitted[0]
        
        # Find weighted average of last 5 fits
        self.bestx = np.average(np.array(self.recent_xfitted), axis=0, weights=range(len(self.recent_xfitted),0,-1))    
        
        
# Define a class to represent a lane
class Lane():
    
    def __init__(self):
        self.left = Line()
        self.right = Line()
        
    def find_base_pts(self,binary_warped):
        """Takes a histogram of the bottom half of the image,
        finds the peak of the left and right halves of the histogram.
        These will be the starting points for the left and right lines.
        """
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        return (leftx_base, rightx_base)
    
    def find_diffs(self,left_fitx,right_fitx,left_fit,right_fit, ploty):
        """ For this and previous lane measurements, 
        calculate the difference of the average distance between lines 
        and the line coeficient differences.
        """
        avg_dist = abs(np.mean(left_fitx-right_fitx))
        
        #Note: current_fit refers to previous lane measurements
        if (self.left.current_fit is not None) and (self.right.current_fit is not None):
            left_fitx = self.left.fitx(ploty, self.left.current_fit)
            right_fitx = self.right.fitx(ploty, self.right.current_fit) 
            avg_dist_current = abs(np.mean(left_fitx-right_fitx))
            self.left.diffs = self.left.current_fit - left_fit
            self.right.diffs = self.right.current_fit - right_fit
        else:        
            avg_dist_current = avg_dist
            
        dist_diff = abs(avg_dist-avg_dist_current)
        coef_diff = {'left': np.mean(np.absolute(self.left.diffs)),
                     'right': np.mean(np.absolute(self.right.diffs))}
        
        return dist_diff, coef_diff
    
    def find_lines(self, img):

        # get base points for the lane lines
        base_pts = self.find_base_pts(img)
        self.left.line_base_pos = base_pts[0]
        self.right.line_base_pos = base_pts[1]

        # find the line coeficients 
        left_fit = self.left.find_line(img)
        right_fit = self.right.find_line(img)  

        # find x values
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        left_fitx = self.left.fitx(ploty, left_fit)
        right_fitx = self.right.fitx(ploty, right_fit)
           
        # find differences between this and last detections     
        dist_diff, coef_diff = self.find_diffs(left_fitx, right_fitx, left_fit, right_fit, ploty)
   
        # Evaluate diffs and set current line params  
        self.left.set_current(dist_diff, coef_diff['left'], left_fit, ploty)
        self.right.set_current(dist_diff, coef_diff['right'], right_fit, ploty)
  

def lane_warp(img):
    """This function does a perspective transform on the lane image,
    to make it look like a bird's-eye view."""
    src = np.float32([[190,720],[588,453],[694,453],[1125,720]])
    dst = np.float32([[320,720],[320,0],[960,0],[960,720]])
    M = cv2.getPerspectiveTransform(src,dst)
    Minv = cv2.getPerspectiveTransform(dst,src)
    warped = cv2.warpPerspective(img, M, img.shape[1::-1], flags=cv2.INTER_LINEAR)
    return warped, np.int32(src), np.int32(dst), Minv


def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)      
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255       
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def abs_sobel_thresh(img, orient='x',kernel = 3, thresh_min=0, thresh_max=255):    
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 2) Take the derivative in x or y given orient = 'x' or 'y'
    if orient == 'x':
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=kernel)
    else:
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=kernel)
    # 3) Take the absolute value of the derivative or gradient
    abs_sobelx = np.absolute(sobelx)
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    # 5) Create a mask of 1's where the scaled gradient magnitude 
            # is > thresh_min and < thresh_max
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
    # 6) Return this mask as your binary_output imag
    return sxbinary

def mag_thresh(img, sobel_kernel=3, mag_thresh=(0, 255)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = sobel_kernel)
    # 3) Calculate the magnitude
    abs_sobelxy = np.sqrt(sobelx**2 + sobely**2)
    # 4) Scale to 8-bit (0 - 255) and convert to type = np.uint8
    scaled_sobel = np.uint8(255*abs_sobelxy/np.max(abs_sobelxy))
    # 5) Create a binary mask where mag thresholds are met
    sxybinary = np.zeros_like(scaled_sobel)
    sxybinary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1
    # 6) Return this mask as your binary_output image
    return sxybinary

def s_channel_thresh(img, s_thresh_min = 170, s_thresh_max = 255):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1
    return s_binary


def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = sobel_kernel)
    # 3) Take the absolute value of the x and y gradients
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient 
    grad_direction = np.arctan2(abs_sobely, abs_sobelx)
    # 5) Create a binary mask where direction thresholds are met
    grad_binary = np.zeros_like(grad_direction)
    grad_binary[(grad_direction >= thresh[0]) & (grad_direction <= thresh[1])] = 1
    # 6) Return this mask as your binary_output image
    return grad_binary

def radius(ploty,left_fitx,right_fitx):
    y_eval = np.max(ploty)
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, left_fitx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, right_fitx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    return left_curverad, right_curverad

def center_offset(leftx_base, rightx_base, base_length = 1280):
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    left_offset = leftx_base
    right_offset = base_length - rightx_base
    avg_offset = (left_offset + right_offset)/2
    center_offset = -(avg_offset - right_offset)
    center_offset_meters = center_offset*xm_per_pix
    return center_offset_meters
    
#Instantiate lane object
lane = Lane()

 

    