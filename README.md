
---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/chess_undist.JPG "Undistorted"
[image2]: ./output_images/road_undst.JPG "Road Transformed"
[image3]: ./output_images/shadow_mask.JPG "Shadow Masking"
[image4]: ./output_images/masked_binary.JPG "Masking Result"
[image5]: ./output_images/gradients.JPG "Gradient combinations"
[image6]: ./output_images/gradients_comb.JPG "Final Gradient"
[image7]: ./output_images/warped_lines.JPG "Perspective Transform"
[image8]: ./output_images/histogram.JPG "Find Base Points"
[image9]: ./output_images/sliding_boxes.JPG "Lane Finding"
[image10]: ./output_images/final_lane.JPG "Final result"
[video1]: ./final_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how I you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first cells of the notebook.

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

I applied the same process as before to a road image and got the following result:
![alt text][image2]

#### 2. Color transforms and Gradients 

All the code regarding these operations, can be found under the "Color and Gradient Filtering" section of the notebook.

For the color transforms I created a mask to isolate shadows, using the gray color space. Then I used it to remove the shadows from the S channel binary:
![alt text][image3]
![alt text][image4]

For the gradients, I used two combinations. Comb1 used the x and y gradients and Comb2 used magnitude and direction gradients.
![alt text][image5]

Adding Comb1 with Comb2 we get this:
![alt text][image6]



#### 3. Perspective transform 

The code for my perspective transform includes a function called `lane_warp()`, which appears in lines 37 through 45 in the file `lane_finding_images.py`.  The `lane_warp()` function takes as inputs an image (`img`), and outputs the warped image (`top_down`), the source and destination points (`src_pts`,`dst_pts`) and the inverse matrix(`Minv`). I chose to hardcode the source and destination points by trial and error.

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 190, 720      | 320,720       | 
| 588,453       | 320,0         |
| 694,453       | 960,0         |
| 1125,720      | 960,720       |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image7]

#### 4. Finding pixels and polynomial fitting

All the code regarding this part can be found under the "Lane Finding" section of the notebook.

To identify the lines I first used an histogram to find the peaks in pixels at the bottom half of the image.
![alt text][image8]

After I detected the lines base positions, I used a function called `lane_find()` that can be found in lines 137 through 223 in the file `lane_finding_images.py`. The function uses a sliding window algorithm to find the rest of the pixels, fitting a 2nd order polynomial. Like this:
![alt text][image9]


#### 5. Curvature and Offset calculation
I did this in lines 260 through 281 in my code in `lane_finding_images.py`.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this under the "Image pipeline with Curvature and Offset Calculation" section of the notebook. Here is the result:
![alt text][image10]

---

### Pipeline (video)

#### 1. Video Link
The pipeline for the video is a the end of the notebook. It uses functions and objects from the file `lane_finding_video.py`.
I created a new class called `Lane()`, to make the code more functional, and I made some improvements to be able to skip some bad frames when they occur. Smoothing is also used, with weighted averages of past frames.

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

I had a really tough time getting a solid pipeline working. My first tries at the color and gradient thresholds weren't so great and the video ended up with some really bad frames when it caught shadows. To solve this I implemented a system to check for differences in the coeficients and the average distance between lines of current and previous frames. Later on, I perfected the gradients and color binaries and I'm satisfied with the results I got. This pipeline still does a terrible job with the challenge videos, so I guess there's a lot of room for improvement here. I've also seen other project videos on youtube, and I understood that there's really a LOT of features you can add to make this more robust. One thing I got curious about but didn't have time to work on was using convolution to search for lines.
