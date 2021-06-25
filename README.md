# Monocular-Video-based-Odometry-using-KITTI-dataset

Aim:
For every pair of images, we need to find the rotation matrix (R) & translation vector (t), which depicts vehicular dynamics (motion) b/w 2 frames of image obrained. Vector t can ony be estimated till a scale factor here

Source:
Source of Image and Poses dataset: http://www.cvlibs.net/datasets/kitti/eval_odometry.php

Implementation Algorithm:

1. Image Capture: Kt and Kt + 1
2. Undistortion of images:(Distortion happens once lines that are straight in reality become arced in images, hence reimbursement lens distortion).
3. Using FAST algorithm to detect features in image It, and tracking these features using method regarding optical flow, to Kt + 1. 
4. Launch a new search of coordiantes if no. of tracked coordinates are < than the threshold(here 2000).
5. Implementing Nister's 5-point algorithm along with RANSAC to find required matrix equation.
6. Estimate R (rotation matrix), t(translation vector) from required matrix established by Nister's algorithm.
7. Acquire scale information from an external source, such as the speedometer, and concatenate t and R.

Results:

![alt text](https://user-images.githubusercontent.com/86003669/123432856-73fcac00-d5e8-11eb-8db2-7841229ca0ae.png)

