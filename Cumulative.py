#Main Sustantiation

import numpy as np
import cv2
import os


""" 
Here,

FOlen  = Focal length of camera lens 
MainPoint = Principal point of camera = The principal point is the point on the image plane onto which the perspective center is projected. It is also the point from which the focal length of the lens is measured
detector = open CV feature discoverer 

"""


class Monocular__Visual__Odometry(object):
    
    def __init__(self, PathImage, PathPose, FoLen = 718.8560, MainPoint = (607.1928, 185.2157),LucasKanadePara=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)), 
                 detector=cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)):
        
        self.file_path       =   PathImage
        self.detector        =    detector
        self.LucasKanadePara = LucasKanadePara
        self.focal           = FoLen
        self.R               = np.zeros(shape=(3, 3))
        self.t               = np.zeros(shape=(3, 3))
        self.MainPoint       = MainPoint
        self.n_features      = 0
        self.id              = 0
        

        try:
            if not all([".png" in x for x in os.listdir(PathImage)]):
                raise ValueError("Path of Image is incorrect with respect to image file type")
       
        except Exception as x:
            print(x)
            raise ValueError("Present Path of Image is non-existent")




        try:
            with open(PathPose) as M:
                self.pose = M.readlines()
        
        except Exception as x:
            print(x)
            raise ValueError("The PathPose is invalid / probably doesnt point to txt file")

        self.process_frame()


    def SubsequentFrame(self):
        
        return self.id < len(os.listdir(self.file_path)) 


    def Discover(self, img):
        '''Discovering features and analyze in desired format
        
                
        Returns -------- >>> Array of coordinates (x, y) location found keypoint
        '''

        p0 = self.Detector.Discover(img)
        
        return np.array([x.pt for x in p0], dtype=np.float32).reshape(-1, 1, 2)


    def VisionBasedOdometry(self):
       
        if self.n_features < 2000:
            self.p0 = self.Discover(self.bygoneFrame)


        # Calculation of  optical flow between frames        
        self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.bygoneFrame, self.presentFrame, self.p0, None, **self.LucasKanadePara)
    
        self.Oldgood = self.p0[st == 1]
        self.Newgood = self.p1[st == 1]



        if self.id < 2:
            E, _ = cv2.findEssentialMat(self.Newgood, self.Oldgood, self.focal, self.MainPoint, cv2.RANSAC, 0.999, 1.0, None)
            _, self.R, self.t, _ = cv2.recoverPose(E, self.Oldgood, self.Newgood, self.R, self.t, self.focal, self.MainPoint, None)
        else:
            E, _ = cv2.findEssentialMat(self.Newgood, self.Oldgood, self.focal, self.MainPoint, cv2.RANSAC, 0.999, 1.0, None)
            _, R, t, _ = cv2.recoverPose(E, self.Oldgood, self.Newgood, self.R.copy(), self.t.copy(), self.focal, self.MainPoint, None)

            absolute_scale = self.get_absolute_scale()
            if (absolute_scale > 0.1 and abs(t[2][0]) > abs(t[0][0]) and abs(t[2][0]) > abs(t[1][0])):
                self.t = self.t + absolute_scale*self.R.dot(t)
                self.R = R.dot(self.R)

        # Saving total no. of good features
        self.n_features = self.Newgood.shape[0]


    def get_mono_coordinates(self):
        diag = np.array([[-1, 0, 0],
                        [0, -1, 0],
                        [0, 0, -1]])
        adj_coord = np.matmul(diag, self.t)
        return adj_coord.flatten()


    def get_TrueCoordinates(self):
        '''true coordinates of vehicle ------>  np.array => Array in format [x, y, z]
        '''
        return self.TrueCoordinates.flatten()
         
            
    def ImageProcessing(self):
        

        if self.id < 2:
            self.bygoneFrame = cv2.imread(self.file_path +str().zfill(6)+'.png', 0)
            self.presentFrame = cv2.imread(self.file_path + str(1).zfill(6)+'.png', 0)
            self.VisionBasedOdometry()
            self.id = 2
        else:
            self.bygoneFrame = self.presentFrame
            self.presentFrame = cv2.imread(self.file_path + str(self.id).zfill(6)+'.png', 0)
            self.id += 1
            self.VisionBasedOdometry()


    def FetchTrueScale(self):
        '''Scale estimation to multiply translation vectors  ------>   float => Scalar value allowing for scale estimation
        '''
        pose = self.pose[self.id - 1].strip().split()
        
        xprevious = float(pose[3])
        yprevious = float(pose[7])
        zprevious = float(pose[11])
        
        pose = self.pose[self.id].strip().split()
        
        x = float(pose[3])
        y = float(pose[7])
        z = float(pose[11])

        true_vect = np.array([[x], [y], [z]])
        self.TrueCoordinates= true_vect
        prev_vect = np.array([[xprevious], [yprevious], [zprevious]])        
        return np.linalg.norm(true_vect - prev_vect)
    
    
#Launch Code
import Monocular__Visual__Odometry
import cv2 as cv


PathImage = 'Path of image from desktop'
PathPose  =  'Path of poses from desktop'


focal = 718.8560
MainPoint = (607.1928, 185.2157)
RCumulative = np.zeros((3, 3))
tCumulative = np.empty(shape=(3, 1))

# Parameters for lucas kanade 
LucasKanadePara = dict( winSize  = (21,21), criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))


# Arbitrary colours
color = np.random.randint(0,255,(5000,3))

vo = Monocular__Visual__Odometry(PathImage, PathPose, focal, MainPoint, LucasKanadePara)
Trajectory = np.zeros(shape=(600, 800, 3))

# mask = np.zeros_like(vo.current_frame)
# flag = False
while(vo.SubsequentFrame()):

    frame = vo.presentFrame
    cv.imshow('frame', frame)
    k = cv.waitKey(1)
    if k == 27:
        break


    if k == 121:
        flag = not flag
        toggle_out = lambda flag: "On" if flag else "Off"
        print("Flow lines turned ", toggle_out(flag))
        mask = np.zeros_like(vo.old_frame)
        mask = np.zeros_like(vo.presentFrame)

    vo.process_frame()

    print(vo.get_mono_coordinates())

    mono_coord = vo.get_mono_coordinates()
    TrueCoordinates= vo.get_TrueCoordinatesinates()

    print("MeanSquaredError: ", np.linalg.norm(mono_coord - TrueCoordinates))
    print("x: {}, y: {}, z: {}".format(*[str(pt) for pt in mono_coord]))
    print("true_x: {}, true_y: {}, true_z: {}".format(*[str(pt) for pt in TrueCoordinates]))

    draw_x, draw_y, draw_z = [int(round(x)) for x in mono_coord]
    true_x, true_y, true_z = [int(round(x)) for x in TrueCoordinates]

    Trajectory = cv.circle(Trajectory, (true_x + 400, true_z + 100), 1, list((0, 0, 255)), 4)
    Trajectory = cv.circle(Trajectory, (draw_x + 400, draw_z + 100), 1, list((0, 255, 0)), 4)

    cv.putText(Trajectory, 'Actual Position:', (140, 90), cv.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
    cv.putText(Trajectory, 'Red', (270, 90), cv.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 255), 1)
    cv.putText(Trajectory, 'Estimated Odometry Position:', (30, 120), cv.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
    cv.putText(Trajectory, 'Green', (270, 120), cv.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0), 1)

    cv.imshow('Trajectory', Trajectory)
cv.imwrite("/path/to/destination/image.png", Trajectory)
cv.destroyAllWindows()
            
"""
            where
            
            First Argument is Path to the destination on file system, where image is ought to be saved.
            Second Argument is ndarray containing image
            Returns True is returned if the image is written to file system, else False."""