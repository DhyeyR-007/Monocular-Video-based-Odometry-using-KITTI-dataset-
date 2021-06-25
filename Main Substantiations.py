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
    
    
    



            

