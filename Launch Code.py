import matplotlib.pyplot as plt
import Monocular__Visual__Odometry
import numpy as np
import cv2 as cv
import os





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