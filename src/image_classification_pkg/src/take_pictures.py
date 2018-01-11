import cv2
import sys
import matplotlib
import numpy
import sklearn
import time
from time import sleep


def end_program():
    cv2.destroyAllWindows()
    vc.release()
    sys.exit(0)




cpt = 50
maxFrames = 90  # if you want 5 frames only.
b_start = False
rval = False

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False


while rval:  # don't even start if the image is not read correctly
    rval, frame = vc.read()  # read the image again at each loop
    key = cv2.waitKey(20)

    if key == 27:  # exit on ESC
        end_program()
    elif key == 32:  # continue on space
        break
    cv2.imshow("preview", frame)


while rval: # don't even start if the image is not read correctly
    # insert code here for changing the image to: 1) B/W; 2) thresholded; 3) contours

    cv2.imshow("preview", frame)
    rval, frame = vc.read() # read the image again at each loop
    key = cv2.waitKey(20)

    if key == 27: # exit on ESC
        end_program()

    if cpt < maxFrames:

        rval, frame = vc.read() # read frame and return code.
        if not rval:  # if return code is bad, abort.
            end_program()
        cv2.imshow("preview", frame)

        cv2.imwrite("../data_set\SQU%03i.jpg" % cpt, frame)
        cpt += 1
        sleep(0.2)
        print cpt
    else:
        end_program()

