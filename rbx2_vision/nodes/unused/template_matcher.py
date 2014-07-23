#!/usr/bin/env python


""" template_matcher.py - Version 0.1 2013-04-25

    Match a test image against a number of stored templates
    
"""
import cv2.cv as cv
import cv2
import numpy as np
from time import clock
import os, os.path

help_message = "USAGE: template_matcher.py test_image.jpg [<n_pyr>]"

def read_images(path, sz=None):
    """Reads the images in a given folder, resizes images on the fly if size is given.

    Args:
        path: Path to a folder with subfolders representing the subjects (persons).
        sz: A tuple with the size Resizes

    Returns:
        A list [X,y]

            X: The images, which is a Python list of numpy arrays.
            y: The corresponding labels (the unique number of the subject, person) in a Python list.
    """
    c = 0
    X,y,labels = [], [], []
    for dirname, dirnames, filenames in os.walk(path):
        for subdirname in dirnames:
            #print c, subdirname
            subject_path = os.path.join(dirname, subdirname)
            file_index = 0
            for filename in os.listdir(subject_path):
                file_index += 1
                if file_index == 1:
                    continue
                try:
                    im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE)
                    # resize to given size (if given)
                    if (sz is not None):
                        im = cv2.resize(im, sz)
                    X.append(cv2.equalizeHist(np.asarray(im, dtype=np.uint8)))
                    y.append(c)
                    labels.append(subdirname)
                except IOError, (errno, strerror):
                    print "I/O error({0}): {1}".format(errno, strerror)
                except:
                    print "Unexpected error:", sys.exc_info()[0]
                    raise
            c = c+1
    return [X,y, labels]

if __name__ == '__main__':
    import sys
    try:
        test_file = sys.argv[1]
    except:
        
        print help_message
        
    try:
        n_pyr = int(sys.argv[2])
    except:
        n_pyr = 0
        
    snapshot_dir = "/home/patrick/Dropbox/Robotics/ros/rbx2-git/rbx2_vision/data/orl_faces"
    snapshot_size = 100
        
    # Read in the template images        
    [X,y, labels] = read_images(snapshot_dir)
        
    test_image = cv2.imread(snapshot_dir + "/s23/10.pgm", cv2.IMREAD_GRAYSCALE)
    
    # We need a copy of the original image for later work
    test_image_copy = cv2.equalizeHist(test_image.copy())
    
    # Use pyrDown() n_pyr times on the test image.  We only need to do this once for this image.
    #for i in range(n_pyr):
        #image_copy = cv2.pyrDown(image_copy)
    
    # Time how long this is going to take    
    start = clock()
    
    # Track which scale and rotation gives the best match
    maxScore = -1
    
    for i in range(len(X)):
        template = X[i]

        result = cv2.matchTemplate(test_image_copy, template , cv2.TM_CCOEFF_NORMED)   
            
        # Find the maximum value on the result map
        (minValue, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
        
        #print maxValue
    
        if maxValue > maxScore:
            maxScore = maxValue
            best_i = i
            
    
    print "Best match:", labels[best_i], "with score:",  maxScore
            
    # Stop the clock and print elapsed time
    elapsed = (clock() - start) * 1000
    print "Time elapsed: ", elapsed, "ms"
    
    cv2.imshow("Test Image", test_image_copy)
    cv2.imshow("Best Match", X[best_i])
    cv2.moveWindow("Best Match", 200, 0)
    cv2.waitKey(0)
    

    