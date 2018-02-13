#!/usr/bin/env python

import rospy
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_inverse
from geometry_msgs.msg import Twist, Pose2D
import math
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import numpy as np
import threading
import copy
import rospkg
from matplotlib import pyplot as plt


#-------------------------------------------------------------------------------
# Object search class
#-------------------------------------------------------------------------------
class ObjectSearch:
    def __init__(self):

        # Navigation
        self.goalStatesText = ['PENDING',
                               'ACTIVE',
                               'PREEMPTED',
                               'SUCCEEDED',
                               'ABORTED',
                               'REJECTED',
                               'PREEMPTING',
                               'RECALLING',
                               'RECALLED',
                               'LOST']

        # Vision
        self.image = []
        self.processImage = False
        self.lock = threading.Lock()

        rospack = rospkg.RosPack()
        self.debugImageDir = rospack.get_path('adventure_recognition') + "/images/debug/"
        self.trainImageDir = rospack.get_path('adventure_recognition') + "/images/train/"
        self.trainImageNames = ['bottle_0a.jpg', 'bottle_1a.jpg', 'bottle_2a.jpg']
        self.thresholds = [25, 15, 10]

        # Initialize node
        rospy.init_node('adventure_recognition')

        # Image subscriber and cv_bridge
        self.imageTopic = "/camera/rgb/image_raw/compressed"
        #self.imageTopic = "/usb_cam/image_raw/compressed" #Webcam
        self.imageSub = rospy.Subscriber(self.imageTopic,CompressedImage,imageCallback)
        self.counter = 0

    def training(self):
        global sift
        self.MIN_MATCH_COUNT = 15
        self.img1 = cv2.imread(self.trainImageDir + self.trainImageNames[self.counter])
        self.kp1, self.des1 = sift.detectAndCompute(self.img1, None)
        self.flann = cv2.BFMatcher()
        self.counter = self.counter + 1


def drawMatches(img1, kp1, img2, kp2, matches):

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = img1

    # Place the next image to the right of it
    out[:rows2,cols1:] = img2

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:

        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (0, 255, 0), 1)
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (0, 255, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (0, 255, 0), 1)

    # Also return the image if you'd like a copy
    return out


    # Image callback
def imageCallback(data):
    global objectSearch
    global sift_stage
    if not sift_stage:
        return

    global frame
    frame += 1
    if (frame == 1) or (frame % 15 == 0):

        # Capture image
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV 3.0

        if objectSearch.processImage:
            print "Capturing image"
            objectSearch.image = copy.deepcopy(cv_image)
            objectSearch.processImage = False

        cv2.waitKey(1)

        kp2, des2 = sift.detectAndCompute(cv_image,None)

        matches = objectSearch.flann.knnMatch(objectSearch.des1, des2, k=2)

        #store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        # At least 10 matches have to be found
        if len(good)>objectSearch.MIN_MATCH_COUNT:
            print "Good Matches! - %d/%d" % (len(good),objectSearch.MIN_MATCH_COUNT)
            src_pts = np.float32([ objectSearch.kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h = objectSearch.img1.shape
            pts = np.float32([ [0,0],[0,h[0]-1],[h[1]-1,h[0]-1],[h[1]-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            cv_image = cv2.polylines(cv_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)

            dist = math.sqrt(pow(dst[0][0][0]-dst[1][0][0],2)+pow(dst[0][0][1]-dst[1][0][1],2))
            dist = abs(dist)
            print "BB",dist

            new_dist = (127*60) / dist
            print "Distance", new_dist

            global velocity_pub
            stop_threshold = objectSearch.thresholds[objectSearch.counter - 1]
            velocity_message = Twist()
            if new_dist > stop_threshold:
                print 'Set speed to 0.1'
                velocity_message.linear.x = 0.1
            else:
                velocity_message.linear.x = 0
                global sift_flag_pub
                sift_stage = False
                sift_flag_pub.publish(Bool(True))

            velocity_pub.publish(velocity_message)

        else:
            print "Not enough matches are found - %d/%d" % (len(good),objectSearch.MIN_MATCH_COUNT)
            matchesMask = None
            velocity_message = Twist()
            velocity_message.angular.z = -0.2
            velocity_pub.publish(velocity_message)

        img3 = drawMatches(objectSearch.img1, objectSearch.kp1, cv_image, kp2, good)

        cv2.imshow("Sift", img3)


def sift_arguments_callback(message):
    if message.data:
        global sift_stage
        sift_stage = True
        print 'Flag Set'
    else:
        print 'Sift Arguments Error'


def navigation_callback(message):
    global objectSearch
    objectSearch.training()


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    frame = 0
    sift_stage = False
    sift = cv2.xfeatures2d.SIFT_create()
    sift_flag_pub = rospy.Publisher('sift_flag', Bool)
    velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)
    rospy.Subscriber('sift_arguments', Bool, sift_arguments_callback)
    rospy.Subscriber('navigation_arguments', Pose2D, navigation_callback)
    objectSearch = ObjectSearch()
    rospy.spin()
