#!/usr/bin/python

import cv2
import numpy as np
from optparse import OptionParser
from os.path import isfile, join
import sys, os, math
import csv
writer = csv.writer(open("corners.csv", 'w'))

# Color print
class bcolors:
    HEADER = '\033[95m'
    PLAIN = '\033[37m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def offset(str_, p_offset):
    for i in xrange(p_offset):
        str_ = '...' + str_
    return str_

def hdr(str_, p_offset=0):
    return offset(bcolors.HEADER + str_ + bcolors.ENDC, p_offset)

def wht(str_, p_offset=0):
    return offset(bcolors.PLAIN + str_ + bcolors.ENDC, p_offset)

def okb(str_, p_offset=0):
    return offset(bcolors.OKBLUE + str_ + bcolors.ENDC, p_offset)

def okg(str_, p_offset=0):
    return offset(bcolors.OKGREEN + str_ + bcolors.ENDC, p_offset)

def wrn(str_, p_offset=0):
    return offset(bcolors.WARNING + str_ + bcolors.ENDC, p_offset)

def err(str_, p_offset=0):
    return offset(bcolors.FAIL + str_ + bcolors.ENDC, p_offset)

def bld(str_, p_offset=0):
    return offset(bcolors.BOLD + str_ + bcolors.ENDC, p_offset)
    


# Main class
class CbDetect:
    def __init__(self, filename):

        self.img= cv2.imread(filename)
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.height, self.width, self.channels = self.img.shape
        self.process()

    def process(self):
        print hdr(okb("Processing started"))
        ksize_morph = np.ones((7,7), np.uint8)
        ksize_morph2 = np.ones((11,11), np.uint8)
        ksize_filter = (15,15)
        sigmaX = 2.4
        sigmaY = 2.4

        self.gray = cv2.GaussianBlur( self.gray, ksize_filter, 2, 2 , cv2.BORDER_CONSTANT)

        self.ret, self.bin = cv2.threshold(self.gray, 180, 255, cv2.THRESH_BINARY)
        self.masked = cv2.multiply(self.gray, self.bin)

        self.dst = cv2.Canny(self.bin,130,200,3)
        self.dst = cv2.dilate(self.dst, ksize_morph, iterations=1)
        
        self.corners = np.zeros((self.height, self.width, self.channels), np.uint8)
        contours, hierarchy = cv2.findContours(self.dst,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cont_len = len(contours)
        for i in range(cont_len):
            cnt = contours[i]
            template = cv2.approxPolyDP(cnt,5,True)
            area = cv2.contourArea(cnt)

            if len(template) == 4 and area > 1500 and area < 30000:
                V1 = (template[0][0][1], template[0][0][0])
                V2 = (template[1][0][1], template[1][0][0])
                V3 = (template[2][0][1], template[2][0][0])
                V4 = (template[3][0][1], template[3][0][0])
                self.corners[V1] = 255
                self.corners[V2] = 255
                self.corners[V3] = 255
                self.corners[V4] = 255

        self.corners = cv2.dilate(self.corners, ksize_morph2, iterations=1)
        self.corners = cv2.cvtColor(self.corners, cv2.COLOR_BGR2GRAY);        
        self.ret2, self.corners = cv2.threshold(self.corners,0.1*self.corners.max(),255,0)

        contours, hierarchy = cv2.findContours(self.corners, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
             M = cv2.moments(c)
             x = int(M["m10"] / M["m00"])
             Y = int(M["m01"] / M["m00"])
             point = x, Y
             writer.writerow(point)
             cv2.circle(self.img, (x, Y), 6, (0, 0, 255), -1)

        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', self.img)
        cv2.resizeWindow('image', 1280,1024)

        if cv2.waitKey(0) & 0xff == 27:
            cv2.destroyAllWindows()



# Dispatcher
if __name__ == '__main__':
    class MyParser(OptionParser):
        def format_epilog(self, formatter):
            return self.epilog

    examples = ("")
    parser = MyParser(usage="Usage: detect.py <options>", epilog=examples)
    parser.add_option('-n', '--name', dest='input_name',
        help='specify input image file name')

    (options, args) = parser.parse_args()

    if (options.input_name == None):
        parser.print_help()
        exit(1)

    cb_detect = CbDetect(options.input_name)

